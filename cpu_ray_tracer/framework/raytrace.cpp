//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
    #undef min
    #undef max
#else
    // Includes for Linux
#endif

#include "raytrace.h"
#include "material.h"
#include "shape.h"
#include "minimizer.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "utils.h"

const float Radians = PI / 180.0f;    // Convert degrees to radians

Scene::Scene() 
{ 
}

Scene::~Scene()
{
    for (Material*& mat : materials) {
        delete mat;
    }
        
    for (Shape*& shape : shapes) {
        delete shape;
    }
        
    for (MeshData*& mesh : meshes) {
        delete mesh;
    }
        
}

void Scene::Finit()
{
}

void Scene::AddMesh(MeshData* mesh)
{ 
    meshes.emplace_back(mesh);
}

Quaternionf Orientation(int i, 
                        const std::vector<std::string>& strings,
                        const std::vector<float>& f)
{
    Quaternionf q(1,0,0,0); // Unit quaternion
    int size = static_cast<int>(strings.size());
    while (i<size) {
        std::string c = strings[i++];
        if (c == "x")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitX());
        else if (c == "y")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitY());
        else if (c == "z")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitZ());
        else if (c == "q")  {
            q *= Quaternionf(f[i+0], f[i+1], f[i+2], f[i+3]);
            i+=4; }
        else if (c == "a")  {
            q *= angleAxis(f[i+0]*Radians, Vector3f(f[i+1], f[i+2], f[i+3]).normalized());
            i+=4; } }
    return q;
}

void Scene::Command(const std::vector<std::string>& strings,
                    const std::vector<float>& f)
{
    if (strings.size() == 0) return;
    std::string c = strings[0];
    
    if (c == "screen") {
        width = int(f[1]);
        height = int(f[2]); 
    }

    else if (c == "camera") {
        // syntax: camera x y z   ry   <orientation spec>
        // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
        cam.Init(Vector3f(f[1], f[2], f[3]), Orientation(5, strings, f), f[4], width, height);
    }

    else if (c == "ambient") {
        // syntax: ambient r g b
        // Sets the ambient color.  Note: This parameter is temporary.
        // It will be ignored once your raytracer becomes capable of
        // accurately *calculating* the true ambient light.
        //realtime->setAmbient(Vector3f(f[1], f[2], f[3])); 
    }

    else if (c == "brdf")  {
        // syntax: brdf  r g b   r g b  alpha
        // later:  brdf  r g b   r g b  alpha  r g b ior
        // First rgb is Diffuse reflection, second is specular reflection.
        // third is beer's law transmission followed by index of refraction.
        // Creates a Material instance to be picked up by successive shapes
		if (static_cast<int>(f.size()) > 8) {
			Vector3f kd = Vector3f(f[1], f[2], f[3]);
			Vector3f kt = Vector3f(f[8], f[9], f[10]);
			if (kd.norm() > 0 && kt.norm() > 0)
				assert(0);
			materials.push_back(new BRDF(kd, Vector3f(f[4], f[5], f[6]), f[7],
				kt, f[11]));
		}
		else {
			materials.push_back(new BRDF(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7]));
		}
        currentMat = materials.back(); 
    }

    else if (c == "light") {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        materials.push_back(new Light(Vector3f(f[1], f[2], f[3])));
        currentMat = materials.back();
    }
   
    else if (c == "sphere") {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius
        Shape* sphere = new Sphere(Vector3f(f[1], f[2], f[3]), f[4], *currentMat);
        if (currentMat->IsLight()) {
            lights.push_back(sphere);
        }
        shapes.push_back(sphere);
    }

    else if (c == "box") {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        Shape* box = new Box(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), *currentMat);
        if (currentMat->IsLight())
            lights.push_back(box);
        shapes.push_back(box);
    }

    else if (c == "cylinder") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        Shape* cylinder = new Cylinder(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], *currentMat);
        if (currentMat->IsLight())
            lights.push_back(cylinder);
        shapes.push_back(cylinder);
    }

    else if (c == "mesh") {
        // syntax: mesh   filename   tx ty tz   s   <orientation>
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
        Matrix4f modelTr = translate(Vector3f(f[2],f[3],f[4]))
                          *scale(Vector3f(f[5],f[5],f[5]))
                          *toMat4(Orientation(6,strings,f));
        ReadAssimpFile(strings[1], modelTr);
        MeshData* mesh = meshes.back();

        for (TriData& indices : mesh->triangles) {
            shapes.emplace_back(new Triangle(mesh->vertices, indices, *currentMat));
        }
    }

	else if (c == "ibl") {
		materials.push_back(new IBL(strings[1]));
		currentMat = materials.back();
	}

    else if (c == "union") {
        Shape* b = shapes.back();
        if (b->mat.IsLight()) assert(0);
        shapes.pop_back();

        Shape* a = shapes.back();
        if (a->mat.IsLight()) assert(0);
        shapes.pop_back();

        shapes.push_back(new Union(a, b, *currentMat));
    }
    else if (c == "intersect") {
        Shape* b = shapes.back();
        if (b->mat.IsLight()) assert(0);
        shapes.pop_back();

        Shape* a = shapes.back();
        if (a->mat.IsLight()) assert(0);
        shapes.pop_back();

        shapes.push_back(new Intersect(a, b, *currentMat));
    }
    else if (c == "difference") {
        Shape* b = shapes.back();
        if (b->mat.IsLight()) assert(0);
        shapes.pop_back();

        Shape* a = shapes.back();
        if (a->mat.IsLight()) assert(0);
        shapes.pop_back();

        shapes.push_back(new Difference(a, b, *currentMat));
    }

    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}

void Scene::TraceImage(Color* image, const int pass)
{
    std::vector<Shape*> shapes_light(shapes);
    shapes_light.insert(shapes_light.end(), lights.begin(), lights.end());
    KdBVH<float, 3, Shape*> tree(shapes_light.begin(), shapes_light.end());
    for (int i = 0; i < pass; ++i) {
#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
        for (int y = 0; y < height; y++) {
            fprintf(stderr, "Rendering %4d, %4d th pass of total %4d pass\r", y, i+1, pass);
            for (int x = 0; x < width; x++) {
				Color col = RayTrace(x, y, tree);
                if (std::isnan(col.x()) || std::isinf(col.x()) ||
                    std::isnan(col.y()) || std::isinf(col.y()) ||
                    std::isnan(col.z()) || std::isinf(col.z())) {
                    continue;
                }
					
				image[y * width + x] += col;
            }
        }
    }
    fprintf(stderr, "\n");

#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            image[y * width + x] /= static_cast<float>(pass);
        }
    }
}

Color Scene::RayTrace(int x, int y, const KdBVH<float, 3, Shape*>& tree)
{
    Ray ray = cam.GenerateRay(x, y);
    Minimizer initial_minimizer(ray);
    BVMinimize(tree, initial_minimizer);
    
    Vector3f C(0, 0, 0);
    Vector3f W(1, 1, 1);
    Intersection P = initial_minimizer.closest_info;
    Vector3f N = P.normal;
    if (P.shape_hit == nullptr) return C;
    if (P.shape_hit->mat.IsLight()) return P.shape_hit->mat.EvalRadiance(P);
    
	Vector3f wo = -ray.direction;

    const float russian_roulette = 0.8f;
    while (random_float() < russian_roulette) {
        //Explicit light connection
        Intersection L = lights[0]->mat.SampleLight(static_cast<Sphere&>(*lights[0]));
        float p = lights[0]->mat.PdfLight(L) / GeometryFactor(P, L);

        Vector3f wi = L.pos - P.pos;
        wi.normalize();
		float q = P.shape_hit->mat.PdfBrdf(wo, N, wi) * russian_roulette;
		float wmis = p * p / (p * p + q * q);
        ray = Ray(P.pos, wi);
        Minimizer explicit_minimizer(ray);
        BVMinimize(tree, explicit_minimizer);
        Intersection I = explicit_minimizer.closest_info;

        Vector3f f;
        if (p > 0 && I.shape_hit == lights[0]) {
            f = P.shape_hit->mat.EvalScattering(wo, N, wi, P.t);
            Vector3f radiance = lights[0]->mat.EvalRadiance(L);
            Vector3f fp = f / p;
            Vector3f c(radiance.x() * W.x() * fp.x(),
                radiance.y() * W.y() * fp.y(), radiance.z() * W.z() * fp.z());
             C += 0.5f * wmis * c;
        }

        //Extend path
        N = P.normal;
        wi = P.shape_hit->mat.SampleBrdf(wo, N);
        wi.normalize();

        ray = Ray(P.pos, wi);
        Minimizer implicit_minimizer(ray);
        BVMinimize(tree, implicit_minimizer);
        Intersection Q = implicit_minimizer.closest_info;

        if (Q.shape_hit == nullptr) break;
        f = P.shape_hit->mat.EvalScattering(wo, N, wi, P.t);
        p = russian_roulette * P.shape_hit->mat.PdfBrdf(wo, N, wi);
        if (p < EPSILON) break;

        Vector3f fp = f / p;

        W.x() *= fp.x();
        W.y() *= fp.y();
        W.z() *= fp.z();

        //Implicit light connection
        if (Q.shape_hit->mat.IsLight()) {
			float q = Q.shape_hit->mat.PdfLight(Q) / GeometryFactor(P, Q);
			wmis = p * p / (p * p + q * q);
            Vector3f radiance = Q.shape_hit->mat.EvalRadiance(Q);
            Vector3f c(radiance.x() * W.x(), radiance.y() * W.y(), radiance.z() * W.z());
            C += 0.5f * wmis * c;
            break;
        }

        //Step forward
        P = Q;
		wo = -wi;
    }

    return Color(C);
}

