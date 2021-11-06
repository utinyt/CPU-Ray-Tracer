///////////////////////////////////////////////////////////////////////
// A framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include "geom.h"
#include "camera.h"
#include "meshData.h"

class Shape;

////////////////////////////////////////////////////////////////////////////////
// Scene
class Realtime;

class Scene {
public:
    int width, height;
    Camera cam;
    Material* currentMat;

    Scene();
    ~Scene();

    void Finit();

    // The scene reader-parser will call the Command method with the
    // contents of each line in the scene file.
    void Command(const std::vector<std::string>& strings,
                 const std::vector<float>& f);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::string& path, const Matrix4f& M);

    // Once ReadAssimpFile parses the information from the model file,
    // it will call:
    void AddMesh(MeshData* mesh);

    // The main program will call the TraceImage method to generate
    // and return the image.  This is the Ray Tracer!
    void TraceImage(Color* image, const int pass);

private:
    Color RayTrace(int x, int y, const KdBVH<float, 3, Shape*>& tree);

    std::vector<Material*> materials;
    std::vector<Shape*> shapes; // shapes + lights
    std::vector<Shape*> lights;
    std::vector<MeshData*> meshes;
};
