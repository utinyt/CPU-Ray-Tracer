#include "geom.h"
#include "material.h"
#include "utils.h"
#include "ray.h"
#include "shape.h"
#include "utils.h"
#include "stb_image.h"
#include "rgbe.h"

#define GGX // PHONG // BECKMAN

void SetIOR(const Vector3f& wo, const Vector3f& normal,float ior, float& e, float& eo, float& ei) {
	float wo_dot_normal = wo.dot(normal);
	if (wo_dot_normal > 0) {
		ei = 1.f;
		eo = ior;
	}
	else {
		ei = ior;
		eo = 1.f;
	}
	e = ei / eo;
}

BRDF::BRDF(const Vector3f& diff, const Vector3f& spec, float shininess,
	const Vector3f& transmission, float index_of_refraction)
	: Material(diff), ks(spec), alpha(shininess), kt(transmission), ior(index_of_refraction) {
	float kd_mag = diff.norm();
	float ks_mag = spec.norm();
	s = kd_mag + ks_mag + transmission.norm();
	pd = kd_mag / s;
	pr = ks_mag / s;
	pt = 1 - (pd + pr);
}

Intersection BRDF::SampleLight(Sphere& sphere) const {
	assert(0);
	return Intersection();
}

Vector3f BRDF::EvalRadiance(const Intersection& intersection) const {
	assert(0);
	return Vector3f();
}

float BRDF::PdfLight(const Intersection& intersection) const {
	assert(0);
	return 0.0f;
}

//reflective material
Vector3f BRDF::SampleBrdf(const Vector3f& wo, const Vector3f& normal) const {
	float xi = random_float();
	float xi1 = random_float();
	float xi2 = random_float();

	//diffuse
	if (xi < pd) {
		return SampleLobe(normal, std::sqrt(xi1), 2 * PI * xi2);
	}

	float cos_theta = 0;
#ifdef GGX
	float ag = std::sqrt(2 / (alpha + 2));
	cos_theta = std::cos(std::atan(ag * std::sqrt(xi1) / std::sqrt(1 - xi1)));
#endif
#ifdef PHONG
	cos_theta = std::pow(xi1, 1 / (alpha + 1));
#endif
#ifdef BECKMAN
	float ab = std::sqrt(2 / (alpha + 2));
	cos_theta = std::cos(std::atan(std::sqrt(-ab * ab * std::log10(1 - xi1))));
#endif
	Vector3f m = SampleLobe(normal, cos_theta, 2 * PI * xi2);

	//reflection
	if(xi < pd + pr){
		return 2 * (std::abs(wo.dot(m)) * m) - wo;
	}
	//transmission
	else {
		float e = 0, eo = 0, ei = 0;
		SetIOR(wo, normal, ior, e, eo, ei);

		float wo_dot_m = wo.dot(m);
		float r = 1 - e * e * (1 - wo_dot_m* wo_dot_m);
		if (r < 0) {
			//total internal reflection
			return 2 * (std::abs(wo_dot_m) * m) - wo;
		}
		else {
			float sign = (wo.dot(normal) >= 0.f) ? 1.f : -1.f;
			return (e * wo_dot_m - sign * std::sqrt(r)) * m - e * wo;
		}
	}
}

float BRDF::PdfBrdf(const Vector3f& wo, const Vector3f& normal, const Vector3f& wi) const {
	//diffuse
	float Pd = std::abs(wi.dot(normal)) / PI;
	Vector3f m = (wo + wi).normalized();
	//reflection
	float Pr = D(m, normal) * std::abs(m.dot(normal)) / (4 * std::abs(wi.dot(m)));
	//transmission
	float e = 0, eo = 0, ei = 0;
	SetIOR(wo, normal, ior, e, eo, ei);
	m = -(wi * eo + wo * ei).normalized();
	float wo_dot_m = wo.dot(m);
	float r = 1.f - e * e * (1.f - wo_dot_m * wo_dot_m);
	float Pt = 0;
	if (r < 0) {
		//total internal reflection
		Pt = Pr;
	}
	else {
		Pt = D(m, normal) * std::abs(m.dot(normal)) * eo * eo *
			std::abs(wi.dot(m)) / ((eo*wi.dot(m) + ei*wo.dot(m)) * (eo * wi.dot(m) + ei * wo.dot(m)));
	}

	return pd * Pd + pr * Pr + pt * Pt;
}

Vector3f BRDF::EvalScattering(const Vector3f& wo, const Vector3f& normal, const Vector3f& wi, float t) const {
	//diffuse
	Vector3f Ed = kd / PI;
	Vector3f m = (wo + wi).normalized();
	//reflection
	Vector3f Er = D(m, normal) * G(wi, wo, m, normal) * F(wi.dot(m)) / 
		(4 * std::abs(wi.dot(normal)) * std::abs(wo.dot(normal)));

	//transmission
	float e = 0, eo = 0, ei = 0;
	SetIOR(wo, normal, ior, e, eo, ei);
	m = -(wi * eo + wo * ei).normalized();
	float wo_dot_m = wo.dot(m);
	float r = 1.f - e * e * (1.f - wo_dot_m * wo_dot_m);
	Vector3f Et(0.f, 0, 0);

	Vector3f att;
	if (wo.dot(normal) < 0) {
		att.x() = std::pow(2.71828f, t * std::log(kt.x()));
		att.y() = std::pow(2.71828f, t * std::log(kt.y()));
		att.z() = std::pow(2.71828f, t * std::log(kt.z()));
	}
	else {
		att = Vector3f(1, 1, 1);
	}

	if (r < 0) {
		//total internal reflection
		Et.x() = att.x() * Er.x();
		Et.y() = att.y() * Er.y();
		Et.z() = att.z() * Er.z();
		//Et = Er;
		
	}
	else {
		Et = D(m, normal) * G(wi, wo, m, normal) * (Vector3f(1.f, 1.f, 1.f) - F(wi.dot(m))) /
			(std::abs(wi.dot(normal)) * std::abs(wo.dot(normal)));

		Et *= std::abs(wi.dot(m)) * std::abs(wo.dot(m)) * eo * eo / ((eo * wi.dot(m) + ei * wo.dot(m))*(eo * wi.dot(m) + ei * wo.dot(m)));

		Et.x() *= att.x();
		Et.y() *= att.y();
		Et.z() *= att.z();
	}

	return std::abs(normal.dot(wi)) * (Ed + Er + Et);
}

float BRDF::D(const Vector3f& m, const Vector3f& normal) const
{
	float m_dot_normal = m.dot(normal);
	
#ifdef PHONG
	return Chi(m_dot_normal) * (alpha + 2) / (2 * PI) * std::pow(m_dot_normal, alpha);;
#endif
	float m_dot_normal_power4 = static_cast<float>(std::pow(m_dot_normal, 4));
	float tan_theta_m = std::sqrt(1 - m_dot_normal * m_dot_normal) / m_dot_normal;
	float probability = 0;
#ifdef BECKMAN
	float ab = std::sqrt(2 / (alpha + 2));
	probability = Chi(m_dot_normal) / (PI * ab * ab * m_dot_normal_power4) *
		std::pow(2.71828f, (-tan_theta_m * tan_theta_m / (ab * ab)));
#endif
#ifdef GGX
	float ag = std::sqrt(2 / (alpha + 2));
	float ag_tan_theta_m = ag * ag + tan_theta_m * tan_theta_m;
	probability = Chi(m_dot_normal) * ag * ag /
		(PI * m_dot_normal_power4 * ag_tan_theta_m * ag_tan_theta_m);
#endif	

	return probability;
}

float BRDF::G(const Vector3f& wi, const Vector3f& wo, const Vector3f& m, const Vector3f& normal) const
{
	return G1(wi, m, normal) * G1(wo, m, normal);
}

float BRDF::G1(const Vector3f& v, const Vector3f& m, const Vector3f& normal) const
{
	float v_dot_normal = v.dot(normal);
	if (v_dot_normal > 1)
		return 1.f;

	float tan_theta_v = std::sqrt(1 - (v_dot_normal * v_dot_normal)) / v_dot_normal;	
	if (tan_theta_v == 0)
		return 1.f;

#ifdef GGX
	float ag = std::sqrt(2 / (alpha + 2));
	return Chi(v.dot(m) / v_dot_normal) * 2 / (1 + std::sqrt(1 + ag * ag * tan_theta_v * tan_theta_v));
#endif

	float probability = 0;

#ifdef PHONG
	if (alpha < 1.6f) {
		float a = std::sqrt(alpha / 2 + 1) / tan_theta_v;
		probability = Chi(v.dot(m) / v_dot_normal) * (3.535f * a+ 2.181f * a* a) / (1 + 2.276f * a+ 2.577f * a* a);
	}
	else {
		probability = Chi(v.dot(m) / v_dot_normal);
	}
#endif

#ifdef BECKMAN
	float ab = std::sqrt(2 / (alpha + 2));
	float a = 1 / (ab * tan_theta_v);
	if (a < 1.6f) {
		probability = Chi(v.dot(m) / v_dot_normal) * (3.535f * a+ 2.181f * a* a) / (1 + 2.276f * a+ 2.577f * a* a);
	}
	else {
		probability = Chi(v.dot(m) / v_dot_normal);
	}
#endif
	return probability;
}

Vector3f BRDF::F(float d) const
{
	d = std::abs(d);
	float scalar = static_cast<float>(std::pow(1 - d, 5));
	float r = ks.x() + (1 - ks.x()) * scalar;
	float g = ks.y() + (1 - ks.y()) * scalar;
	float b = ks.z() + (1 - ks.z()) * scalar;
	return Vector3f(r, g, b);
}

Intersection Light::SampleLight(Sphere& sphere) const {
	Intersection intersection = SampleSphere(sphere.GetCenter(), sphere.GetRadius());
	intersection.shape_hit = &sphere;
	return intersection;
}

Vector3f Light::EvalRadiance(const Intersection& intersection) const {
	return kd;
}

float Light::PdfLight(const Intersection& intersection) const{
	float radius = static_cast<Sphere*>(intersection.shape_hit)->GetRadius();
	return 1 / (4 * PI * radius * radius); //Surface area of the sphere
}

Vector3f Light::SampleBrdf(const Vector3f& wo, const Vector3f& normal) const {
	assert(0);
	return Vector3f();
}

float Light::PdfBrdf(const Vector3f& wo, const Vector3f& normal, const Vector3f& wi) const {
	assert(0);
	return 0;
}

Vector3f Light::EvalScattering(const Vector3f& wo, const Vector3f& normal, const Vector3f& wi, float t) const {
	assert(0);
	return Vector3f();
}

IBL::IBL(const std::string& filename)
{
	FILE* fp = fopen(filename.c_str(), "rb");
	char errbuf[100] = { 0 };
	
	int r = RGBE_ReadHeader(fp, &width, &height, nullptr, errbuf);
	if (r != RGBE_RETURN_SUCCESS) {
		printf("error: %s\n", errbuf);
		assert(0);
	}

	image = new float[width * height * 3];

	r = RGBE_ReadPixels_RLE(fp, image, width, height, errbuf);
	if (r != RGBE_RETURN_SUCCESS) {
		printf("error: %s\n", errbuf);
		assert(0);
	}

	pBuffer = new float[width * (height + 1)];
	pUDist = &pBuffer[width * height];
	float* pSinTheta = new float[height];

	float angleFrac = PI / static_cast<float>(height);
	float theta = angleFrac * 0.5f;
	for (int i = 0; i < height; ++i, theta+=angleFrac) {
		pSinTheta[i] = sin(theta);
	}

	for ( int i = 0, m = i; i < width; i++, m += height) {
		float* pVDist = &pBuffer[m];
		unsigned int k = i * 3;
		pVDist[0] = 0.2126f * image[k + 0] + 0.7152f * image[k + 1] + 0.0722f * image[k + 2];
		pVDist[0] *= pSinTheta[0];

		for ( int j = 1, k = (width + i) * 3; j < height; j++, k += width * 3) {
			float lum = 0.2126f * image[k + 0] + 0.7152f * image[k + 1] + 0.0722f * image[k + 2];
			pVDist[j] = pVDist[j - 1] + lum * pSinTheta[j];
		}

		if (i == 0)
			pUDist[i] = pVDist[height - 1];
		else
			pUDist[i] = pUDist[i - 1] + pVDist[height - 1];
	}

	delete[] pSinTheta;
}

IBL::~IBL()
{
	delete[] pBuffer;
	delete[] image;
}

Intersection IBL::SampleLight(Sphere& sphere) const
{
	Intersection B;

	float u = random_float();
	float v = random_float();
	float maxUVal = pUDist[width - 1];
	float* pUPos = std::lower_bound(pUDist, pUDist + width, u * maxUVal);

	int iu = pUPos - pUDist;
	float* pVDist = &pBuffer[height * iu];
	float* pVPos = std::lower_bound(pVDist, pVDist + height, v * pVDist[height - 1]);

	int iv = pVPos - pVDist;

	float phi = 2 * PI - 2 * PI * iu / width;
	float theta = PI * iv / height;
	B.normal = Vector3f(std::sin(theta)*std::cos(phi), std::sin(theta)*std::sin(phi), std::cos(theta));
	B.pos = B.normal * sphere.GetRadius();
	B.shape_hit = &sphere;
	return B;
}

Vector3f IBL::EvalRadiance(const Intersection& intersection) const
{
	Vector3f P = intersection.pos.normalized();
	float u = (2 * PI - std::atan2(P[1], P[0])) / (2 * PI);
	u = u - std::floor(u);
	float v = std::acos(P[2]) / PI;
	int i0 = static_cast<int>(std::floor(u * width));
	int j0 = static_cast<int>(std::floor(v * height));
	float uw[2], vw[2];
	uw[1] = u * width - i0;
	uw[0] = 1.f - uw[1];
	vw[1] = v * height - j0;
	vw[0] = 1.f - vw[1];
	Vector3f r(0, 0, 0);
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			int k = 3 * (((j0 + j) % height) * width + ((i0 + i) % width));
			for (int c = 0; c < 3; c++)
				r[c] += uw[i] * vw[j] * image[k + c];
		}
	}

	return r;
}

float IBL::PdfLight(const Intersection& intersection) const
{
	Vector3f P = intersection.pos.normalized();
	float fu = (2 * PI - std::atan2(P[1], P[0])) / (2*PI);
	fu = fu - std::floor(fu);
	int u = static_cast<int>(std::floor(width*fu));
	int v = static_cast<int>(std::floor(height*std::acos(P[2]) / PI));
	float angleFrac = PI / static_cast<float>(height);
	float* pVDist = &pBuffer[height * u];
	float pdfU = (u == 0) ? pUDist[0] : pUDist[u] - pUDist[u - 1];
	pdfU /= pUDist[width - 1];
	pdfU *= width / (2 * PI);

	float pdfV = (v == 0) ? pVDist[0] : pVDist[v] - pVDist[v - 1];
	pdfV /= pVDist[height - 1];
	pdfV *= height / (PI);

	float theta = angleFrac * 0.5f + angleFrac * v;
	float radius = static_cast<Sphere*>(intersection.shape_hit)->GetRadius();
	float pdf = pdfU * pdfV * std::sin(theta) / (4.f * PI * radius * radius);

	return pdf;
}

Vector3f IBL::SampleBrdf(const Vector3f& wo, const Vector3f& normal) const
{
	assert(0);
	return Vector3f();
}

float IBL::PdfBrdf(const Vector3f& wo, const Vector3f& normal, const Vector3f& wi) const
{
	assert(0);
	return 0.0f;
}

Vector3f IBL::EvalScattering(const Vector3f& wo, const Vector3f& normal, const Vector3f& wi, float t) const
{
	assert(0);
	return Vector3f();
}
