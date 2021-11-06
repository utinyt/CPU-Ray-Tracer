#ifndef MATERIAL_H
#define MATERIAL_H

struct Intersection;
class Sphere;
class Material {
public:
	Material() : kd(Vector3f(1.f, 1.f, 1.f)) {}
	Material(const Vector3f& d) : kd(d) {}
	Material(const Material& mat) { kd = mat.kd; }
	virtual ~Material() {}
	virtual bool IsLight() const = 0;

	//emissive material
	virtual Intersection SampleLight(Sphere& sphere) const = 0;
	virtual Vector3f EvalRadiance(const Intersection& intersection) const = 0;
	virtual float PdfLight(const Intersection& intersection) const = 0;

	//reflective material
	virtual Vector3f SampleBrdf(const Vector3f& wo, const Vector3f& normal) const = 0;
	virtual float PdfBrdf(const Vector3f& wo, const Vector3f& normal, const Vector3f& wi) const = 0;
	virtual Vector3f EvalScattering(const Vector3f& wo, const Vector3f& normal, const Vector3f& wi, float t) const = 0;

public:
	Vector3f kd;
};

class BRDF final : public Material {
public:
	BRDF(const Vector3f& diff, const Vector3f& spec, float shininess, const Vector3f& transmission = Vector3f(0, 0, 0), float index_of_refraction = 1.f);

	virtual bool IsLight() const override { return false; }

	//emissive material
	virtual Intersection SampleLight(Sphere& sphere) const override;
	virtual Vector3f EvalRadiance(const Intersection& intersection) const override;
	virtual float PdfLight(const Intersection& intersection) const override;

	//reflective material
	virtual Vector3f SampleBrdf(const Vector3f& wo, const Vector3f& normal) const override;
	virtual float PdfBrdf(const Vector3f& wo, const Vector3f& normal, const Vector3f& wi) const override;
	virtual Vector3f EvalScattering(const Vector3f& wo, const Vector3f& normal, const Vector3f& wi, float t) const override;

public:
	Vector3f ks;
	float alpha;
	Vector3f kt;
	float ior;

private:
	float D(const Vector3f& m, const Vector3f& normal) const;
	float G(const Vector3f& wi, const Vector3f& wo, const Vector3f& m, const Vector3f& normal) const;
	float G1(const Vector3f& v, const Vector3f& m, const Vector3f& normal) const;
	Vector3f F(float d) const;
	float Chi(float d) const { return d > 0 ? 1.f : 0.f; }

	float pd = 0;
	float pr = 0;
	float pt = 0;
	float s = 0;
};

class Light final : public Material {
public:
	Light(const Vector3f& color) : Material(color) {}
	virtual bool IsLight() const override { return true; }

	//emissive material
	virtual Intersection SampleLight(Sphere& sphere) const override;
	virtual Vector3f EvalRadiance(const Intersection& intersection) const override;
	virtual float PdfLight(const Intersection& intersection) const override;

	//reflective material
	virtual Vector3f SampleBrdf(const Vector3f& wo, const Vector3f& normal) const override;
	virtual float PdfBrdf(const Vector3f& wo, const Vector3f& normal, const Vector3f& wi) const override;
	virtual Vector3f EvalScattering(const Vector3f& wo, const Vector3f& normal, const Vector3f& wi, float t) const override;
};

class IBL final : public Material {
public:
	IBL(const std::string& filename);
	~IBL();

	virtual bool IsLight() const override { return true; }

	//emissive material
	virtual Intersection SampleLight(Sphere& sphere) const override;
	virtual Vector3f EvalRadiance(const Intersection& intersection) const override;
	virtual float PdfLight(const Intersection& intersection) const override;

	//reflective material
	virtual Vector3f SampleBrdf(const Vector3f& wo, const Vector3f& normal) const override;
	virtual float PdfBrdf(const Vector3f& wo, const Vector3f& normal, const Vector3f& wi) const override;
	virtual Vector3f EvalScattering(const Vector3f& wo, const Vector3f& normal, const Vector3f& wi, float t) const override;

private:
	float* pBuffer = nullptr;
	float* pUDist = nullptr;
	float* image = nullptr;
	int width = 0, height = 0, channel_num = 0;
};

#endif
