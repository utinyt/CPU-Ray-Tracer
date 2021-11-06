#include <random>
#include "geom.h"
#include "utils.h"
#include "ray.h"
#include "shape.h"

// A good quality *thread-safe* Mersenne Twister random number generator.
std::random_device device;
std::mt19937_64 RNGen(device());
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1]

float random_float()
{
    return static_cast<float>(myrandom(RNGen));
}

float GeometryFactor(const Intersection& A, const Intersection& B)
{
	Vector3f D = A.pos - B.pos;
	float DD = D.dot(D);
	return std::abs(A.normal.dot(D) * B.normal.dot(D) / (DD*DD));
}

Vector3f SampleLobe(const Vector3f& normal, float c, float phi) {
	float s = std::sqrt(1 - (c * c));
	Vector3f K(s * std::cos(phi), s * std::sin(phi), c);
	Quaternionf q = Quaternionf::FromTwoVectors(Vector3f::UnitZ(), normal);
	return q._transformVector(K);
}

Intersection SampleSphere(const Vector3f& center, float radius)
{
	float xi1 = static_cast<float>(myrandom(RNGen));
	float xi2 = static_cast<float>(myrandom(RNGen));
	float z = 2 * xi1 - 1;
	float r = std::sqrt(1 - z * z);
	float a = 2 * PI * xi2;
	Vector3f normal(r * std::cos(a), r * std::sin(a), z);
	normal.normalize();
	return Intersection{nullptr, FINFINITY, center + radius * normal, normal};
}
