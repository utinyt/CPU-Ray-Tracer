#ifndef UTILS_H
#define UTILS_H

struct Intersection;

const float EPSILON = 0.000001f;
const float PI = 3.14159f;

float GeometryFactor(const Intersection& A, const Intersection& B);
Vector3f SampleLobe(const Vector3f& normal, float c, float phi);
Intersection SampleSphere(const Vector3f& center, float radius);

float random_float();

#endif