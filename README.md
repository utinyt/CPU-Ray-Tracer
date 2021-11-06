# CPU-Ray-Tracer
This is a one-semester project generating high quality images using ray-tracing technique.<br>
Instructor: Dr. Gary Herron

## Project 1 - Ray casting
![ray_casting](https://github.com/utinyt/CPU-Ray-Tracer/blob/main/output_images/project1_ray_casting.png)<br>
#### Ray-casting
Implemented intersection calculations between ray vs simple primitives:
* Ray vs Sphere
* Ray vs Slab (Infinite volume bounded by two parallel planes)
* Ray vs AABB
* Ray vs Triangle
* Ray vs Cylinder
<br>
Used Eigen's implementation of sparial data structure to accelerate ray intersection calculation.
<br><br>

## Project 2 - Path tracing
![path_tracing](https://github.com/utinyt/CPU-Ray-Tracer/blob/main/output_images/project2_path_tracing.png)<br>
(800x600, 512 passes per pixel)
<br><br>
Instead of shooting a single ray, the program shoots ray multiple times and average the colors from each ray. Free anti-aliasing comes from slightly changing the direction of the ray so that it doesn't always head to the center of the pixel.<br>
<br>

## Project 3 - Reflection
![reflection](https://github.com/utinyt/CPU-Ray-Tracer/blob/main/output_images/project3_reflection.png)<br>
(600x480, 512 passes per pixel, GGX)
<br><br>
Used GGX BRDF for choosing the halfway vector ((wo + wi) / |wo + wi|), and calculated the reflection vector as a final output. Specular term is added to the BRDF.
<br><br>

## Project 4 - Transmission
![transmission](https://github.com/utinyt/CPU-Ray-Tracer/blob/main/output_images/project4_transmissionpng.png)<br>
(600x480, 512 passes per pixel)
<br>

#### Beer's law
As the ray goes through the object, the amount of light is attenuated, which is determined by the distance t the ray traveled through the object and pre-defined RGB value specifying the color after the ray transmits through the object.

#### Multiple importance sampling
Much smarter way of choosing a path (MIS) is used for this project.
<br><br>

## Project 5 - Image based lighting & Ray marching
![ibl1](https://github.com/utinyt/CPU-Ray-Tracer/blob/main/output_images/project5_ibl1.png)<br>
(2560x1080, 4096 passes per pixel)<br><br>
![ibl2](https://github.com/utinyt/CPU-Ray-Tracer/blob/main/output_images/project5_ibl2.png)<br>
(2560x1080, 1024 passes per pixel)<br><br>
![ibl3](https://github.com/utinyt/CPU-Ray-Tracer/blob/main/output_images/project5_ibl3.png)<br>
(2560x1080, 512 passes per pixel)<br>

#### Image based lighting
A HDR texture is mapped to the large sphere acting as a skydome. Every pixel is a source of light, and the part which has a maximum brightness casts shadow on the other side of the objects.

#### Ray marching
Raymarching finds the closest point in any direction and 'march' forward to the ray direction by the distance between current point and the closest point just found. The point keeps marching until the closest distance is really small.

#### Constructive solid geometry
Most simple geometry has its own signed distance function (SDF) returning the distance from a given point to itself. Objects can be combined by tweaking distances from those functions. The object on the left was constructed with a sphere, a box, and 3 cylinders: <br>
A = Intersect(sphere, box) <br>
B = Union(cylinderA, Union(cylinderB, cylinderC)) <br>
Final = Difference(A, B) <br>

#### HDR Image reference: http://www.hdrlabs.com/sibl/archive.html
