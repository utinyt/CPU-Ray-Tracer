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

## Project 2 - Path tracing
![path_tracing](https://github.com/utinyt/CPU-Ray-Tracer/blob/main/output_images/project2_path_tracing.png)<br>
(800x600, 512 passes per pixel)
<br><br>
Instead of shooting a single ray, the program shoots ray multiple times and average the colors from each ray. Free anti-aliasing comes from slightly changing the direction of the ray so that it doesn't always head to the center of the pixel.<br>

## Project 3 - Reflection
![reflection](https://github.com/utinyt/CPU-Ray-Tracer/blob/main/output_images/project3_reflection.png)<br>
(600x480, 512 passes per pixel)
<br><br>

## Project 4 - Transmission
![transmission](https://github.com/utinyt/CPU-Ray-Tracer/blob/main/output_images/project4_transmissionpng.png)<br>
(600x480, 512 passes per pixel)
<br><br>

## Project 5 - Image base lighting & Ray marching
![ibl1](https://github.com/utinyt/CPU-Ray-Tracer/blob/main/output_images/project5_ibl1.png)<br>
(2560x1080, 4096 passes per pixel)<br>
![ibl2](https://github.com/utinyt/CPU-Ray-Tracer/blob/main/output_images/project5_ibl2.png)<br>
(2560x1080, 1024 passes per pixel)<br>
![ibl3](https://github.com/utinyt/CPU-Ray-Tracer/blob/main/output_images/project5_ibl3.png)<br>
(2560x1080, 512 passes per pixel)<br>
<br>



#### Image reference: http://www.hdrlabs.com/sibl/archive.html
