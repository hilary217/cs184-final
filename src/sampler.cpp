#include "sampler.h"
 
namespace CGL {

// Uniform Sampler2D Implementation //

Vector2D UniformGridSampler2D::get_sample() const {

  return Vector2D(random_uniform(), random_uniform());

}

// Uniform Hemisphere Sampler3D Implementation //

Vector3D UniformHemisphereSampler3D::get_sample() const {

  double Xi1 = random_uniform();
  double Xi2 = random_uniform();

  double theta = acos(Xi1);
  double phi = 2.0 * PI * Xi2;

  double xs = sinf(theta) * cosf(phi);
  double ys = sinf(theta) * sinf(phi);
  double zs = cosf(theta);

  return Vector3D(xs, ys, zs);

}

// Uniform Sphere Sampler3D Implementation //

Vector3D UniformSphereSampler3D::get_sample() const {
    double z = random_uniform() * 2 - 1;
    double sinTheta = sqrt(std::max(0.0, 1.0f - z * z));

    double phi = 2.0f * PI * random_uniform();

    return Vector3D(cos(phi) * sinTheta, sin(phi) * sinTheta, z);
}

Vector3D UniformSphereSampler3D::get_sample(float *pdf) const {
    double z = random_uniform() * 2 - 1;
    double sinTheta = sqrt(std::max(0.0, 1.0f - z * z));

    double phi = 2.0f * PI * random_uniform();
    *pdf = 1. / (4. * PI);
    return Vector3D(cos(phi) * sinTheta, sin(phi) * sinTheta, z);
}

Vector3D CosineWeightedHemisphereSampler3D::get_sample() const {
  float f;
  return get_sample(&f);
}

Vector3D CosineWeightedHemisphereSampler3D::get_sample(float *pdf) const {

  double Xi1 = random_uniform();
  double Xi2 = random_uniform();

  double r = sqrt(Xi1);
  double theta = 2. * PI * Xi2;
  *pdf = sqrt(1-Xi1) / PI;
  return Vector3D(r*cos(theta), r*sin(theta), sqrt(1-Xi1));
}

Vector3D HenyeyGreensteinSampler3D::get_sample() const {
  // TO CHANGE
  float f;
  return get_sample(&f);
}

Vector3D HenyeyGreensteinSampler3D::get_sample(float *pdf) const {
  // Sampling of z changed according to HenyeyGreenstein phase function
  // THIS SAMPLER IS WRONG
  double g2 = g * g;
  double u, z;

  do {
    u = random_uniform();
    z = (1. + g2) / (2. * g) - 
    ((1. - g2) * (1. - g2)) / (32. * PI * PI * g2 * g * u * u);
  } while(z > 1. or z < -1.);

  double sinTheta = sqrt(std::max(0.0, 1.0f - z * z));

  double phi = 2.0f * PI * random_uniform();
  *pdf = (1. - g2) / (4. * PI * pow((1. + g2 - 2. * g * z), 1.5));
  return Vector3D(cos(phi) * sinTheta, sin(phi) * sinTheta, z);
}

Vector3D SchlickSampler3D::get_sample() const {
  // TO CHANGE
  float f;
  return get_sample(&f);
}

Vector3D SchlickSampler3D::get_sample(float *pdf) const {
  // Sampling of z changed according to HenyeyGreenstein phase function
  double u, z;
  // std::srand(int(time(0)));
  int rand_num = std::rand() % 3;
  // double k1 = rand_num == 0 ? k.r : (rand_num == 1 ? k.g : k.b); 
  double k1 = k.b; 
  double k2 = k1 * k1;

  do {
    u = random_uniform();
    z = ((k2 - 1) / (2. * k1 * u - k1 + 1) + 1.) / k1;
    // printf("%f", z);
  } while(z > 1. or z < -1.);

  double sinTheta = sqrt(std::max(0.0, 1.0f - z * z));

  double phi = 2.0f * PI * random_uniform();
  *pdf = (1. - k2) / (2. * pow((1. - k1 * z), 2.));
  return Vector3D(cos(phi) * sinTheta, sin(phi) * sinTheta, z);
}

double DistanceSampler1D::get_sample() const {
  float f;
  return get_sample(&f);
}

double DistanceSampler1D::get_sample(float *pdf) const {
  double extinction = pos2extinction(origin);
  double u = random_uniform();
  double distance;
  distance = - log(1. - u) / extinction;
  *pdf = extinction * exp(- extinction * distance);
  return distance;
}

} // namespace CGL
