#include "phase.h"

#include <iostream>
#include <algorithm>
#include <utility>

using std::min;
using std::max;
using std::swap;

namespace CGL {

Spectrum IsotropicPhase::f(const Vector3D& wo, const Vector3D& wi) {
  // This function takes in both wo and wi and returns the evaluation of
  // the BSDF for those two directions.
  double pdf = 1. / (4. * PI);
  return Spectrum(pdf, pdf, pdf);
}

Spectrum IsotropicPhase::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // This function takes in only wo and provides pointers for wi and pdf,
  // which should be assigned by this function.
  // After sampling a value for wi, it returns the evaluation of the BSDF
  // at (wo, *wi).
  *wi = sampler.get_sample(pdf);
  return f(wo, *wi);
}

Spectrum HenyeyGreensteinPhase::f(const Vector3D& wo, const Vector3D& wi) {
  // This function takes in both wo and wi and returns the evaluation of
  // the BSDF for those two directions.
  double costheta = dot(wo, -wi) / (wo.norm() * wi.norm());
  double g2 = g * g;

  double numerator = 1. - g2;
  double denominator = 4. * PI * pow((1. + g2 - 2. * g * costheta), 1.5);
  double p_hg = numerator / denominator;
  return Spectrum(p_hg, p_hg, p_hg);
}

Spectrum HenyeyGreensteinPhase::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // This function takes in only wo and provides pointers for wi and pdf,
  // which should be assigned by this function.
  // After sampling a value for wi, it returns the evaluation of the BSDF
  // at (wo, *wi).
  *wi = sampler.get_sample(pdf);
  return f(wo, *wi);
}

Spectrum SchlickPhase::f(const Vector3D& wo, const Vector3D& wi) {
  // This function takes in both wo and wi and returns the evaluation of
  // the BSDF for those two directions.
  double costheta = dot(wo, -wi) / (wo.norm() * wi.norm());
  Spectrum identity = Spectrum(1., 1., 1.);

  Spectrum numerator = identity - k * k;
  Spectrum root = identity + costheta * k;
  Spectrum denominator = 4. * PI * root * root;
  return numerator / denominator;
}

Spectrum SchlickPhase::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // This function takes in only wo and provides pointers for wi and pdf,
  // which should be assigned by this function.
  // After sampling a value for wi, it returns the evaluation of the BSDF
  // at (wo, *wi).
  *wi = sampler.get_sample(pdf);
  return f(wo, *wi);
}

void Phase::reflect(const Vector3D& wo, Vector3D* wi) {

  // TODO: 1.1
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  (*wi).x = -wo.x;
  (*wi).y = -wo.y;
  (*wi).z = wo.z;

}

bool Phase::refract(const Vector3D& wo, Vector3D* wi, float ior) {

  // TODO: 1.3
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.

  // "entering"
  float eta = wo.z >= 0 ? 1. / ior : ior;
  float judge = 1. - eta * eta * (1. - pow(wo.z, 2));

  if (judge < 0)
    return false; // total internal reflection

  (*wi).x = - eta * wo.x;
  (*wi).y = - eta * wo.y;
  float sign_z = wo.z >= 0 ? -1. : 1.;
  (*wi).z = sign_z * sqrt(judge);
  return true;
}

} // namespace CGL
