#include "bsdf.h"

#include <iostream>
#include <algorithm>
#include <utility>

using std::min;
using std::max;
using std::swap;

namespace CGL {

void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {
  Vector3D z = Vector3D(n.x, n.y, n.z);
  Vector3D h = z;
  if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z)) h.x = 1.0;
  else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z)) h.y = 1.0;
  else h.z = 1.0;

  z.normalize();
  Vector3D y = cross(h, z);
  y.normalize();
  Vector3D x = cross(z, y);
  x.normalize();

  o2w[0] = x;
  o2w[1] = y;
  o2w[2] = z;
}

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO: 1.2
  // Using BSDF::reflect(), implement sample_f for a mirror surface
  reflect(wo, wi);
  *pdf = 1.; 
  return reflectance / abs_cos_theta(*wi);
}

// Microfacet BSDF //

double MicrofacetBSDF::G(const Vector3D& wo, const Vector3D& wi) {
    return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D& h) {
  // TODO: 2.2
  // Compute Beckmann normal distribution function (NDF) here.
  // You will need the roughness alpha.
  float cos_theta_h = cos_theta(h);
  float tan_theta_h = sqrt(1 - cos_theta_h * cos_theta_h) / cos_theta_h;
  return exp(- pow( tan_theta_h/alpha , 2 ) ) / (PI * alpha * alpha * pow(cos_theta_h, 4));

  // return std::pow(cos_theta(h), 100.0);
}

Spectrum MicrofacetBSDF::F(const Vector3D& wi) {
  // TODO: 2.3
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Spectrum.
  Spectrum eta2_k2 = eta * eta + k * k;
  Spectrum two_eta_cos_theta = 2 * eta * cos_theta(wi);
  Spectrum unit_spectrum = Spectrum(1., 1., 1.);
  Spectrum cos2theta = pow(cos_theta(wi), 2) * unit_spectrum;

  Spectrum Rs = 
    (eta2_k2 - two_eta_cos_theta + cos2theta) / 
    (eta2_k2 + two_eta_cos_theta + cos2theta);
  
  Spectrum Rp = 
    (eta2_k2 * cos2theta - two_eta_cos_theta + unit_spectrum) / 
    (eta2_k2 * cos2theta + two_eta_cos_theta + unit_spectrum);
  
  return (Rs + Rp) / 2;
  // return Spectrum();
}

Spectrum MicrofacetBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  // TODO: 2.1
  // Implement microfacet model here
  float n_dot_wo = wo.z, n_dot_wi = wi.z;
  if (not (n_dot_wi > 0 and n_dot_wo > 0))
    return Spectrum();
  
  Vector3D h = wo + wi;
  h.normalize();
  return F(wi) * G(wo, wi) * D(h) / (4 * n_dot_wo * n_dot_wi);

  // return Spectrum();
}

Spectrum MicrofacetBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO: 2.4
  // *Importance* sample Beckmann normal distribution function (NDF) here.
  // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
  //       and return the sampled BRDF value.

  Vector2D sampled_pair = sampler.get_sample();
  float r1 = sampled_pair.x, r2 = sampled_pair.y;

  float tan_theta_h = sqrt(- alpha * alpha * log(1 - r1));
  float cos_theta_h = 1. / sqrt( 1 + tan_theta_h * tan_theta_h );
  float sin_theta_h = tan_theta_h * cos_theta_h;
  float phi_h = 2 * PI * r2;

  // Compute h from sampled theta and phi
  Vector3D h = Vector3D( 
    sin_theta_h * cos(phi_h), 
    sin_theta_h * sin(phi_h), 
    cos_theta_h);

  // Compute wi with h and wo
  *wi = 2 * dot(wo, h) * h - wo;

  // Check if wi is valid
  if (wi -> z <= 0){
    *pdf = 0.;
    return Spectrum();
  }
  
  // Compute pdf w.r.t. wi using pdf w.r.t theta and phi
  float p_theta = 
    2 * tan_theta_h * exp( - pow( tan_theta_h / alpha , 2 ) ) / 
    pow( alpha * cos_theta_h , 2 );
  float p_phi = 1 / (2 * PI);

  float p_w_h = p_theta * p_phi / sin_theta_h;
  float p_w_wi = p_w_h / (4 * dot(*wi, h));

  *pdf = p_w_wi;

  return f(wo, *wi);

  // *wi = cosineHemisphereSampler.get_sample(pdf); //placeholder
  // return MicrofacetBSDF::f(wo, *wi);
}

// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO: 1.4
  // Compute Fresnel coefficient and either reflect or refract based on it.
  if (not refract(wo, wi, ior)) {
    reflect(wo, wi);
    *pdf = 1.;
    return reflectance / abs_cos_theta(*wi);
  }

  // Compute Schlick's reflection coefficient R
  float R0 = pow( ((1 - ior) / (1 + ior)) , 2);
  float R = R0 + (1 - R0) * pow( (1 - abs_cos_theta(*wi)) , 5);

  if (coin_flip(R)) {
    reflect(wo, wi);
    *pdf = R;
    return R * reflectance / abs_cos_theta(*wi);
  }
  else {
    refract(wo, wi, ior);
    *pdf = 1. - R;
    float eta = wo.z >= 0 ? 1. / ior : ior;
    return (1. - R) * transmittance / abs_cos_theta(*wi) / pow(eta, 2);
  }
}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {

  // TODO: 1.1
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  (*wi).x = -wo.x;
  (*wi).y = -wo.y;
  (*wi).z = wo.z;

}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {

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

// Emission BSDF //

Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0 / PI;
  *wi  = sampler.get_sample(pdf);
  return Spectrum();
}

} // namespace CGL
