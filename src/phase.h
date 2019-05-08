#ifndef CGL_STATICSCENE_PHASE_H
#define CGL_STATICSCENE_PHASE_H

#include "CGL/CGL.h"
#include "CGL/spectrum.h"
#include "CGL/vector3D.h"
#include "CGL/matrix3x3.h"

#include "sampler.h"
#include "image.h"

#include <algorithm>

namespace CGL {

/**
 * Interface for Phase functions.
 */
class Phase {
 public:

  /**
   * Evaluate Phase.
   * Given incident light direction wi and outgoing light direction wo. Note
   * that both wi and wo are defined in the local coordinate system at the
   * point of intersection.
   * \param wo outgoing light direction in local space of point of intersection
   * \param wi incident light direction in local space of point of intersection
   * \return reflectance in the given incident/outgoing directions
   */
  virtual Spectrum f (const Vector3D& wo, const Vector3D& wi) = 0;

  /**
   * Evaluate Phase.
   * Given the outgoing light direction wo, compute the incident light
   * direction and store it in wi. Store the pdf of the outgoing light in pdf.
   * Again, note that wo and wi should both be defined in the local coordinate
   * system at the point of intersection.
   * \param wo outgoing light direction in local space of point of intersection
   * \param wi address to store incident light direction
   * \param pdf address to store the pdf of the output incident direction
   * \return reflectance in the output incident and given outgoing directions
   */
  virtual Spectrum sample_f (const Vector3D& wo, Vector3D* wi, float* pdf) = 0;

  /**
   * Get the emission value of the particle material. For non-emitting particle
   * this would be a zero energy spectrum.
   * \return emission spectrum of the surface material
   */
  virtual Spectrum get_emission () const = 0;

  /**
   * If the Phase is a delta distribution. Materials that are perfectly specular,
   * (e.g. water, glass, mirror) only scatter light from a single incident angle
   * to a single outgoing angle. These BSDFs are best described with alpha
   * distributions that are zero except for the single direction where light is
   * scattered.
   */
  virtual bool is_delta() const = 0;

  /**
   * Reflection helper
   */
  virtual void reflect(const Vector3D& wo, Vector3D* wi);

  /**
   * Refraction helper
   */
  virtual bool refract(const Vector3D& wo, Vector3D* wi, float ior);

  const HDRImageBuffer* reflectanceMap;
  const HDRImageBuffer* normalMap;

}; // class Phase

/**
 * IsotropicPhase()
 */
class IsotropicPhase : public Phase {
 public:

  IsotropicPhase() { }

  Spectrum f(const Vector3D& wo, const Vector3D& wi);
  Spectrum sample_f(const Vector3D& wo, Vector3D* wi, float* pdf);
  Spectrum get_emission() const { return Spectrum(); }
  bool is_delta() const { return false; }

private:
  UniformSphereSampler3D sampler;

}; // class IsotropicPhase

/**
 * The Henyey-Greenstein Phase Function()
 */
class HenyeyGreensteinPhase : public Phase {
 public:

  HenyeyGreensteinPhase() : g(0.2) { }
  HenyeyGreensteinPhase(const double &g) : g(g) { }

  Spectrum f(const Vector3D& wo, const Vector3D& wi);
  Spectrum sample_f(const Vector3D& wo, Vector3D* wi, float* pdf);
  Spectrum get_emission() const { return Spectrum(); }
  bool is_delta() const { return false; }

private:
  // g is a parameter that determines the relative strength of 
  // forward and backward scattering, The asymmetryparameter g 
  // has a physical meaning — it represents the average cosine
  // of the scattered directions. Positive values of g give 
  // forward scattering, and negative values give backwards 
  // scattering. Avalueof g = 0 results in isotropic scattering
  double g;
  HenyeyGreensteinSampler3D sampler = HenyeyGreensteinSampler3D(g);

}; // class HenyeyGreensteinPhase


/**
 * The Schlick Phase Function()
 */
class SchlickPhase : public Phase {
 public:

  SchlickPhase() : k(Spectrum(0.2, 0.2, 0.4)) { }
  SchlickPhase(const double &k) : k(Spectrum(k, k, k)) { }

  Spectrum f(const Vector3D& wo, const Vector3D& wi);
  Spectrum sample_f(const Vector3D& wo, Vector3D* wi, float* pdf);
  Spectrum get_emission() const { return Spectrum(); }
  bool is_delta() const { return false; }

private:
  // k ∈ [−1,1] acts similarly to the g parameter and controls the 
  // preferential scattering direction. As with g, total backward scattering
  // is obtained with k=−1, k=1 gives total forward scattering, and k = 0 
  // corresponds to isotropic scattering. 
  Spectrum k;
  SchlickSampler3D sampler = SchlickSampler3D(k);

}; // class SchlickPhase


}

#endif  // CGL_STATICSCENE_BSDF_H