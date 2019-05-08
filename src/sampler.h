#ifndef CGL_SAMPLER_H
#define CGL_SAMPLER_H

#include "CGL/vector2D.h"
#include "CGL/vector3D.h"
#include "CGL/misc.h"
#include "random_util.h"

namespace CGL {

/**
 * Interface for generating point samples within the unit square
 */
class Sampler1D {
 public:

  /**
   * Virtual destructor.
   */
  virtual ~Sampler1D() { }

  /**
   * Take a point sample of the unit square
   */
  virtual double get_sample() const = 0;

}; // class Sampler2D

class Sampler2D {
 public:

  /**
   * Virtual destructor.
   */
  virtual ~Sampler2D() { }

  /**
   * Take a point sample of the unit square
   */
  virtual Vector2D get_sample() const = 0;

}; // class Sampler2D

/**
 * Interface for generating 3D vector samples
 */
class Sampler3D {
 public:

  /**
   * Virtual destructor.
   */
  virtual ~Sampler3D() { }

  /**
   * Take a vector sample of the unit hemisphere
   */
  virtual Vector3D get_sample() const = 0;

}; // class Sampler3D


/**
 * A Sampler2D implementation with uniform distribution on unit square
 */
class UniformGridSampler2D : public Sampler2D {
 public:

  Vector2D get_sample() const;

}; // class UniformSampler2D

/**
 * A Sampler3D implementation with uniform distribution on unit hemisphere
 */
class UniformHemisphereSampler3D : public Sampler3D {
 public:

  Vector3D get_sample() const;

}; // class UniformHemisphereSampler3D

/**
 * A Sampler3D implementation with uniform distribution on unit sphere
 */
class UniformSphereSampler3D : public Sampler3D {
 public:

  Vector3D get_sample() const;
  Vector3D get_sample(float* pdf) const;
}; // class UniformHemisphereSampler3D

/**
 * A Sampler3D implementation with cosine-weighted distribution on unit
 * hemisphere.
 */
class CosineWeightedHemisphereSampler3D : public Sampler3D {
 public:

  Vector3D get_sample() const;
  // Also returns the pdf at the sample point for use in importance sampling.
  Vector3D get_sample(float* pdf) const;

}; // class UniformHemisphereSampler3D

/**
 * TODO (extra credit) :
 * Jittered sampler implementations
 */

class HenyeyGreensteinSampler3D : public Sampler3D {
 public:
  HenyeyGreensteinSampler3D() : g(0.0) {}
  HenyeyGreensteinSampler3D(double &g) : g(g) {}
  Vector3D get_sample() const;
  Vector3D get_sample(float* pdf) const;
  
 private:
  double g;
}; // class HenyeyGreensteinSampler3D

class SchlickSampler3D : public Sampler3D {
 public:
  SchlickSampler3D() : k(Spectrum(0., 0., 0.)) {}
  SchlickSampler3D(Spectrum &k) : k(k) {}
  Vector3D get_sample() const;
  Vector3D get_sample(float* pdf) const;
  
 private:
  Spectrum k;
}; // class SchlickSampler3D

class DistanceSampler1D : public Sampler1D {
 public:
  DistanceSampler1D() : extinction(0.5) {}
  DistanceSampler1D(double &extinction) : extinction(extinction) {}
  double get_sample() const;
  double get_sample(float* pdf) const;
  
 private:
  double extinction;
}; // class DistanceSampler1D
} // namespace CGL




#endif //CGL_SAMPLER_H
