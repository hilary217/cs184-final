#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CGL { namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  Vector3D o_ = r.o; Vector3D d_ = r.d;
  Vector3D c_ = o;
  double a = dot(d_, d_);
  double b = 2 * dot((o_ - c_), d_);
  double c = dot((o_ - c_), (o_ - c_)) - r2;
  double delta = b * b - 4 * a * c;
  if (delta < 0) return false;
  double sqrt_delta = sqrt(delta);
  t1 = (- b - sqrt_delta) / (2 * a);
  t2 = (- b + sqrt_delta) / (2 * a);
  return true;

}

bool Sphere::intersect(const Ray& r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  double t1, t2;
  if (not test(r, t1, t2)) return false;
  if (t1 <= r.max_t and t1 >= r.min_t) {
    r.max_t = t1;
    return true;
  }
  if (t2 <= r.max_t and t2 >= r.min_t) {
    r.max_t = t2;
    return true;
  }
  return false;
}

bool Sphere::intersect(const Ray& r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  double t1, t2;
  if (not test(r, t1, t2)) return false;
  if (t1 <= r.max_t and t1 >= r.min_t) {
    r.max_t = t1;
    i -> t = t1;
    i -> primitive = this;
    i -> bsdf = get_bsdf();
    Vector3D hit_point = r.o + t1 * r.d;
    i -> n = normal(hit_point);
    return true;
  }
  if (t2 <= r.max_t and t2 >= r.min_t) {
    r.max_t = t2;
    i -> t = t2;
    i -> primitive = this;
    i -> bsdf = get_bsdf();
    Vector3D hit_point = r.o + t2 * r.d;
    i -> n = normal(hit_point);
    return true;
  }
  return false;
  
}

void Sphere::draw(const Color& c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c, float alpha) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CGL
