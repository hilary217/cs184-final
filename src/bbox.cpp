#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

  Vector3D o = r.o;
  Vector3D d = r.d;
  
  double x_min = min.x, y_min = min.y, z_min = min.z;
  double x_max = max.x, y_max = max.y, z_max = max.z;

  if (d.x == 0 and (o.x < x_min or o.x > x_max)) return false;
  if (d.y == 0 and (o.y < y_min or o.y > y_max)) return false;
  if (d.z == 0 and (o.z < z_min or o.z > z_max)) return false;
  
  double tx_min = (x_min - o.x) / d.x; 
  double tx_max = (x_max - o.x) / d.x;
  if (tx_min > tx_max) {
    double tmp;
    tmp = tx_min;
    tx_min = tx_max;
    tx_max = tmp;
  }
  double ty_min = (y_min - o.y) / d.y; 
  double ty_max = (y_max - o.y) / d.y; 
  if (ty_min > ty_max) {
    double tmp;
    tmp = ty_min;
    ty_min = ty_max;
    ty_max = tmp;
  }
  double tz_min = (z_min - o.z) / d.z;
  double tz_max = (z_max - o.z) / d.z;
  if (tz_min > tz_max) {
    double tmp;
    tmp = tz_min;
    tz_min = tz_max;
    tz_max = tmp;
  }

  double t_min = tx_min > ty_min ? 
  (tx_min > tz_min ? tx_min : tz_min) : 
  (ty_min > tz_min ? ty_min : tz_min);

  double t_max = tx_max < ty_max ? 
  (tx_max < tz_max ? tx_max : tz_max) : 
  (ty_max < tz_max ? ty_max : tz_max);
  
  if (t_min <= t_max) {
    if (not (t_min > t1 or t_max < t0))
      return true;
    // if (t_min >= t0 or t_max <= t1) {

    //   // t0 = t_min > t0 ? t_min : t0;
    //   // t1 = t_max < t1 ? t_max : t1;
    //   return true;
    // }
    // else {
    //   // printf("ray not hit\n\n");
    //   return false;
    // }
  }
  return false;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
