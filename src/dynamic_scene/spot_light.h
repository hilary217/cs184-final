#ifndef CGL_DYNAMICSCENE_SPOTLIGHT_H
#define CGL_DYNAMICSCENE_SPOTLIGHT_H

#include "scene.h"
#include "../static_scene/light.h"

namespace CGL { namespace DynamicScene {

class SpotLight : public SceneLight {
 public:

  SpotLight(const Collada::LightInfo& light_info, 
           const Matrix4x4& transform) {

    this->spectrum = light_info.spectrum;
    this->position = (transform * Vector4D(light_info.position, 1)).to3D();
    this->direction = (transform * Vector4D(light_info.direction, 1)).to3D() - position;
    this->direction.normalize();

    this->falloff_exp = light_info.falloff_exp;
    this->angle = light_info.falloff_deg;
    this->constant_att = light_info.constant_att;
    this->linear_att = light_info.linear_att;
    this->quadratic_att = light_info.quadratic_att;
  }

  StaticScene::SceneLight *get_static_light() const {
    StaticScene::SpotLight* l = 
      new StaticScene::SpotLight(spectrum, position, direction, angle * PI / 180., 
      falloff_exp, constant_att, linear_att, quadratic_att);
    return l;
  }

 private:

  Spectrum spectrum;
  Vector3D direction;
  Vector3D position;
  double falloff_exp;
  float angle;
  double constant_att;
  double linear_att;
  double quadratic_att;

};

} // namespace DynamicScene
} // namespace CGL

#endif //CGL_DYNAMICSCENE_SPOTLIGHT_H
