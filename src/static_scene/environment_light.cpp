#include "environment_light.h"

#include <algorithm>
#include <iostream>
#include <fstream>

namespace CGL { namespace StaticScene {

EnvironmentLight::EnvironmentLight(const HDRImageBuffer* envMap)
    : envMap(envMap) {
    	init();
}

EnvironmentLight::~EnvironmentLight() {
    delete[] pdf_envmap;
    delete[] conds_y;
    delete[] marginal_y;
}


void EnvironmentLight::init() {
	uint32_t w = envMap->w, h = envMap->h;
  pdf_envmap = new double[w * h];
	conds_y = new double[w * h];
	marginal_y = new double[h];

	std::cout << "[PathTracer] Initializing environment light...";

  // 3.3 step 1
	// Store the environment map pdf to pdf_envmap

	double sum = 0;
	for (int j = 0; j < h; ++j) {
		for (int i = 0; i < w; ++i) {
            pdf_envmap[w * j + i] = envMap->data[w * j + i].illum() * sin(PI * (j+.5) / h);
            sum += pdf_envmap[w * j + i];
		}
  }

  // normalize
  for (int j = 0; j < h; ++j) {
		for (int i = 0; i < w; ++i) {
      pdf_envmap[w * j + i] /= sum;
    }
	}

  // TODO: 3.3 step 2
  // Store the marginal distribution for y to marginal_y. Make sure to normalize pdf_envmap.
  double current_marginal_y_accumulation = 0.;
  double real_marginal_y[h];
  for (int j = 0; j < h; ++j) {
		for (int i = 0; i < w; ++i) {
      marginal_y[j] += pdf_envmap[w * j + i];
    }
    real_marginal_y[j] = marginal_y[j];
    marginal_y[j] += current_marginal_y_accumulation;
    current_marginal_y_accumulation = marginal_y[j];
	}

  // TODO: 3.3 step 3
  // Store the conditional distribution for x given y to conds_y
  for (int j = 0; j < h; ++j) {
    double current_cond_y_accumulation = 0.;
		for (int i = 0; i < w; ++i) {
      conds_y[w * j + i] = 
        pdf_envmap[w * j + i] / real_marginal_y[j] + current_cond_y_accumulation;
      current_cond_y_accumulation = conds_y[w * j + i];
    }
	}

	if (true) {
    std::cout << "Saving out probability_debug image for debug." << std::endl;
    save_probability_debug();
  }

	std::cout << "done." << std::endl;
}

// Helper functions

void EnvironmentLight::save_probability_debug() {
	uint32_t w = envMap->w, h = envMap->h;
	uint8_t* img = new uint8_t[4*w*h];

	for (int j = 0; j < h; ++j) {
		for (int i = 0; i < w; ++i) {
			img[4 * (j * w + i) + 3] = 255;
			img[4 * (j * w + i) + 0] = 255 * marginal_y[j];
			img[4 * (j * w + i) + 1] = 255 * conds_y[j * w + i];
			img[4 * (j * w + i) + 2] = 0;
		}
	}

    lodepng::encode("probability_debug.png", img, w, h);
    delete[] img;
}

Vector2D EnvironmentLight::theta_phi_to_xy(const Vector2D &theta_phi) const {
    uint32_t w = envMap->w, h = envMap->h;
    double x = theta_phi.y / 2. / PI * w;
    double y = theta_phi.x / PI * h;
    return Vector2D(x, y);
}

Vector2D EnvironmentLight::xy_to_theta_phi(const Vector2D &xy) const {
    uint32_t w = envMap->w, h = envMap->h;
    double x = xy.x;
    double y = xy.y;
    double phi = x / w * 2.0 * PI;
    double theta = y / h * PI;
    return Vector2D(theta, phi);
}

Vector2D EnvironmentLight::dir_to_theta_phi(const Vector3D &dir) const {
    Vector3D unit_dir = dir.unit();
    double theta = acos(unit_dir.y);
    double phi = atan2(-unit_dir.z, unit_dir.x) + PI;
    return Vector2D(theta, phi);
}

Vector3D EnvironmentLight::theta_phi_to_dir(const Vector2D& theta_phi) const {
    double theta = theta_phi.x;
    double phi = theta_phi.y;

    double y = cos(theta);
    double x = cos(phi - PI) * sin(theta);
    double z = -sin(phi - PI) * sin(theta);

    return Vector3D(x, y, z);
}

// Credits to Luowen Qian from Spring 2018 for this more robust bilerp
Spectrum EnvironmentLight::bilerp(const Vector2D& xy) const {
    long right = lround(xy.x), left, v = lround(xy.y);
    double u1 = right - xy.x + .5, v1;
    if (right == 0 || right == envMap->w) {
        left = envMap->w - 1;
        right = 0;
    } else left = right - 1;
    if (v == 0) v1 = v = 1; else if (v == envMap->h) {
        v = envMap->h - 1;
        v1 = 0;
    } else v1 = v - xy.y + .5;
    auto bottom = envMap->w * v, top = bottom - envMap->w;
    auto u0 = 1 - u1;
    return (envMap->data[top + left] * u1 + envMap->data[top + right] * u0) * v1 +
        (envMap->data[bottom + left] * u1 + envMap->data[bottom + right] * u0) * (1 - v1);
}


Spectrum EnvironmentLight::sample_L(const Vector3D& p, Vector3D* wi,
                                    float* distToLight,
                                    float* pdf) const {
  // TODO: 3.2
	// First implement uniform sphere sampling for the environment light

  // *wi = sampler_uniform_sphere.get_sample();
  // *distToLight = INF_D;
  // *pdf = 1 / (4 * PI);
  // Ray new_ray = Ray(p, *wi);
  // return sample_dir(new_ray);


  // TODO: 3.3
	// Later implement full importance sampling
	
  uint32_t w = envMap->w, h = envMap->h;
  
  // generate random number between [0, 1]
  Vector2D uni_2d_sample = sampler_uniform2d.get_sample();
  double u = uni_2d_sample.x, v = uni_2d_sample.y;

  // use inversion method (provided in some slide) 
  //to get the x, y of sample point
  int x, y;
  
  // std::cout << std::endl << "u: " << v << std::endl;
  for (int j = 0; j < h; j++) {
    // std::cout << marginal_y[j] << " ";
    if (marginal_y[j] > u) {
      y = j;
      break;
    }
  }

  // std::cout << std::endl << "v: " << v << std::endl;
  int row_skip = y * w;
  for (int i = 0; i < w; i++) {
    // std::cout << conds_y[row_skip + i] << " ";
    if (conds_y[row_skip + i] > v) {
      x = i;
      break;
    }
  }
  
  // get the sampled direction in theta_phi representation
  Vector2D theta_phi = xy_to_theta_phi(Vector2D(x, y));
  double theta = theta_phi.x, phi = theta_phi.y;
  double sin_theta = sin(theta);
  
  // return the results
  *wi = theta_phi_to_dir(theta_phi);
  *distToLight = INF_D;
  *pdf = pdf_envmap[w * y + x] * w * h / (2 * PI * PI * sin_theta);
  return envMap -> data[w * y + x];
}

Spectrum EnvironmentLight::sample_dir(const Ray& r) const {
  // TODO: 3.1
	// Use the helper functions to convert r.d into (x,y)
	// then bilerp the return value
  
  Vector2D theta_phi = dir_to_theta_phi(r.d);
  Vector2D x_y = theta_phi_to_xy(theta_phi);
  x_y -= Vector2D(0.5, 0.5);

  // Bilinear interpolation
  int width = envMap -> w, height = envMap -> h;

  int x0 = floor(x_y.x);
  int x1 = x0 + 1;
  int y0 = floor(x_y.y);
  int y1 = y0 + 1;
  double t = x_y.x - x0, u = x_y.y - y0;

  // x0 x1 y0 y1 out of scope?
  x0 = x0 < 0 ? x0 + 1 : x0;
  x1 = x1 > width - 1 ? x1 - 1 : x1;
  y0 = y0 < 0 ? y0 + 1 : y0;
  y1 = y1 > height - 1 ? y1 - 1 : y1;

  // do interpolation
  Spectrum envMap_00 = envMap -> data[y0 * width + x0];
  Spectrum envMap_01 = envMap -> data[y1 * width + x0];
  Spectrum envMap_10 = envMap -> data[y0 * width + x1];
  Spectrum envMap_11 = envMap -> data[y1 * width + x1];

  Spectrum envMap_bot = (1 - t) * envMap_00 + t * envMap_10;
  Spectrum envMap_top = (1 - t) * envMap_01 + t * envMap_11;

  return (1 - u) * envMap_bot + u * envMap_top;

  // return Spectrum(0.5, 0.5, 0.5);
}

} // namespace StaticScene
} // namespace CGL
