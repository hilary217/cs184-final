//
// TODO: Copy over 3-1 code after turning on BUILD_3-1 flag
//

#include "part1_code.h"
#include <time.h>

using namespace CGL::StaticScene;

using std::min;
using std::max;

namespace CGL {
  Spectrum PathTracer::estimate_reduced_radiance(const Spectrum &src_radiance, const Vector3D &src, const Vector3D &recv)
  {
    // Estimate the radiance to at the recv point
    // the radiance are absorbed/out-scattered by the medium
    // while travelling from light source to the recv point 

    // return src_radiance * exp(-0.25 * (src - recv).norm());

    double max_t = (src - recv).norm();
    Vector3D d = (recv - src).unit();

    double extinction, optical_depth = 0., total_dist = 0.;
    // printf("max_t: %f\n", max_t);
    
    while (true) {
      extinction = pos2extinction(src + d * total_dist);

      // have reached the nearest surface
      if (total_dist + space_step > max_t) {
        optical_depth += extinction * (max_t - total_dist);
        break;
      }
      else {
        optical_depth += extinction * space_step;
        total_dist += space_step;
      }
    }
    
    return src_radiance * exp(-optical_depth);
  }

  Spectrum PathTracer::estimate_direct_lighting_hemisphere(
    const Ray& r, const Intersection& isect, const Interaction& interact) {
    // Estimate the lighting from this intersection coming directly from a light.
    // For this function, sample uniformly in a hemisphere. 

    // reflect
    if (not interact.interacted) {
      // make a coordinate system for a hit point
      // with N aligned with the Z direction.
      Matrix3x3 o2w;
      make_coord_space(o2w, isect.n);
      Matrix3x3 w2o = o2w.T();

      // w_out points towards the source of the ray (e.g.,
      // toward the camera if this is a primary ray)
      const Vector3D& hit_p = r.o + r.d * isect.t;
      const Vector3D& w_out = w2o * (-r.d);

      // This is the same number of total samples as estimate_direct_lighting_importance (outside of delta lights). 
      // We keep the same number of samples for clarity of comparison.
      int num_samples = scene->lights.size() * ns_area_light;
      Spectrum L_out;

      // TODO (Part 3.2): 
      // Write your sampling loop here
      // COMMENT OUT `normal_shading` IN `est_radiance_global_illumination` BEFORE YOU BEGIN
      for (int j = 0; j != num_samples; j++) {
        Vector3D wi = hemisphereSampler -> get_sample();
        double pdf;
        
        Intersection i;
        Vector3D wi_world = o2w * wi;

        Vector3D biased_hit_p = hit_p + EPS_D * wi_world;

        Ray sample_ray = Ray(biased_hit_p, wi_world);
        if (bvh -> intersect(sample_ray, &i)) {
          Spectrum emission =  i.bsdf -> get_emission();
          if (emission != Spectrum()) {
            Vector3D light_pos = biased_hit_p + i.t * wi_world;
            Spectrum L_reduced = estimate_reduced_radiance(
              emission, biased_hit_p, light_pos);
            L_out += 
              (2 * PI / double(num_samples)) * L_reduced * 
              isect.bsdf -> f(w2o * -r.d, wi) * cos_theta(wi);
          }
        }
      }
      return L_out;
    }
    // scattering
    else {
      Matrix3x3 o2w;
      make_coord_space(o2w, interact.n);
      Matrix3x3 w2o = o2w.T();

      const Vector3D& hit_p = r.o + r.d * interact.t;
      const Vector3D& w_out = w2o * (-r.d);

      int num_samples = scene->lights.size() * ns_area_light;
      Spectrum L_out;

      for (int j = 0; j != num_samples; j++) {
        Vector3D wi = sphereSampler -> get_sample();
        double pdf;
        
        // Intersection i here is the intersection between wi and light source
        Intersection i;
        Vector3D wi_world = o2w * wi;

        Vector3D biased_hit_p = hit_p + EPS_D * wi_world;

        Ray sample_ray = Ray(biased_hit_p, wi_world);
        if (bvh -> intersect(sample_ray, &i)) {
          Spectrum emission =  i.bsdf -> get_emission();
          if (emission != Spectrum()) {
            Vector3D light_pos = biased_hit_p + i.t * wi_world;
            Spectrum L_reduced = estimate_reduced_radiance(
              emission, biased_hit_p, light_pos);
            L_out += 
              (4 * PI / double(num_samples)) * pos2scattering(hit_p) / pos2extinction(hit_p) *
              L_reduced * interact.phase -> f(w_out, wi);
          }
        }

      }
      return L_out;
    }
  }

  Spectrum PathTracer::estimate_direct_lighting_importance(
    const Ray& r, const Intersection& isect, const Interaction& interact) {
    // Estimate the lighting from this intersection coming directly from a light.
    // To implement importance sampling, sample only from lights, not uniformly in a hemisphere. 

    // reflect
    if (not interact.interacted) {
      // make a coordinate system for a hit point
      // with N aligned with the Z direction.
      Matrix3x3 o2w;
      make_coord_space(o2w, isect.n);
      Matrix3x3 w2o = o2w.T();

      // w_out points towards the source of the ray (e.g.,
      // toward the camera if this is a primary ray)
      const Vector3D& hit_p = r.o + r.d * isect.t;
      const Vector3D& w_out = w2o * (-r.d);
      Spectrum L_out;

      // TODO (Part 3.2): 
      // Here is where your code for looping over scene lights goes
      // COMMENT OUT `normal_shading` IN `est_radiance_global_illumination` BEFORE YOU BEGIN
      vector<SceneLight*> &lights = scene -> lights;
      
      for (auto it = lights.begin(); it != lights.end(); it ++) {
        SceneLight *light = *it;
        Vector3D wi;
        float dist, pdf;

        if (light -> is_delta_light()) {
          // one sample
          Spectrum radiance_in = light -> sample_L(hit_p, &wi, &dist, &pdf);
          if (radiance_in != Spectrum()) {
            // std::cout << "Sample_L reflect: " << radiance_in << std::endl;
            // std::cout << interact.t <<  std::endl << std::endl;
          }
          Vector3D w_in = w2o * wi;
          
          if (cos_theta(w_in) < 0) continue;

          Vector3D biased_hit_p = hit_p + EPS_D * wi;
          
          Ray out_ray = Ray(biased_hit_p, wi, double(dist));

          Intersection i;
          if (not bvh -> intersect(out_ray, &i)) {
            Vector3D light_pos = biased_hit_p + dist * wi;
            Spectrum L_reduced = estimate_reduced_radiance(
              radiance_in, light_pos, biased_hit_p);
            L_out += L_reduced * isect.bsdf -> f(w_out, w_in) * 
              cos_theta(w_in) / pdf;
            // std::cout << "bsdf delta: " << L_out << std::endl;
            // std::cout << "bsdf delta dist: " << dist << std::endl;
          }
        }

        else
        {
          // ns_area
          for (int j = 0; j != ns_area_light; j++) {
            Spectrum radiance_in = light -> sample_L(hit_p, &wi, &dist, &pdf);
            Vector3D w_in = w2o * wi;
            // cout << "area " << pdf << endl;
            // cout << "ns_area_light " << ns_area_light << endl;
            
            // if (cos_theta(w_in) < 0) continue;

            Vector3D biased_hit_p = hit_p + EPS_D * wi;
            
            Ray out_ray = Ray(biased_hit_p, wi, double(dist));

            Intersection i;
            if (not bvh -> intersect(out_ray, &i)) {
              Vector3D light_pos = biased_hit_p + dist * wi;
              Spectrum L_reduced = estimate_reduced_radiance(
                radiance_in, light_pos, biased_hit_p);
              L_out += (1. / ns_area_light) * 
                L_reduced * isect.bsdf -> f(w_out, w_in) * cos_theta(w_in) / pdf;
              // std::cout << "bsdf: " << L_out << std::endl;
            }
          }
        }
        
      }

      return L_out;
    }
    // scatter
    else {
      Matrix3x3 o2w;
      make_coord_space(o2w, interact.n);
      Matrix3x3 w2o = o2w.T();

      const Vector3D& hit_p = r.o + r.d * interact.t;
      const Vector3D& w_out = w2o * (-r.d);
      Spectrum L_out;

      vector<SceneLight*> &lights = scene -> lights;
      
      for (auto it = lights.begin(); it != lights.end(); it ++) {
        SceneLight *light = *it;
        Vector3D wi;
        float dist, pdf;

        if (light -> is_delta_light()) {
          Spectrum radiance_in = light -> sample_L(hit_p, &wi, &dist, &pdf);
          if (radiance_in != Spectrum()) {
            // std::cout << "Sample_L: " << radiance_in << std::endl;
            // std::cout << interact.t <<  std::endl << std::endl;
          }
          Vector3D w_in = w2o * wi;
          
          if (cos_theta(w_in) < 0) continue;

          Vector3D biased_hit_p = hit_p + EPS_D * wi;
          Ray out_ray = Ray(biased_hit_p, wi, double(dist));

          Intersection i;
          if (not bvh -> intersect(out_ray, &i)) {
            Vector3D light_pos = biased_hit_p + dist * wi;
            Spectrum L_reduced = estimate_reduced_radiance(
              radiance_in, light_pos, biased_hit_p);
            // std::cout << "Sample_L: " << radiance_in << std::endl;
            // std::cout << "L_reduced: " << L_reduced << std::endl;
            L_out += pos2scattering(hit_p) / pos2extinction(hit_p) *
              L_reduced * interact.phase -> f(w_out, w_in) / pdf;
            // std::cout << "phase delta: " << L_out << std::endl;
            // std::cout << "phase delta dist: " << dist << std::endl;
          }
        }

        else
        {
          // ns_area
          for (int j = 0; j != ns_area_light; j++) {
            Spectrum radiance_in = light -> sample_L(hit_p, &wi, &dist, &pdf);
            Vector3D w_in = w2o * wi;
            // cout << "area " << pdf << endl;
            // cout << "ns_area_light " << ns_area_light << endl;
            
            if (cos_theta(w_in) < 0) continue;

            Vector3D biased_hit_p = hit_p + EPS_D * wi;
            Ray out_ray = Ray(biased_hit_p, wi, double(dist));

            Intersection i;
            if (not bvh -> intersect(out_ray, &i)) {
              Vector3D light_pos = biased_hit_p + dist * wi;
              Spectrum L_reduced = estimate_reduced_radiance(
                radiance_in, light_pos, biased_hit_p);
              L_out += (1. / ns_area_light) * (pos2scattering(hit_p) / pos2extinction(hit_p)) * 
                L_reduced * interact.phase -> f(w_out, w_in) / pdf;
              // std::cout << "phase: " << L_out << std::endl;
            }
          }
        }
        
      }
      return L_out;
    }

  }

  Spectrum PathTracer::zero_bounce_radiance(
    const Ray&r, const Intersection& isect, const Interaction& interact) {
    // Returns the light that results from no bounces of light
      
      if (interact.interacted) {
        return Spectrum();
      }
      Vector3D p = r.o + isect.t * r.d;
      return estimate_reduced_radiance(
        isect.bsdf -> get_emission(), p, r.o);
  }

  Spectrum PathTracer::one_bounce_radiance(
    const Ray&r, const Intersection& isect, const Interaction& interact) {
    // Returns either the direct illumination by hemisphere or importance sampling
    // depending on `direct_hemisphere_sample`
    // (you implemented these functions in Part 3)

    // return Spectrum();
    if (direct_hemisphere_sample)
      return estimate_direct_lighting_hemisphere(r, isect, interact);
    else
      return estimate_direct_lighting_importance(r, isect, interact);
  }

  Spectrum PathTracer::at_least_one_bounce_radiance(
    const Ray&r, const Intersection& isect, const Interaction& interact) {

    // reflect
    if (not interact.interacted) {
      Matrix3x3 o2w;
      make_coord_space(o2w, isect.n);
      Matrix3x3 w2o = o2w.T();

      Vector3D hit_p = r.o + r.d * isect.t;
      Vector3D w_out = w2o * (-r.d);

      Spectrum L_out = Spectrum();
      if (!isect.bsdf -> is_delta()) {
        L_out += one_bounce_radiance(r, isect, interact);
      }

      // TODO (Part 4.2): 
      // Here is where your code for sampling the BSDF,
      // performing Russian roulette step, and returning a recursively 
      // traced ray (when applicable) goes
      Vector3D w_in;
      float pdf_dir;
      Spectrum sampled_bsdf = isect.bsdf ->sample_f(w_out, &w_in, &pdf_dir);
      
      float cpdf = 0.6;
      // if (false) {
      // if ((r.depth > 1 and coin_flip(cpdf)) or (r.depth == max_ray_depth)) {
      if ((r.depth > 1 and coin_flip(cpdf))) {
        Vector3D wi = o2w * w_in;
        Ray new_ray = Ray(hit_p + EPS_D * wi, wi, INF_D, r.depth - 1);
        Intersection i;
        if (bvh -> intersect(new_ray, &i)) {
          for (size_t j = 0; j < 2; j++) {
            
            Interaction ita;
            float pdf_dist;
            DistanceSampler1D* distanceSampler = new DistanceSampler1D(&pos2extinction, space_step);
            distanceSampler -> set_ray(&new_ray);
            distanceSampler -> set_max_t(i.t);
            double sampled_dist = distanceSampler -> get_sample(&pdf_dist);
            delete distanceSampler;

            if (sampled_dist >= i.t) {
              ita.interacted = false;
            }
            else {
              Vector3D next_ita_point = new_ray.o + new_ray.d * sampled_dist;
              SchlickPhase *phase_pos = new SchlickPhase(pos2phase(next_ita_point));

              ita.interacted = true;
              ita.t = sampled_dist;
              new_ray.max_t = sampled_dist;
              ita.n = -new_ray.d;
              ita.phase = phase_pos;
            } 
            Spectrum radiance_in = at_least_one_bounce_radiance(new_ray, i, ita);
            if (isect.bsdf -> is_delta())
              radiance_in += zero_bounce_radiance(new_ray, i, ita);
            if (pdf_dir != 0)
              L_out += (1. / 2.) * radiance_in * sampled_bsdf * abs_cos_theta(w_in) / pdf_dir / cpdf;
          }
        }
      }
      
      return L_out;
    }
    // scatter
    else {
      Matrix3x3 o2w;
      make_coord_space(o2w, interact.n);
      Matrix3x3 w2o = o2w.T();

      Vector3D hit_p = r.o + r.d * interact.t;
      Vector3D w_out = w2o * (-r.d);

      Spectrum L_out = Spectrum();
      L_out += one_bounce_radiance(r, isect, interact);

      // TODO (Part 4.2): 
      // Here is where your code for sampling the BSDF,
      // performing Russian roulette step, and returning a recursively 
      // traced ray (when applicable) goes
      Vector3D w_in;
      float pdf_dir;
      Spectrum sampled_phase_f = interact.phase ->sample_f(w_out, &w_in, &pdf_dir);
      delete interact.phase;
      
      float cpdf = 0.6;
      // if (false) {
      // if ((r.depth > 1 and coin_flip(cpdf)) or (r.depth == max_ray_depth)) {

      if ((r.depth > 1 and coin_flip(cpdf))) {
        Vector3D wi = o2w * w_in;
        Ray new_ray = Ray(hit_p + EPS_D * wi, wi, INF_D, r.depth - 1);
        Intersection i;
        if (bvh -> intersect(new_ray, &i)) {
          for (size_t j = 0; j < 2; j++) {

            Interaction ita;
            float pdf_dist;
            DistanceSampler1D* distanceSampler = new DistanceSampler1D(&pos2extinction, space_step);
            distanceSampler -> set_ray(&new_ray);
            distanceSampler -> set_max_t(i.t);
            double sampled_dist = distanceSampler -> get_sample(&pdf_dist);
            delete distanceSampler;
            if (sampled_dist >= i.t) {
              ita.interacted = false;
            }
            else {
              Vector3D next_ita_point = new_ray.o + new_ray.d * sampled_dist;
              SchlickPhase *phase_pos = new SchlickPhase(pos2phase(next_ita_point));

              ita.interacted = true;
              ita.t = sampled_dist;
              new_ray.max_t = sampled_dist;
              ita.n = -new_ray.d;
              ita.phase = phase_pos;
            } 
            Spectrum radiance_in = at_least_one_bounce_radiance(new_ray, i, ita);
            if (pdf_dir != 0)
              L_out += (1. / 2. ) * pos2scattering(hit_p) / pos2extinction(hit_p) *
                radiance_in * sampled_phase_f / pdf_dir / cpdf;
          }
        }
      }
      // if (r.depth == max_ray_depth) {
      //   std::cout << L_out << std::endl;
      // }
      
      return L_out;
    }
  }

  Spectrum PathTracer::est_radiance_global_illumination(Ray &r) {
    Intersection isect;
    Interaction interact;
    Spectrum L_out = Spectrum();

    // You will extend this in assignment 3-2. 
    // If no intersection occurs, we simply return black.
    // This changes if you implement hemispherical lighting for extra credit.

    if (!bvh->intersect(r, &isect)) {
      isect.t = INF_D;
      // return envLight ? envLight -> sample_dir(r) : L_out;
      // return L_out;
    }

    // This line returns a color depending only on the normal vector 
    // to the surface at the intersection point.
    // REMOVE IT when you are ready to begin Part 3.

    // return normal_shading(isect.n);

    // TODO (Part 3): Return the direct illumination.

    // L_out = estimate_direct_lighting_importance(r, isect);
    // return L_out;

    // TODO (Part 4): Accumulate the "direct" and "indirect" 
    // parts of global illumination into L_out rather than just direct

    // Here we estimate the radiance of the ray which just came out from the 
    // camera freshly. Sample the distance of the first interaction (reflection/
    // /scattering) here.
    for (size_t i = 0; i < ns_dist; i++) {
      float pdf;
      DistanceSampler1D* distanceSampler = new DistanceSampler1D(&pos2extinction, space_step);
      distanceSampler -> set_ray(&r);
      distanceSampler -> set_max_t(isect.t + EPS_F);
      double sampled_dist = distanceSampler -> get_sample(&pdf);
      delete distanceSampler;
      // std::cout << "isect: " << isect.t << std::endl;

      // if sampled distance is no less than the distance to the nearest surface, 
      // then the emission from the surface successfully penetrate the medium and 
      // arrive at the camera
      if (sampled_dist >= isect.t) {
        // std::cout << "reflect " << sampled_dist << " " << isect.t << std::endl;
  
        interact.interacted = false;
        Spectrum to_add = 1. / double(ns_dist) *
        // Spectrum to_add = 1. / double(ns_dist) * pre_pdf / pdf *
          (zero_bounce_radiance(r, isect, interact) + 
          at_least_one_bounce_radiance(r, isect, interact));
        L_out += to_add;
        // std::cout << "reflect " << to_add << std::endl;
      }
      // if sampled distance is no less than the distance to the nearest surface, 
      else {
        // std::cout << "scatter " << sampled_dist << std::endl;
        
        interact.interacted = true;
        Vector3D next_ita_point = r.o + r.d * sampled_dist;
        SchlickPhase *phase_pos = new SchlickPhase(pos2phase(next_ita_point));
        
        interact.t = sampled_dist;
        r.max_t = sampled_dist;
        interact.n = -r.d;
        interact.phase = phase_pos;
        Spectrum to_add = 1. / double(ns_dist) * 
          (zero_bounce_radiance(r, isect, interact) + 
          at_least_one_bounce_radiance(r, isect, interact));
        L_out += to_add;
      }
    }

      // one_bounce_radiance(r, isect);
      // zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect) - one_bounce_radiance(r, isect);
    return L_out;
  }

  Spectrum PathTracer::raytrace_pixel(size_t x, size_t y, bool useThinLens) {
    // TODO (Part 1.1):
    // Make a loop that generates num_samples camera rays and traces them 
    // through the scene. Return the average Spectrum. 
    // You should call est_radiance_global_illumination in this function.

    // TODO (Part 5):
    // Modify your implementation to include adaptive sampling.
    // Use the command line parameters "samplesPerBatch" and "maxTolerance"

    int num_samples = ns_aa;            // total samples to evaluate
    Vector2D origin = Vector2D(double(x),double(y));    // bottom left corner of the pixel
    double width = double(sampleBuffer.w);
    double height = double(sampleBuffer.h);
    
    Spectrum radiance_sum = Spectrum(0, 0, 0);
    // max_ray_depth = 4;

    if (ns_aa == 1) {
      Vector2D p = origin + Vector2D(.5, .5);
      Ray ray = camera -> generate_ray(p.x / width, p.y / height);
      ray.depth = max_ray_depth;
      Spectrum radiance_in = est_radiance_global_illumination(ray);
      radiance_sum += radiance_in;
      
      sampleCountBuffer[x + y * frameBuffer.w] = 1;
      return radiance_sum;
    }
    else {
      int i;
      double n;
      double s1 = 0;
      double s2 = 0;

      for (i = 0; i != num_samples; i++) {
        Vector2D p = origin + gridSampler -> get_sample();

        Vector2D samplesForLens = gridSampler -> get_sample();
        // Ray ray = camera -> generate_ray_for_thin_lens(
        //   p.x / width, p.y / height, 
        //   samplesForLens.x, samplesForLens.y * 2.0 * PI);
        
        Ray ray = camera -> generate_ray(p.x / width, p.y / height);
        
        ray.depth = max_ray_depth;
        Spectrum radiance_in = est_radiance_global_illumination(ray);
        radiance_sum += radiance_in;
        
        //////////////////////////////////////////////////
        double illum_in = radiance_in.illum();
        s1 += illum_in;
        s2 = s2 + illum_in * illum_in;

        if ((i + 1) % samplesPerBatch == 0) {
          n = i + 1;
          double myu = s1 / n;
          double sigma2 = (s2 - s1 * s1 / n) / (n - 1);
          double I = 1.96 * sqrt(sigma2 / n);
          if (I <= maxTolerance * myu)
            break;
        }
        //////////////////////////////////////////////////
      }
      sampleCountBuffer[x + y * frameBuffer.w] = i;
      return radiance_sum / (i);
    }
  }

  // Spectrum PathTracer::raytrace_pixel(size_t x, size_t y, bool useThinLens) {
  //   // TODO (Part 1.1):
  //   // Make a loop that generates num_samples camera rays and traces them 
  //   // through the scene. Return the average Spectrum. 
  //   // You should call est_radiance_global_illumination in this function.

  //   // TODO (Part 5):
  //   // Modify your implementation to include adaptive sampling.
  //   // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  //   int num_samples = ns_aa;            // total samples to evaluate
  //   Vector2D origin = Vector2D(double(x),double(y));    // bottom left corner of the pixel
  //   double width = double(sampleBuffer.w);
  //   double height = double(sampleBuffer.h);
    
  //   Spectrum radiance_sum = Spectrum(0, 0, 0);
  //   // max_ray_depth = 4;

  //   if (ns_aa == 1) {
  //     Vector2D p = origin + Vector2D(.5, .5);
  //     Ray ray = camera -> generate_ray(p.x / width, p.y / height);
  //     ray.depth = max_ray_depth;
  //     Spectrum radiance_in = est_radiance_global_illumination(ray);
  //     radiance_sum += radiance_in;
      
  //     sampleCountBuffer[x + y * frameBuffer.w] = 1;
  //     return radiance_sum;
  //   }
  //   else {
  //     int i;
  //     double n;
  //     double s1 = 0;
  //     double s2 = 0;

  //     for (i = 0; i != num_samples; i++) {
  //       Vector2D p = origin + gridSampler -> get_sample();
  //       Ray ray = camera -> generate_ray(p.x / width, p.y / height);
  //       ray.depth = max_ray_depth;
  //       Spectrum radiance_in = est_radiance_global_illumination(ray);
  //       radiance_sum += radiance_in;
        
  //       //////////////////////////////////////////////////
  //       double illum_in = radiance_in.illum();
  //       s1 += illum_in;
  //       s2 = s2 + illum_in * illum_in;

  //       if ((i + 1) % samplesPerBatch == 0) {
  //         n = i + 1;
  //         double myu = s1 / n;
  //         double sigma2 = (s2 - s1 * s1 / n) / (n - 1);
  //         double I = 1.96 * sqrt(sigma2 / n);
  //         if (I <= maxTolerance * myu)
  //           break;
  //       }
  //       //////////////////////////////////////////////////
  //     }
  //     sampleCountBuffer[x + y * frameBuffer.w] = i;
  //     return radiance_sum / (i);
  //   }
  // }

  // Diffuse BSDF //

  Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
    // This function takes in both wo and wi and returns the evaluation of
    // the BSDF for those two directions.

    return reflectance / PI;
  }

  Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
    // This function takes in only wo and provides pointers for wi and pdf,
    // which should be assigned by this function.
    // After sampling a value for wi, it returns the evaluation of the BSDF
    // at (wo, *wi).
    *wi = sampler.get_sample(pdf);
    // return reflectance / PI;
    return f(wo, *wi);
  }

  // Camera //
  Ray Camera::generate_ray(double x, double y) const {
    // TODO (Part 1.2):
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Note: hFov and vFov are in degrees.
    // 
    double h_bias = tan(radians(hFov)*.5);
    double v_bias = tan(radians(vFov)*.5);
    Vector3D p_on_sensor = Vector3D(
      -h_bias + x * (2 * h_bias), 
      -v_bias + y * (2 * v_bias), 
      -1
      );
    Vector3D direction = c2w * p_on_sensor;
    direction.normalize();
    Ray r = Ray(pos, direction);
    // cout << x << " " << y << endl;
    r.min_t = nClip;
    r.max_t = fClip;
    return r;
  }
}
