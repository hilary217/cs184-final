#include "bvh.h"

#include "CGL/CGL.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL { namespace StaticScene {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  root = construct_bvh(_primitives, max_leaf_size);

}

BVHAccel::~BVHAccel() {
  if (root) delete root;
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

void BVHAccel::draw(BVHNode *node, const Color& c, float alpha) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->draw(c, alpha);
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color& c, float alpha) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->drawOutline(c, alpha);
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(const std::vector<Primitive*>& prims, size_t max_leaf_size) {
  
  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  // BBox centroid_box, bbox;

  // for (Primitive *p : prims) {
  //     BBox bb = p->get_bbox();
  //     bbox.expand(bb);
  //     Vector3D c = bb.centroid();
  //     centroid_box.expand(c);
  // }

  // BVHNode *node = new BVHNode(bbox);


  // node->prims = new vector<Primitive *>(prims);
  // return node;

  BBox bbox;
  for (Primitive *p : prims) {
    bbox.expand(p->get_bbox());
  }

  BVHNode *node = new BVHNode(bbox);
  
  if (prims.size() <= max_leaf_size) {
    // cout << "leaf" << endl;
    // cout << prims.size() << endl;

    node -> prims = new vector<Primitive *>(prims);
    return node;
  }

  Vector3D bbox_extent = bbox.extent;
  int split_axis = 
    (bbox_extent[0] > bbox_extent[1]) ? 
    ((bbox_extent[0] > bbox_extent[2]) ? 0 : 2) :
    ((bbox_extent[1] > bbox_extent[2]) ? 1 : 2);
  
  // using midpoint of bbox as split point
  Vector3D split_point = (bbox.max + bbox.min) / 2;
  double split_point_value = split_point[split_axis];

  vector<Primitive *> l_prims = vector<Primitive *>();
  vector<Primitive *> r_prims = vector<Primitive *>();

  for (Primitive *p : prims) {
    if (p -> get_bbox().centroid()[split_axis] < split_point_value)
      l_prims.push_back(p);
    else
      r_prims.push_back(p);
  }
  
  while (l_prims.empty() or r_prims.empty()) {
    // cout << "dead" << endl;
    // cout << l_prims.size() << endl;
    // cout << r_prims.size() << endl;
    // cout <<split_point_value << endl;
    double new_split_point = 0.;
    double n_prims = 0.;
    for (Primitive *p : prims) {
      n_prims += 1;
      new_split_point += p -> get_bbox().centroid()[split_axis];
    }
    split_point_value = new_split_point / n_prims;

    if (l_prims.empty()) {
      // split_point_value = (bbox.max[split_axis] + split_point_value) / 2;
      r_prims.clear();
    }
    else {
      // split_point_value = (bbox.min[split_axis] + split_point_value) / 2;
      l_prims.clear();
    }
    for (Primitive *p : prims) {
      if (p -> get_bbox().centroid()[split_axis] < split_point_value)
        l_prims.push_back(p);
      else
        r_prims.push_back(p);
    }
  }

  node -> l = construct_bvh(l_prims, max_leaf_size);
  node -> r = construct_bvh(r_prims, max_leaf_size);

  // cout << node -> l -> isLeaf() << endl;
  // cout << node -> r -> isLeaf() << endl;
  
  return node;
}


bool BVHAccel::intersect(const Ray& ray, BVHNode *node) const {

  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  // for (Primitive *p : *(root->prims)) {
  //   total_isects++;
  //   if (p->intersect(ray)) 
  //     return true;
  // }
  // return false;

  // cout << "hello" << endl;
  if (not node -> bb.intersect(ray, ray.min_t, ray.max_t)) {
    return false;
  }
  // cout << "hello" << endl << endl;
  if (node -> isLeaf()) {
    for (Primitive *p : *(node -> prims)) {
      if (p -> intersect(ray)) {
        return true;
      }
    }
    return false;
  }
  bool l_intersect = intersect(ray, node -> l);
  bool r_intersect = intersect(ray, node -> r);
  return l_intersect or r_intersect;
}

bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode *node) const {

  // TODO (Part 2.3):
  // Fill in the intersect function.

  // bool hit = false;
  // for (Primitive *p : *(root->prims)) {
  //   total_isects++;
  //   if (p->intersect(ray, i)) 
  //     hit = true;
  // }
  // return hit;
  
  // cout << "hello" << endl;
  if (not node -> bb.intersect(ray, ray.min_t, ray.max_t)) {
    return false;
  }
  // cout << "hello" << endl << endl;
  if (node -> isLeaf()) {
    bool hit = false;
    for (Primitive *p : *(node -> prims)) {
      if (p->intersect(ray, i)) {
        // total_isects++;
        hit = true;
      }
    }
    return hit;
  }
  bool l_intersect = intersect(ray, i, node -> l);
  bool r_intersect = intersect(ray, i, node -> r);
  return l_intersect or r_intersect;
  
}

}  // namespace StaticScene
}  // namespace CGL
