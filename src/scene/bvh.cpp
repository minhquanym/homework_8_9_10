#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  int numPrim = 0;
  BBox bbox, centroidBox;
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
    Vector3D centroid = bb.centroid();
    centroidBox.expand(centroid);
    numPrim++;
  }

  BVHNode *node = new BVHNode(bbox);
  node->start = start;
  node->end = end;

  // Leaf node
  if (numPrim <= max_leaf_size) {
    return node;
  }


  int axisNum;
  Vector3D mainExtent = centroidBox.extent;
  if (mainExtent.x > mainExtent.y && mainExtent.x > mainExtent.z) {
    axisNum = 0;
  } else if (mainExtent.y > mainExtent.z && mainExtent.y > mainExtent.x) {
    axisNum = 1;
  } else {
    axisNum = 2;
  }

  Vector3D splitCentroid = centroidBox.centroid();


  std::vector<Primitive *> left, right;
  // For loop to split primitives
  for (auto p = start; p != end; p++) {
    Vector3D pCentroid = (*p)->get_bbox().centroid();
    switch (axisNum) {
    case 0:
      if (pCentroid.x < splitCentroid.x) {
        left.push_back(*p);
      } else {
        right.push_back(*p);
      }
      break;
    case 1:
      if (pCentroid.y < splitCentroid.y) {
        left.push_back(*p);
      } else {
        right.push_back(*p);
      }
      break;
    case 2:
      if (pCentroid.z < splitCentroid.z) {
        left.push_back(*p);
      } else {
        right.push_back(*p);
      }
      break;
    }
  }

  auto endLeft = start;
  for (int i = 0; i < left.size(); i++) {
    *endLeft = left[i];
    endLeft++;
  }
  node->l = construct_bvh(start, endLeft, max_leaf_size);

  auto beginRight = endLeft;
  for (int i = 0; i < right.size(); i++) {
    *endLeft = right[i];
    endLeft++;
  }
  node->r = construct_bvh(beginRight, endLeft, max_leaf_size);
  return node;

}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  double t0, t1;
  if (!node->bb.intersect(ray, t0, t1)) {
    return false;
  }
  else {
    if (t1 < ray.min_t || t0 > ray.max_t) {
      return false;
    }
    else {
      
      // node = leaf node
      if (node->isLeaf()) {
        bool hit = false;
        for (auto p = node->start; p != node->end; p++) {
          total_isects++;
          if ((*p)->has_intersection(ray)) {
            // return true;
            hit = true;
          }
        }
        return hit;
      }
      // node != leaf node
      else {
        bool left = false;
        bool right = false;
        left = left || has_intersection(ray, node->l);
        right = right || has_intersection(ray, node->r);
        return left || right;
      }
    }
  }
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  double t0, t1;
  if (!node->bb.intersect(ray, t0, t1)) {
    return false;
  }
  else {
    // return true;
    if (t1 < ray.min_t || t0 > ray.max_t) {
      return false;
    }
    else {
      // node = leaf node  
      if (node->isLeaf()) {
        bool hit = false;
        for (auto p = node->start; p != node->end; p++) {
          total_isects++;
          if ((*p)->intersect(ray, i)) {
          
            hit = true;
          }
        }
        return hit;
      }
      // node != leaf node
      else {
        bool left = false;
        bool right = false;
        left = left || intersect(ray, i, node->l);
        right = right || intersect(ray, i, node->r);
        return left || right;
      } 
    }
  }

}

} // namespace SceneObjects
} // namespace CGL
