// Copyright 2023 Justin Hu
//
// This file is part of libplanetgen.
//
// libplanetgen is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// libplanetgen is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// libplanetgen. If not, see <https://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "planet.h"

#include <algorithm>
#include <glm/gtc/epsilon.hpp>

using namespace std;
using namespace glm;

namespace planetgen {
namespace {
/**
 * Convert lat-long coordinate ray to OpenGL coordinate system ray
 *
 * Assumes camera is over the prime meridian (e.g. 0 N, 0 E = 0, 0, 1)
 *
 * @param latLon latitude (radians north) and longitude (radians east) (in that
 * order) as a vec2
 *
 * @returns direction vector that points to the same spot as the lat-lon
 * coordinates
 */
constexpr vec3 sphericalToCartesian(vec2 const &latLon) noexcept {
  float lat = latLon.s;
  float lon = latLon.t;
  return vec3{cos(lat) * sin(lon), sin(lat), cos(lat) * cos(lon)};
}
constexpr float TRIANGLE_INTERSECTION_EPSILON = 1e-9f;
}  // namespace

TriangleTerrainTreeNode::TriangleTerrainTreeNode(
    array<vec3, 3> const &vertices) noexcept
    : vertices(vertices) {
#ifndef NDEBUG
  // invariant - this triangle starts approximately equilateral
  float side0 = length(vertices[0] - vertices[1]);
  float side1 = length(vertices[1] - vertices[2]);
  float side2 = length(vertices[2] - vertices[0]);
  assert(epsilonEqual(side0 / side1, 1.f, 1e-3f));
  assert(epsilonEqual(side1 / side2, 1.f, 1e-3f));
#endif
}
bool TriangleTerrainTreeNode::contains(vec2 const &location) noexcept {
  // Moller-Trumbore algorithm - see
  // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
  vec3 rayVector = sphericalToCartesian(location);

  vec3 edge1 = vertices[1] - vertices[0];
  vec3 edge2 = vertices[2] - vertices[0];

  vec3 horizontal = cross(rayVector, edge2);
  float alignment = dot(edge1, horizontal);

  if (alignment > -TRIANGLE_INTERSECTION_EPSILON) {
    // ray does not hit the right face
    return false;
  }

  // find first barycentric coord
  vec3 s = -vertices[0];
  float u = dot(s, horizontal) / alignment;

  // bounds check for the triangle
  if (u < -TRIANGLE_INTERSECTION_EPSILON ||
      u > 1.0 + TRIANGLE_INTERSECTION_EPSILON) {
    return false;
  }

  // find second barycentric coord
  vec3 q = cross(s, edge1);
  float v = dot(rayVector, q) / alignment;

  // check we're in the triangle
  if (v < -TRIANGLE_INTERSECTION_EPSILON ||
      u + v > 1.0 + TRIANGLE_INTERSECTION_EPSILON) {
    return false;
  }

  // check we are in the positive part of the ray
  return dot(edge2, q) / alignment > TRIANGLE_INTERSECTION_EPSILON;
}
void TriangleTerrainTreeNode::inflate(float radius) noexcept {
  for_each(vertices.begin(), vertices.end(),
           [&radius](vec3 &vertex) { vertex = radius * normalize(vertex); });
}
array<vec3, 3> const &TriangleTerrainTreeNode::getVertices() const noexcept {
  return vertices;
}

QuadTerrainTreeNode::QuadTerrainTreeNode(array<vec3, 3> const &vertices_,
                                         float resolution) noexcept
    : TriangleTerrainTreeNode(vertices_) {
  array<vec3, 3> innerVertices = {(vertices[0] + vertices[1]) / 2.f,
                                  (vertices[1] + vertices[2]) / 2.f,
                                  (vertices[2] + vertices[0]) / 2.f};

  if (length(innerVertices[0] - innerVertices[1]) <= resolution) {
    children[0] = make_unique<LeafTerrainTreeNode>(
        array<vec3, 3>{vertices[0], innerVertices[0], innerVertices[2]});
    children[1] = make_unique<LeafTerrainTreeNode>(
        array<vec3, 3>{innerVertices[0], vertices[1], innerVertices[1]});
    children[2] = make_unique<LeafTerrainTreeNode>(
        array<vec3, 3>{innerVertices[2], innerVertices[1], vertices[2]});
    children[3] = make_unique<LeafTerrainTreeNode>(
        array<vec3, 3>{innerVertices[0], innerVertices[1], innerVertices[2]});
  } else {
    children[0] = make_unique<QuadTerrainTreeNode>(
        array<vec3, 3>{vertices[0], innerVertices[0], innerVertices[2]},
        resolution);
    children[1] = make_unique<QuadTerrainTreeNode>(
        array<vec3, 3>{innerVertices[0], vertices[1], innerVertices[1]},
        resolution);
    children[2] = make_unique<QuadTerrainTreeNode>(
        array<vec3, 3>{innerVertices[2], innerVertices[1], vertices[2]},
        resolution);
    children[3] = make_unique<QuadTerrainTreeNode>(
        array<vec3, 3>{innerVertices[0], innerVertices[1], innerVertices[2]},
        resolution);
  }
}
TerrainData &QuadTerrainTreeNode::operator[](vec2 const &location) noexcept {
  auto found =
      find_if(children.begin(), children.end(),
              [&location](unique_ptr<TriangleTerrainTreeNode> const &child) {
                return child->contains(location);
              });
  if (found != children.end()) {
    return (**found)[location];
  } else {
    found = max_element(
        children.begin(), children.end(),
        [&location](unique_ptr<TriangleTerrainTreeNode> const &lhs,
                    unique_ptr<TriangleTerrainTreeNode> const &rhs) {
          vec3 rayVector = sphericalToCartesian(location);
          return dot(rayVector,
                     accumulate(lhs->getVertices().begin(),
                                lhs->getVertices().end(), vec3{0, 0, 0}) /
                         3.f) <
                 dot(rayVector,
                     accumulate(rhs->getVertices().begin(),
                                rhs->getVertices().end(), vec3{0, 0, 0}) /
                         3.f);
        });
    return (**found)[location];
  }
}
void QuadTerrainTreeNode::inflate(float radius) noexcept {
  for_each(children.begin(), children.end(),
           [&radius](unique_ptr<TriangleTerrainTreeNode> &child) {
             return child->inflate(radius);
           });
  TriangleTerrainTreeNode::inflate(radius);

#ifndef NDEBUG
  // invariant: children 0, 1, 2 share vertices 0, 1, 2 with this
  for (size_t idx = 0; idx < vertices.size(); ++idx) {
    assert(find(children[idx]->getVertices().begin(),
                children[idx]->getVertices().end(),
                vertices[idx]) != children[idx]->getVertices().end());
  }
  // invariant: children 0, 1, 2, share two vertices with child 3
  for (size_t childIdx = 0; childIdx < 3; ++childIdx) {
    int found = 0;
    for (vec3 const &vertex : children[3]->getVertices()) {
      if (find(children[childIdx]->getVertices().begin(),
               children[childIdx]->getVertices().end(),
               vertex) != children[childIdx]->getVertices().end()) {
        ++found;
      }
    }
    assert(found == 2);
  }
#endif
}

LeafTerrainTreeNode::LeafTerrainTreeNode(
    array<vec3, 3> const &vertices) noexcept
    : TriangleTerrainTreeNode(vertices), data() {}
TerrainData &LeafTerrainTreeNode::operator[](vec2 const &) noexcept {
  return data;
}

IcosahedronTerrainTreeNode::IcosahedronTerrainTreeNode(
    float radius, float resolution) noexcept {
  // north polar cap
  vec3 northPole = radius * vec3{0, 1, 0};
  float tropic = atan(0.5f);
  array<vec3, 5> northVertices;
  for (size_t idx = 0; idx < northVertices.size(); ++idx) {
    northVertices[idx] =
        radius * sphericalToCartesian(vec2{tropic, idx * 2 * M_PIf / 5.f});
  }

  for (size_t idx = 0; idx < 5; ++idx) {
    children[idx] = make_unique<QuadTerrainTreeNode>(
        array<vec3, 3>{northVertices[idx % 5], northVertices[(idx + 1) % 5],
                       northPole},
        resolution);
  }

  // south polar cap
  vec3 southPole = radius * vec3{0, -1, 0};
  array<vec3, 5> southVertices;
  for (size_t idx = 0; idx < southVertices.size(); ++idx) {
    southVertices[idx] =
        radius * sphericalToCartesian(
                     vec2{-tropic, 2 * M_PIf / 10.f + idx * 2 * M_PIf / 5.f});
  }

  for (size_t idx = 0; idx < 5; ++idx) {
    children[idx + 5] = make_unique<QuadTerrainTreeNode>(
        array<vec3, 3>{southVertices[idx % 5], southPole,
                       southVertices[(idx + 1) % 5]},
        resolution);
  }

  // rest of northern hemisphere
  for (size_t idx = 0; idx < 5; ++idx) {
    children[idx + 10] = make_unique<QuadTerrainTreeNode>(
        array<vec3, 3>{northVertices[idx % 5], southVertices[idx % 5],
                       northVertices[(idx + 1) % 5]},
        resolution);
  }

  // rest of southern hemisphere
  for (size_t idx = 0; idx < 5; ++idx) {
    children[idx + 15] = make_unique<QuadTerrainTreeNode>(
        array<vec3, 3>{southVertices[idx % 5], southVertices[(idx + 1) % 5],
                       northVertices[(idx + 1) % 5]},
        resolution);
  }

  for_each(children.begin(), children.end(),
           [&radius](unique_ptr<TriangleTerrainTreeNode> &child) {
             return child->inflate(radius);
           });

#ifndef NDEBUG
  // invariant - each of the 12 vertices is seen five times
  array<vec3, 12> vertices = {
      radius * normalize(northPole),
      radius * normalize(southPole),
  };
  transform(
      northVertices.begin(), northVertices.end(), vertices.begin() + 2,
      [&radius](vec3 const &vertex) { return radius * normalize(vertex); });
  transform(
      southVertices.begin(), southVertices.end(), vertices.begin() + 2 + 5,
      [&radius](vec3 const &vertex) { return radius * normalize(vertex); });

  for (vec3 const &vertex : vertices) {
    int found = 0;
    for (unique_ptr<TriangleTerrainTreeNode> const &child : children) {
      if (find(child->getVertices().begin(), child->getVertices().end(),
               vertex) != child->getVertices().end()) {
        ++found;
      }
    }
    assert(found == 5);
  }
#endif
}
TerrainData &IcosahedronTerrainTreeNode::operator[](
    vec2 const &location) noexcept {
  auto found =
      find_if(children.begin(), children.end(),
              [&location](unique_ptr<TriangleTerrainTreeNode> const &child) {
                return child->contains(location);
              });
  if (found != children.end()) {
    return (**found)[location];
  } else {
    found = max_element(
        children.begin(), children.end(),
        [&location](unique_ptr<TriangleTerrainTreeNode> const &lhs,
                    unique_ptr<TriangleTerrainTreeNode> const &rhs) {
          vec3 rayVector = sphericalToCartesian(location);
          return dot(rayVector,
                     accumulate(lhs->getVertices().begin(),
                                lhs->getVertices().end(), vec3{0, 0, 0}) /
                         3.f) <
                 dot(rayVector,
                     accumulate(rhs->getVertices().begin(),
                                rhs->getVertices().end(), vec3{0, 0, 0}) /
                         3.f);
        });
    return (**found)[location];
  }
}

EarthlikePlanet::EarthlikePlanet(
    uint64_t seed, float radius, float resolution,
    atomic<GenerationStatus> &statusReport) noexcept
    : rngEngine(seed), data(), radius(radius), resolution(resolution) {
  statusReport = GenerationStatus::TRIANGULATING;
  data = make_unique<IcosahedronTerrainTreeNode>(radius, resolution);
  statusReport = GenerationStatus::DONE;
}
TerrainData &EarthlikePlanet::operator[](vec2 const &location) noexcept {
  return (*data)[location];
}
}  // namespace planetgen
