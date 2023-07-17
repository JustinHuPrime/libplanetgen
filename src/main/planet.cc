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
#include <cmath>
#include <execution>
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
float angleBetween(vec3 const &a, vec3 const &b) {
  return acos(glm::clamp(dot(normalize(a), normalize(b)), -1.f, 1.f));
}
}  // namespace

Plate::Plate(bool major, bool continental, TerrainData const &center,
             mt19937_64 &rng, float baseWeight,
             float maxWeightDeviation) noexcept
    : major(major), continental(continental), center(center) {
  uniform_real_distribution<float> distribution =
      uniform_real_distribution<float>(baseWeight * (1.f - maxWeightDeviation),
                                       baseWeight * (1.f + maxWeightDeviation));
  transform(directionalWeights.begin(), directionalWeights.end(),
            directionalWeights.begin(),
            [&rng, &distribution](float) { return distribution(rng); });
}
float Plate::weightedDistanceTo(TerrainData const &point) const noexcept {
  // avoid division by zero issues
  if (&point == &center) {
    return 0.f;
  }

  // get unweighted angle
  float unweightedAngle = angleBetween(center.centroid, point.centroid);

  // project direction to terrain centroid onto center plane
  vec3 toPoint = point.centroid - center.centroid;
  vec3 toPointProjected = toPoint - center.normal * dot(toPoint, center.normal);

  // project northward vector onto center plane
  vec3 north = vec3{0, 1, 0};
  vec3 northProjected = north - center.normal * dot(north, center.normal);

  // calculate angle between line from center to projected point and center to
  // north
  float angle = angleBetween(toPointProjected, northProjected);
  assert(!isnan(angle));

  // maybe need to subtract from 2pi if toPointProjected is to the left of
  // northProjected (to make it a reflex angle)
  if (dot(cross(toPointProjected, northProjected), center.normal) < 0) {
    angle = 2.f * M_PIf - angle;
  }

  // find which of the 20 sectors this falls in
  float sectorSize =
      2.f * M_PIf / static_cast<float>(directionalWeights.size());
  size_t sector = static_cast<size_t>(floor(angle / sectorSize)) &
                  directionalWeights.size();

  float proportion = fmod(angle, sectorSize);

  // lerp between the two weighting anchor points
  float weight =
      lerp(directionalWeights[(sector + 1) % directionalWeights.size()],
           directionalWeights[sector], proportion);

  // apply weighting
  return glm::clamp(unweightedAngle - weight, 0.f, M_PIf);
}

TerrainData::TerrainData(std::array<glm::vec3, 3> const &vertices) noexcept
    : centroid(accumulate(vertices.begin(), vertices.end(), vec3{0, 0, 0}) /
               3.f),
      normal(normalize(
          cross(vertices[1] - vertices[0], vertices[2] - vertices[0]))) {}

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
bool TriangleTerrainTreeNode::contains(vec3 const &rayVector) noexcept {
  // Moller-Trumbore algorithm - see
  // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

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
TerrainData &QuadTerrainTreeNode::operator[](vec3 const &rayVector) noexcept {
  auto found =
      find_if(children.begin(), children.end(),
              [&rayVector](unique_ptr<TriangleTerrainTreeNode> const &child) {
                return child->contains(rayVector);
              });
  if (found != children.end()) {
    return (**found)[rayVector];
  } else {
    found = max_element(
        children.begin(), children.end(),
        [&rayVector](unique_ptr<TriangleTerrainTreeNode> const &lhs,
                     unique_ptr<TriangleTerrainTreeNode> const &rhs) {
          return dot(rayVector,
                     accumulate(lhs->getVertices().begin(),
                                lhs->getVertices().end(), vec3{0, 0, 0}) /
                         3.f) <
                 dot(rayVector,
                     accumulate(rhs->getVertices().begin(),
                                rhs->getVertices().end(), vec3{0, 0, 0}) /
                         3.f);
        });
    return (**found)[rayVector];
  }
}
void QuadTerrainTreeNode::forEach(function<void(TerrainData &)> const &f) {
  for_each(children.begin(), children.end(),
           [&f](unique_ptr<TriangleTerrainTreeNode> const &child) {
             child->forEach(f);
           });
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
    : TriangleTerrainTreeNode(vertices), data(vertices) {}
TerrainData &LeafTerrainTreeNode::operator[](vec3 const &) noexcept {
  return data;
}
void LeafTerrainTreeNode::forEach(function<void(TerrainData &)> const &f) {
  return f(data);
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
    vec3 const &rayVector) noexcept {
  auto found =
      find_if(children.begin(), children.end(),
              [&rayVector](unique_ptr<TriangleTerrainTreeNode> const &child) {
                return child->contains(rayVector);
              });
  if (found != children.end()) {
    return (**found)[rayVector];
  } else {
    found = max_element(
        children.begin(), children.end(),
        [&rayVector](unique_ptr<TriangleTerrainTreeNode> const &lhs,
                     unique_ptr<TriangleTerrainTreeNode> const &rhs) {
          return dot(rayVector,
                     accumulate(lhs->getVertices().begin(),
                                lhs->getVertices().end(), vec3{0, 0, 0}) /
                         3.f) <
                 dot(rayVector,
                     accumulate(rhs->getVertices().begin(),
                                rhs->getVertices().end(), vec3{0, 0, 0}) /
                         3.f);
        });
    return (**found)[rayVector];
  }
}
void IcosahedronTerrainTreeNode::forEach(
    function<void(TerrainData &)> const &f) {
  for_each(execution::par_unseq, children.begin(), children.end(),
           [&f](unique_ptr<TriangleTerrainTreeNode> const &child) {
             child->forEach(f);
           });
}

EarthlikePlanet::EarthlikePlanet(atomic<GenerationStatus> &statusReport,
                                 uint64_t seed, Config const &config) noexcept
    : data() {
  // step 0: initialization

  // step 0.1: create map structure
  statusReport = GenerationStatus::TRIANGULATING;
  data =
      make_unique<IcosahedronTerrainTreeNode>(config.radius, config.resolution);

  // step 0.2: initialize rng
  mt19937_64 rng = mt19937_64(seed);

  // step 1: generate plates and heightmap (see
  // https://www.youtube.com/watch?v=x_Tn66PvTn4)

  // step 1.1: generate plates
  uniform_real_distribution<float> zeroToOne(0.f, 1.f);
  uniform_real_distribution<float> negOneToOne(-1.f, 1.f);

  while (plates.size() < config.numMajorPlates + config.numMinorPlates) {
    bool generatingMajorPlate = plates.size() < config.numMajorPlates;

    // step 1.1.1.1: generate random point
    float lon = 2.f * M_PIf * zeroToOne(rng);
    float lat = acos(negOneToOne(rng)) - M_PI_2f;
    vec3 attempt = sphericalToCartesian(vec2{lat, lon});

    // step 1.1.1.2: require that points be far enough away from existing points
    if (any_of(plates.begin(), plates.end(),
               [this, &attempt, &config,
                &generatingMajorPlate](Plate const &plate) {
                 if (generatingMajorPlate || plate.major) {
                   return angleBetween(attempt, plate.center.centroid) <
                          config.minMajorPlateAngle;
                 } else {
                   return angleBetween(attempt, plate.center.centroid) <
                          config.minMinorPlateAngle;
                 }
               })) {
      continue;
    }

    // step 1.1.1.3: place point
    plates.emplace_back(
        generatingMajorPlate,
        zeroToOne(rng) > config.oceanicFraction, operator[](attempt), rng,
        generatingMajorPlate ? config.majorPlateSizeBonus : 0.f,
        config.maxPlateRoughness);
  }

  // step 1.1.2: associate terrain tiles to plates
  data->forEach([this](TerrainData &data) {
    data.plate = &*min_element(
        plates.begin(), plates.end(), [&data](Plate const &a, Plate const &b) {
          return a.weightedDistanceTo(data) < b.weightedDistanceTo(data);
        });
  });

  // step 1.2: set plate motion

  // TODO

  // step 1.3: generate heightmap

  // TODO

  // step 2: calculate prevailing winds (see
  // https://www.youtube.com/watch?v=LifRswfCxFU)

  // step 2.1: generate trade winds

  // TODO

  // step 2.2: generate polar cells

  // TODO

  // step 3.3: generate ferrel cell

  // TODO

  // step 3: calculate currents (see
  // https://www.youtube.com/watch?v=n_E9UShtyY8)

  // step 3.1: equatorial gyres

  // TODO

  // step 3.2: ferrel gyres

  // TODO

  // step 3.3: circumpolar currents

  // TODO

  // step 3.4: fill in gaps

  // TODO

  // step 3.5: ENSO event zones

  // TODO

  // step 4: calculate biomes and climate (see
  // https://www.youtube.com/watch?v=5lCbxMZJ4zA and
  // https://www.youtube.com/watch?v=fag48Nh8PXE)

  // step 4.1: preperatory calculations

  // step 4.1.1: precipitation levels

  // TODO

  // step 4.1.2: temperature levels

  // TODO

  // step 4.1.3: orthographic lift

  // TODO

  // step 4.2: place biomes

  // step 4.2.1: mountain climates

  // TODO

  // step 4.2.2: tropical climates

  // TODO

  // step 4.2.3: continental climates

  // TODO

  // step 4.2.4: polar climates

  // TODO

  // step 5: generate rivers (see
  // https://www.youtube.com/watch?v=cqMiMKnYk5E)

  // step 5.1: drainage basins

  // TODO

  // step 5.2: river basins

  // TODO

  // step 5.3: primary rivers

  // TODO

  // step 5.4: tributaries

  // TODO

  // step 6: calculate resource distribution (see
  // https://www.youtube.com/watch?v=b9qvQspSbWc)

  // step 6.1: calculate historical heightmap

  // TODO

  // step 6.2: calculate historical tectonic plates

  // TODO

  // step 6.3: calculate historical currents

  // TODO

  // step 6.4: calculate historical biomes

  // TODO

  // step 6.5: place resources

  // step 6.5.1: coal

  // TODO

  // step 6.5.2: oil

  // TODO

  // step 6.5.3: ores

  // TODO

  // step 7: generate world history

  // step 7.1: generate classical empires

  // TODO

  // step 7.2: form medieval nations

  // TODO

  // step 7.3: form renaissance nations

  // TODO

  // step 7.3: simulate colonization

  // TODO

  // step 7.3: simulate industrial revolution

  // TODO

  // step 7.4: simulate world wars

  // TODO

  // step 7.5: create superpower blocs

  statusReport = GenerationStatus::DONE;
}
TerrainData &EarthlikePlanet::operator[](vec2 const &location) noexcept {
  return operator[](sphericalToCartesian(location));
}
TerrainData &EarthlikePlanet::operator[](vec3 const &location) noexcept {
  return (*data)[location];
}
}  // namespace planetgen
