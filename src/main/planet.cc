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
#include <glm/glm.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <unordered_map>
#include <utility>

#include "perlin.h"
#include "util/forEachParallel.h"
#include "util/geometry.h"

using namespace std;
using namespace glm;
using namespace planetgen::util;

namespace planetgen {
namespace {
constexpr float TRIANGLE_INTERSECTION_EPSILON = 1e-9f;
float angleBetween(vec3 const &a, vec3 const &b) noexcept {
  return acos(glm::clamp(dot(normalize(a), normalize(b)), -1.f, 1.f));
}
vec3 randomPointInSphere(mt19937_64 &rng) noexcept {
  uniform_real_distribution<float> zeroToOne(0.f, 1.f);
  uniform_real_distribution<float> negOneToOne(-1.f, 1.f);
  float lon = 2.f * M_PIf * zeroToOne(rng);
  float lat = acos(negOneToOne(rng)) - M_PI_2f;
  return sphericalToCartesian(vec2{lat, lon});
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
float Plate::weightedDistanceTo(TerrainData const &point,
                                float bias) const noexcept {
  // avoid division by zero issues
  if (&point == &center) {
    return 0.f;
  }

  // get unweighted angle
  float unweightedAngle = angleBetween(center.centroid, point.centroid);

  // get angle around the plate
  float angle = angleAround(point);

  // find which of the 20 sectors this falls in
  float sectorSize =
      2.f * M_PIf / static_cast<float>(directionalWeights.size());
  size_t sector = static_cast<size_t>(floor(angle / sectorSize)) %
                  directionalWeights.size();

  float proportion = fmod(angle, sectorSize);

  // lerp between the two weighting anchor points
  assert(0 <= sector && sector < directionalWeights.size());
  float weight =
      lerp(directionalWeights[(sector + 1) % directionalWeights.size()],
           directionalWeights[sector], proportion);

  // apply weighting
  return glm::clamp(unweightedAngle - weight + bias, 0.f, M_PIf);
}
float Plate::angleAround(TerrainData const &point) const noexcept {
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

  return angle;
}

TerrainData::TerrainData(array<vec3, 3> const &vertices) noexcept
    : vertices(vertices) {}

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
    : TriangleTerrainTreeNode(vertices), data(this->vertices) {}
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
  forEachParallel(children.begin(), children.end(),
                  [&f](unique_ptr<TriangleTerrainTreeNode> const &child) {
                    child->forEach(f);
                  });
}

EarthlikePlanet::EarthlikePlanet(atomic<GenerationStatus> &statusReport,
                                 uint64_t seed, Config const &config) noexcept
    : data() {
  // step 0: initialization
  statusReport = GenerationStatus::TRIANGULATING;

  // step 0.1: create map structure
  data =
      make_unique<IcosahedronTerrainTreeNode>(config.radius, config.resolution);

  // step 0.2: calculate tile centroids, normals, and initialize B-tree
  data->forEach([this](TerrainData &data) {
    data.centroid =
        accumulate(data.vertices.begin(), data.vertices.end(), vec3{0, 0, 0}) /
        3.f;
    data.normal = normalize(cross(data.vertices[1] - data.vertices[0],
                                  data.vertices[2] - data.vertices[0]));
    data.neighbours[0] = &operator[](
        data.vertices[0] + 1.5f * ((data.vertices[1] + data.vertices[2]) / 2.f -
                                   data.vertices[0]));
    data.neighbours[1] = &operator[](
        data.vertices[1] + 1.5f * ((data.vertices[0] + data.vertices[2]) / 2.f -
                                   data.vertices[1]));
    data.neighbours[2] = &operator[](
        data.vertices[2] + 1.5f * ((data.vertices[0] + data.vertices[1]) / 2.f -
                                   data.vertices[2]));
  });

  // step 0.3: initialize rng
  mt19937_64 rng = mt19937_64(seed);

  // step 1: generate plates and heightmap (see
  // https://www.youtube.com/watch?v=x_Tn66PvTn4)
  statusReport = GenerationStatus::PLATES;

  // step 1.1: generate plates
  uniform_real_distribution<float> zeroToOne(0.f, 1.f);
  uniform_real_distribution<float> negOneToOne(-1.f, 1.f);

  while (plates.size() < config.numMajorPlates + config.numMinorPlates) {
    bool generatingMajorPlate = plates.size() < config.numMajorPlates;

    // step 1.1.1.1: generate random point
    vec3 attempt = randomPointInSphere(rng);

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
  Perlin platePerlin = Perlin(rng, config.maxFeatureSize, config.minFeatureSize,
                              config.octaveDecay);
  data->forEach([this, &platePerlin, &config](TerrainData &data) {
    data.plate = &*min_element(
        plates.begin(), plates.end(),
        [this, &platePerlin, &data, &config](Plate const &a, Plate const &b) {
          size_t aIdx =
              find_if(plates.begin(), plates.end(),
                      [&a](Plate const &compare) { return &compare == &a; }) -
              plates.begin();
          size_t bIdx =
              find_if(plates.begin(), plates.end(),
                      [&b](Plate const &compare) { return &compare == &b; }) -
              plates.begin();
          float bias =
              platePerlin(data.centroid) * config.maxPlatePerlinRoughness;
          if (aIdx < bIdx) {
            return a.weightedDistanceTo(data, bias) <
                   b.weightedDistanceTo(data, -bias);
          } else {
            return a.weightedDistanceTo(data, -bias) <
                   b.weightedDistanceTo(data, bias);
          }
        });
  });

  // step 1.2: set plate motion

  // step 1.2.1: generate per-plate motion
  for_each(plates.begin(), plates.end(),
           [&zeroToOne, &negOneToOne, &rng, &config](Plate &plate) {
             plate.rotationAboutCore =
                 zeroToOne(rng) * randomPointInSphere(rng) / config.radius;
             plate.rotationAboutPlate =
                 negOneToOne(rng) / (config.radius * config.minMajorPlateAngle);
           });

  // step 1.2.2: calculate per-tile motion
  // note - rotation about center of plate accounts for a third of movement,
  // rotation about core of planet accounts for the remaining two thirds
  data->forEach([&config](TerrainData &data) {
    vec3 movementFromCore = cross(data.plate->rotationAboutCore, data.centroid);
    vec3 movementFromRotation =

        cross(data.plate->rotationAboutPlate * data.plate->center.normal,
              data.centroid - data.plate->center.centroid);
    data.plateMovement =
        config.coreMovementFraction * movementFromCore +
        (1.f - config.coreMovementFraction) * movementFromRotation;
  });

  // step 1.3: generate heightmap

  // step 1.3.1: base elevation
  // Rules:
  //
  // apply base height, then apply plate-boundary-specific changes, finally, add
  // layer of fractal perlin noise (with max feature size 3000km, min feature
  // size 50km), ranging from -2km to 4km
  //
  // default continental = 500m elevation
  //
  // default oceanic = -5km elevation
  //
  // Oceanic-continental convergent boundary = max scale of 1000 km, sinusoidal
  // elevation up to 4km high on continental side
  //
  // Oceanic-oceanic convergent boundary = max scale of 500 km, sinusoidal
  // elevation up to 1km high on one plate's side (lower-numbered plate in
  // plates list)
  //
  // Continental-continental convergent boundary = max scale of 1500 km,
  // sinusoidal elevation up to 10km centered on boundary
  //
  // General divergent boundary = inner max scale of 100km, flat
  // elevation of -500m around boundary; outer max scale of 750km, sinusoidal
  // elevation up to 2.5km high around boundary
  //
  // Continental shelf = for 50-100km on oceanic edges of continental plates,
  // use -100m elevation

  Perlin elevationPerlin = Perlin(rng, config.maxFeatureSize,
                                  config.minFeatureSize, config.octaveDecay);
  Perlin featurePerlin = Perlin(rng, config.maxFeatureSize,
                                config.minFeatureSize, config.octaveDecay);
  data->forEach([this, &config, &elevationPerlin,
                 &featurePerlin](TerrainData &data) {
    // generate neighbourhood
    // note - doubling the resolution is required to avoid discretization errors
    vector<TerrainData const *> neighbourhood = neighbourhoodOf(
        data,
        std::max({config.oceanicContinentalSize, config.oceanicOceanicSize,
                  config.continentalContinentalSize,
                  config.continentalShelfSize, config.divergentOuterSize}),
        config.radius);
    vector<TerrainData const *> oceanicNeighbourhood;
    copy_if(neighbourhood.begin(), neighbourhood.end(),
            back_inserter(oceanicNeighbourhood),
            [](TerrainData const *neighbour) {
              return !neighbour->plate->continental;
            });
    vector<TerrainData const *> continentalNeighbourhood;
    copy_if(neighbourhood.begin(), neighbourhood.end(),
            back_inserter(continentalNeighbourhood),
            [](TerrainData const *neighbour) {
              return neighbour->plate->continental;
            });

    // Baseline
    data.elevation = data.plate->continental
                         ? config.continentalElevationBaseline
                         : config.oceanicElevationBaseline;

    // Oceanic-continental convergent
    if (data.plate->continental) {
      vector<TerrainData const *> oceanicLocalNeighbourhood;
      copy_if(oceanicNeighbourhood.begin(), oceanicNeighbourhood.end(),
              back_inserter(oceanicLocalNeighbourhood),
              [&config, &data](TerrainData const *neighbour) {
                return config.radius *
                           angleBetween(data.centroid, neighbour->centroid) <
                       config.continentalShelfSize;
              });

      vector<float> oceanicDistances;
      transform(oceanicLocalNeighbourhood.begin(),
                oceanicLocalNeighbourhood.end(),
                back_inserter(oceanicDistances),
                [&config, &data](TerrainData const *neighbour) {
                  return config.radius *
                         angleBetween(data.centroid, neighbour->centroid);
                });

      if (!oceanicDistances.empty()) {
        float shortestOceanicDistance =
            *min_element(oceanicDistances.begin(), oceanicDistances.end());

        vector<float> alignments;
        transform(oceanicLocalNeighbourhood.begin(),
                  oceanicLocalNeighbourhood.end(), back_inserter(alignments),
                  [&data](TerrainData const *neighbour) {
                    return dot(neighbour->plateMovement - data.plateMovement,
                               normalize(data.centroid - neighbour->centroid));
                  });
        float averageAlignment =
            accumulate(alignments.begin(), alignments.end(), 0.f) /
            static_cast<float>(alignments.size());

        if (averageAlignment > 0.f) {
          data.elevation += averageAlignment *
                            config.oceanicContinentalElevation *
                            sin(shortestOceanicDistance * M_PIf /
                                config.oceanicContinentalSize);
        }
      }
    }

    // Oceanic-oceanic convergent
    if (!data.plate->continental) {
      unordered_map<Plate const *, vector<TerrainData const *>>
          oceanicLocalNeighbourhoodForeigners;
      for_each(
          oceanicNeighbourhood.begin(), oceanicNeighbourhood.end(),
          [&config, &data,
           &oceanicLocalNeighbourhoodForeigners](TerrainData const *neighbour) {
            if (config.radius *
                        angleBetween(data.centroid, neighbour->centroid) <
                    config.oceanicOceanicSize &&
                neighbour->plate != data.plate) {
              oceanicLocalNeighbourhoodForeigners[neighbour->plate].push_back(
                  neighbour);
            }
          });

      unordered_map<Plate const *, vector<float>> oceanicDistances;
      for_each(
          oceanicLocalNeighbourhoodForeigners.begin(),
          oceanicLocalNeighbourhoodForeigners.end(),
          [&config, &data, &oceanicDistances](
              pair<Plate const *, vector<TerrainData const *>> const &entry) {
            transform(entry.second.begin(), entry.second.end(),
                      back_inserter(oceanicDistances[entry.first]),
                      [&config, &data](TerrainData const *neighbour) {
                        return config.radius *
                               angleBetween(data.centroid, neighbour->centroid);
                      });
          });

      unordered_map<Plate const *, size_t> oceanicPlateIndices;
      for_each(
          oceanicLocalNeighbourhoodForeigners.begin(),
          oceanicLocalNeighbourhoodForeigners.end(),
          [this, &oceanicPlateIndices](
              pair<Plate const *, vector<TerrainData const *>> const &entry) {
            oceanicPlateIndices[entry.first] =
                find_if(plates.begin(), plates.end(),
                        [&entry](Plate const &plate) {
                          return &plate == entry.first;
                        }) -
                plates.begin();
          });
      size_t currPlateIndex = find_if(plates.begin(), plates.end(),
                                      [&data](Plate const &plate) {
                                        return &plate == data.plate;
                                      }) -
                              plates.begin();

      data.elevation += accumulate(
          oceanicPlateIndices.begin(), oceanicPlateIndices.end(), 0.f,
          [&currPlateIndex, &oceanicDistances,
           &oceanicLocalNeighbourhoodForeigners, &data,
           &config](float rsf, pair<Plate const *, size_t> const &entry) {
            Plate const *plate = entry.first;
            size_t index = entry.second;

            if (currPlateIndex > index) {
              // skip if this is higher numbered
              return rsf;
            }

            float shortestOceanicDistance = *min_element(
                oceanicDistances[plate].begin(), oceanicDistances[plate].end());

            vector<float> alignments;
            transform(oceanicLocalNeighbourhoodForeigners[plate].begin(),
                      oceanicLocalNeighbourhoodForeigners[plate].end(),
                      back_inserter(alignments),
                      [&data](TerrainData const *neighbour) {
                        return dot(
                            neighbour->plateMovement - data.plateMovement,
                            normalize(data.centroid - neighbour->centroid));
                      });
            float averageAlignment =
                accumulate(alignments.begin(), alignments.end(), 0.f) /
                static_cast<float>(alignments.size());

            if (averageAlignment > 0.f) {
              return rsf + averageAlignment * config.oceanicOceanicElevation *
                               sin(shortestOceanicDistance * M_PIf /
                                   config.oceanicOceanicSize);
            } else {
              return rsf;
            }
          });
    }

    // Continental-continental convergent
    if (data.plate->continental) {
      unordered_map<Plate const *, vector<TerrainData const *>>
          continentalLocalNeighbourhoodForeigners;
      for_each(continentalNeighbourhood.begin(), continentalNeighbourhood.end(),
               [&config, &data, &continentalLocalNeighbourhoodForeigners](
                   TerrainData const *neighbour) {
                 if (config.radius *
                             angleBetween(data.centroid, neighbour->centroid) <
                         config.continentalContinentalSize &&
                     neighbour->plate != data.plate) {
                   continentalLocalNeighbourhoodForeigners[neighbour->plate]
                       .push_back(neighbour);
                 }
               });

      unordered_map<Plate const *, vector<float>> continentalDistances;
      for_each(
          continentalLocalNeighbourhoodForeigners.begin(),
          continentalLocalNeighbourhoodForeigners.end(),
          [&config, &data, &continentalDistances](
              pair<Plate const *, vector<TerrainData const *>> const &entry) {
            transform(entry.second.begin(), entry.second.end(),
                      back_inserter(continentalDistances[entry.first]),
                      [&config, &data](TerrainData const *neighbour) {
                        return config.radius *
                               angleBetween(data.centroid, neighbour->centroid);
                      });
          });

      unordered_map<Plate const *, size_t> continentalPlateIndices;
      for_each(
          continentalLocalNeighbourhoodForeigners.begin(),
          continentalLocalNeighbourhoodForeigners.end(),
          [this, &continentalPlateIndices](
              pair<Plate const *, vector<TerrainData const *>> const &entry) {
            continentalPlateIndices[entry.first] =
                find_if(plates.begin(), plates.end(),
                        [&entry](Plate const &plate) {
                          return &plate == entry.first;
                        }) -
                plates.begin();
          });
      size_t currPlateIndex = find_if(plates.begin(), plates.end(),
                                      [&data](Plate const &plate) {
                                        return &plate == data.plate;
                                      }) -
                              plates.begin();

      data.elevation += accumulate(
          continentalPlateIndices.begin(), continentalPlateIndices.end(), 0.f,
          [&currPlateIndex, &continentalDistances,
           &continentalLocalNeighbourhoodForeigners, &data,
           &config](float rsf, pair<Plate const *, size_t> const &entry) {
            Plate const *plate = entry.first;
            size_t index = entry.second;

            if (currPlateIndex > index) {
              // skip if this is higher numbered
              return rsf;
            }

            float shortestContinentalDistance =
                *min_element(continentalDistances[plate].begin(),
                             continentalDistances[plate].end());

            vector<float> alignments;
            transform(continentalLocalNeighbourhoodForeigners[plate].begin(),
                      continentalLocalNeighbourhoodForeigners[plate].end(),
                      back_inserter(alignments),
                      [&data](TerrainData const *neighbour) {
                        return dot(
                            neighbour->plateMovement - data.plateMovement,
                            normalize(data.centroid - neighbour->centroid));
                      });
            float averageAlignment =
                accumulate(alignments.begin(), alignments.end(), 0.f) /
                static_cast<float>(alignments.size());

            if (averageAlignment > 0.f) {
              return rsf + averageAlignment *
                               config.continentalContinentalElevation *
                               sin(shortestContinentalDistance * M_PIf /
                                   config.continentalContinentalSize);
            } else {
              return rsf;
            }
          });
    }

    // Divergent
    {
      vector<TerrainData const *> localNeighbourhoodForeigners;
      copy_if(neighbourhood.begin(), neighbourhood.end(),
              back_inserter(localNeighbourhoodForeigners),
              [&config, &data](TerrainData const *neighbour) {
                return config.radius * angleBetween(data.centroid,
                                                    neighbour->centroid) <
                           config.continentalContinentalSize &&
                       neighbour->plate != data.plate;
              });

      vector<float> distances;
      transform(localNeighbourhoodForeigners.begin(),
                localNeighbourhoodForeigners.end(), back_inserter(distances),
                [&config, &data](TerrainData const *neighbour) {
                  return config.radius *
                         angleBetween(data.centroid, neighbour->centroid);
                });

      if (!distances.empty()) {
        float shortestDistance =
            *min_element(distances.begin(), distances.end());

        vector<float> alignments;
        transform(localNeighbourhoodForeigners.begin(),
                  localNeighbourhoodForeigners.end(), back_inserter(alignments),
                  [&data](TerrainData const *neighbour) {
                    return dot(neighbour->plateMovement - data.plateMovement,
                               normalize(data.centroid - neighbour->centroid));
                  });
        float averageAlignment =
            accumulate(alignments.begin(), alignments.end(), 0.f) /
            static_cast<float>(alignments.size());

        if (averageAlignment < 0.f) {
          if (shortestDistance <
              -averageAlignment * config.divergentInnerSize) {
            data.elevation +=
                -averageAlignment * config.divergentInnerElevation;
          } else {
            data.elevation +=
                -averageAlignment * config.divergentOuterElevation *
                cos((shortestDistance +
                     averageAlignment * config.divergentInnerSize) *
                    M_PI_2f /
                    (config.divergentOuterSize +
                     averageAlignment * config.divergentInnerSize));
          }
        }
      }
    }

    // Continental shelf
    if (data.plate->continental) {
      vector<TerrainData const *> oceanicLocalNeighbourhood;
      copy_if(oceanicNeighbourhood.begin(), oceanicNeighbourhood.end(),
              back_inserter(oceanicLocalNeighbourhood),
              [&config, &data](TerrainData const *neighbour) {
                return config.radius *
                           angleBetween(data.centroid, neighbour->centroid) <
                       config.continentalShelfSize;
              });

      vector<float> oceanicDistances;
      transform(oceanicLocalNeighbourhood.begin(),
                oceanicLocalNeighbourhood.end(),
                back_inserter(oceanicDistances),
                [&config, &data](TerrainData const *neighbour) {
                  return config.radius *
                         angleBetween(data.centroid, neighbour->centroid);
                });

      if (!oceanicDistances.empty()) {
        float shortestOceanicDistance =
            *min_element(oceanicDistances.begin(), oceanicDistances.end());

        // is in oceanic elevation transition between 0 and shelfSize / 4
        // is continental shelf between shelfSize / 4 and 3 * shelfSize / 4
        // is in continental elevation transition between 3 * shelfSize / 4
        // and shelfSize
        float shelfStartDistance = 0.25f * config.continentalShelfSize;
        float shelfEndDistance = 0.75f * config.continentalShelfSize;
        if (shortestOceanicDistance < shelfStartDistance) {
          // transition between oceanic at 0 and continental shelf at 0.25
          data.elevation += lerp(config.oceanicElevationBaseline -
                                     config.continentalElevationBaseline,
                                 config.continentalShelfElevation -
                                     config.continentalElevationBaseline,
                                 shortestOceanicDistance / shelfStartDistance);
        } else if (shelfStartDistance <= shortestOceanicDistance &&
                   shortestOceanicDistance <= shelfEndDistance) {
          // continental shelf
          data.elevation += config.continentalShelfElevation -
                            config.continentalElevationBaseline;
        } else if (shortestOceanicDistance > shelfEndDistance) {
          // transition away from continental shelf
          float areaRange = config.continentalShelfSize - shelfEndDistance;
          data.elevation +=
              lerp(config.continentalShelfElevation -
                       config.continentalElevationBaseline,
                   config.continentalElevationBaseline -
                       config.continentalElevationBaseline,
                   (shortestOceanicDistance - shelfEndDistance) / areaRange);
        }
      }
    }

    // noise
    float noise = elevationPerlin(data.centroid);
    if (noise <= 0) {
      data.elevation += -noise * config.minNoiseElevation;
    } else {
      data.elevation += noise * config.maxNoiseElevation;
    }
  });

  // step 1.4: place volcanic hotspot islands

  size_t hotspotCount = uniform_int_distribution<size_t>(
      config.minHotspotIslandArcs, config.maxHotspotIslandArcs)(rng);
  vector<TerrainData const *> hotspotRoots;
  while (hotspotRoots.size() < hotspotCount) {
    // select a random point
    TerrainData const &attempt = operator[](randomPointInSphere(rng));

    // that is oceanic
    if (attempt.plate->continental) {
      continue;
    }

    hotspotRoots.push_back(&attempt);
  }

  struct Hotspot {
    TerrainData const *center;
    float elevation;
    float width;
  };
  vector<Hotspot> hotspots;
  for_each(
      hotspotRoots.begin(), hotspotRoots.end(),
      [this, &hotspots, &config, &rng](TerrainData const *root) {
        float arcLength = uniform_real_distribution<float>(
            config.minHotspotIslandArcLength,
            config.maxHotspotIslandArcLength)(rng);
        uniform_real_distribution<float> spacingDistribution =
            uniform_real_distribution<float>(config.minHotspotIslandSpacing,
                                             config.maxHotspotIslandSpacing);
        uniform_real_distribution<float> elevationDistribution =
            uniform_real_distribution<float>(config.minHotspotIslandElevation,
                                             config.maxHotspotIslandElevation);
        uniform_real_distribution<float> sizeDistribution =
            uniform_real_distribution<float>(config.minHotspotIslandSize,
                                             config.maxHotspotIslandSize);

        // magnitude = sin(1/radius)
        vec3 axis = cross(normalize(root->centroid + root->plateMovement),
                          normalize(root->centroid));

        float currentOffset = 0.f;
        size_t idx = 0;
        do {
          vec3 location =
              cross(root->centroid, currentOffset * axis) + root->centroid;
          hotspots.push_back({.center = &operator[](location),
                              .elevation = elevationDistribution(rng) /
                                           pow(config.hotspotIslandSizeDecay,
                                               static_cast<float>(idx)),
                              .width = sizeDistribution(rng)});

          currentOffset += spacingDistribution(rng);
          ++idx;
        } while (currentOffset < arcLength);
      });
  data->forEach([&hotspots, &config](TerrainData &data) {
    vector<Hotspot> inRange;
    copy_if(hotspots.begin(), hotspots.end(), back_inserter(inRange),
            [&data, &config](Hotspot const &hotspot) {
              return config.radius *
                         angleBetween(hotspot.center->centroid, data.centroid) <
                     hotspot.width;
            });

    if (inRange.empty()) {
      return;
    }

    vector<float> elevationBonuses;
    transform(inRange.begin(), inRange.end(), back_inserter(elevationBonuses),
              [&data, &config](Hotspot const &hotspot) {
                float distance =
                    config.radius *
                    angleBetween(hotspot.center->centroid, data.centroid);
                return hotspot.elevation *
                       cos(distance * M_PI_2f / hotspot.width);
              });

    data.elevation +=
        *max_element(elevationBonuses.begin(), elevationBonuses.end());
  });

  // step 2: calculate prevailing winds (see
  // https://www.youtube.com/watch?v=LifRswfCxFU)
  statusReport = GenerationStatus::WINDS;

  // step 2.1: generate trade winds

  // TODO

  // step 2.2: generate polar cells

  // TODO

  // step 3.3: generate ferrel cell

  // TODO

  // step 3: calculate currents (see
  // https://www.youtube.com/watch?v=n_E9UShtyY8)
  statusReport = GenerationStatus::WINDS;

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
  statusReport = GenerationStatus::BIOMES;

  // step 4.1: preparatory calculations

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
  statusReport = GenerationStatus::RIVERS;

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
  statusReport = GenerationStatus::RESOURCES;

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
  statusReport = GenerationStatus::HISTORY;

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
TerrainData const &EarthlikePlanet::operator[](
    vec2 const &location) const noexcept {
  return operator[](sphericalToCartesian(location));
}
TerrainData &EarthlikePlanet::operator[](vec3 const &location) noexcept {
  return (*data)[location];
}
TerrainData const &EarthlikePlanet::operator[](
    vec3 const &location) const noexcept {
  return (*data)[location];
}
vector<TerrainData const *> EarthlikePlanet::neighbourhoodOf(
    TerrainData const &center, float searchRadius,
    float planetRadius) const noexcept {
  unordered_set<TerrainData const *> visited;
  vector<TerrainData const *> toVisit;
  toVisit.push_back(&center);
  while (!toVisit.empty()) {
    TerrainData const *curr = toVisit.back();
    toVisit.pop_back();

    if (planetRadius * angleBetween(center.centroid, curr->centroid) >
        searchRadius) {
      // current thing is outside the search radius; ignore this
      continue;
    }

    if (!visited.insert(curr).second) {
      // try to insert; if already present, continue
      continue;
    }

    // is in the search radius and is also not visited before
    copy(curr->neighbours.begin(), curr->neighbours.end(),
         back_inserter(toVisit));
  }
  vector<TerrainData const *> retval;
  copy(visited.begin(), visited.end(), back_inserter(retval));
  return retval;
}
}  // namespace planetgen
