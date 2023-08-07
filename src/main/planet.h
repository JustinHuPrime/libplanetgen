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

#ifndef PLANETGEN_PLANET_H_
#define PLANETGEN_PLANET_H_

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <glm/glm.hpp>
#include <memory>
#include <random>
#include <unordered_set>
#include <vector>

namespace planetgen {
struct TerrainData;
struct Plate final {
  bool major;
  bool continental;

  TerrainData const &center;

  /**
   * weight applied to distance in 20 compass directions around a circle
   * higher weight = larger in that direction
   */
  std::array<float, 20> directionalWeights;

  /**
   * rotation vector about core of planet
   */
  glm::vec3 rotationAboutCore;
  /**
   * multiply this coefficient with normal to get rotation vector about center
   * of plate
   */
  float rotationAboutPlate;

  /**
   * @param major is this a major plate
   * @param continental is this a continental plate
   * @param center which terrain node is this centred at
   * @param rng rng to use to randomly generate directional weights
   * @param baseWeight base distance weight in all directions
   * @param maxWeightDeviation maximum deviation allowed in a weight, as a
   * proportion of the base weight
   */
  Plate(bool major, bool continental, TerrainData const &center,
        std::mt19937_64 &rng, float baseWeight,
        float maxWeightDeviation) noexcept;
  Plate(Plate const &) noexcept = default;
  Plate(Plate &&) noexcept = default;

  ~Plate() noexcept = default;

  Plate &operator=(Plate const &) noexcept = default;
  Plate &operator=(Plate &&) noexcept = default;

  float weightedDistanceTo(TerrainData const &point, float bias) const noexcept;

  /**
   * Get the angle of a point around the plate where north = 0, east = pi/2,
   * south = pi, west = 3pi/2
   */
  float angleAround(TerrainData const &point) const noexcept;
};

struct TerrainData final {
  std::array<glm::vec3, 3> const &vertices;
  glm::vec3 centroid;
  glm::vec3 normal;

  std::array<TerrainData const *, 3> neighbours;

  Plate const *plate;
  glm::vec3 plateMovement;

  float elevation;

  TerrainData(std::array<glm::vec3, 3> const &vertices) noexcept;
  TerrainData(TerrainData const &) noexcept = default;
  TerrainData(TerrainData &&) noexcept = default;

  ~TerrainData() noexcept = default;

  TerrainData &operator=(TerrainData const &) noexcept = default;
  TerrainData &operator=(TerrainData &&) noexcept = default;
};

class TerrainTreeNode {
 public:
  TerrainTreeNode() noexcept = default;
  TerrainTreeNode(TerrainTreeNode const &) noexcept = default;
  TerrainTreeNode(TerrainTreeNode &&) noexcept = default;

  virtual ~TerrainTreeNode() noexcept = default;

  TerrainTreeNode &operator=(TerrainTreeNode const &) noexcept = default;
  TerrainTreeNode &operator=(TerrainTreeNode &&) noexcept = default;

  virtual TerrainData &operator[](glm::vec3 const &) noexcept = 0;

  virtual void forEach(std::function<void(TerrainData &)> const &) = 0;

 private:
};
class TriangleTerrainTreeNode : public TerrainTreeNode {
 public:
  explicit TriangleTerrainTreeNode(std::array<glm::vec3, 3> const &) noexcept;
  TriangleTerrainTreeNode(TriangleTerrainTreeNode const &) noexcept = default;
  TriangleTerrainTreeNode(TriangleTerrainTreeNode &&) noexcept = default;

  ~TriangleTerrainTreeNode() noexcept = default;

  TriangleTerrainTreeNode &operator=(TriangleTerrainTreeNode const &) noexcept =
      default;
  TriangleTerrainTreeNode &operator=(TriangleTerrainTreeNode &&) noexcept =
      default;

  virtual bool contains(glm::vec3 const &) noexcept;
  virtual void inflate(float radius) noexcept;

  std::array<glm::vec3, 3> const &getVertices() const noexcept;

 protected:
  std::array<glm::vec3, 3> vertices;
};
class QuadTerrainTreeNode final : public TriangleTerrainTreeNode {
 public:
  QuadTerrainTreeNode(std::array<glm::vec3, 3> const &vertices,
                      float resolution) noexcept;
  QuadTerrainTreeNode(QuadTerrainTreeNode const &) noexcept = delete;
  QuadTerrainTreeNode(QuadTerrainTreeNode &&) noexcept = default;

  ~QuadTerrainTreeNode() noexcept override = default;

  QuadTerrainTreeNode &operator=(QuadTerrainTreeNode const &) noexcept = delete;
  QuadTerrainTreeNode &operator=(QuadTerrainTreeNode &&) noexcept = default;

  TerrainData &operator[](glm::vec3 const &) noexcept override;
  void forEach(std::function<void(TerrainData &)> const &) override;

  void inflate(float radius) noexcept override;

 private:
  std::array<std::unique_ptr<TriangleTerrainTreeNode>, 4> children;
};
class LeafTerrainTreeNode final : public TriangleTerrainTreeNode {
 public:
  LeafTerrainTreeNode(std::array<glm::vec3, 3> const &vertices) noexcept;
  LeafTerrainTreeNode(LeafTerrainTreeNode const &) noexcept = delete;
  LeafTerrainTreeNode(LeafTerrainTreeNode &&) noexcept = default;

  ~LeafTerrainTreeNode() noexcept override = default;

  LeafTerrainTreeNode &operator=(LeafTerrainTreeNode const &) noexcept = delete;
  LeafTerrainTreeNode &operator=(LeafTerrainTreeNode &&) noexcept = default;

  TerrainData &operator[](glm::vec3 const &) noexcept override;
  void forEach(std::function<void(TerrainData &)> const &) override;

 private:
  TerrainData data;
};
class IcosahedronTerrainTreeNode final : public TerrainTreeNode {
 public:
  IcosahedronTerrainTreeNode(float radius, float resolution) noexcept;
  IcosahedronTerrainTreeNode(IcosahedronTerrainTreeNode const &) noexcept =
      delete;
  IcosahedronTerrainTreeNode(IcosahedronTerrainTreeNode &&) noexcept = default;

  ~IcosahedronTerrainTreeNode() noexcept override = default;

  IcosahedronTerrainTreeNode &operator=(
      IcosahedronTerrainTreeNode const &) noexcept = delete;
  IcosahedronTerrainTreeNode &operator=(
      IcosahedronTerrainTreeNode &&) noexcept = default;

  TerrainData &operator[](glm::vec3 const &) noexcept override;
  void forEach(std::function<void(TerrainData &)> const &) override;

 private:
  std::array<std::unique_ptr<TriangleTerrainTreeNode>, 20> children;
};

enum class GenerationStatus : uint8_t {
  CONSTRUCTION = 0,
  TRIANGULATING,
  PLATES,
  WINDS,
  CURRENTS,
  BIOMES,
  RIVERS,
  RESOURCES,
  HISTORY,
  DONE,
};

class EarthlikePlanet final {
 public:
  struct Config final {
    /**
     * planetary radius, in meters (suggested limits are 0.5 - 2 earth radii,
     * or 3.1855e6 to 12.742e6 meters - for reference, Earth has a radius
     * of 6.371e6 meters; exceeding these limits may lead to unrealistic
     * results)
     */
    float radius = 6.371e6f;
    /**
     * maximum side length of a face, in meters (suggested value of 50e3 meters)
     * - note; must be significantly less than radius
     */
    float resolution = 50e3f;
    /**
     * number of major plates to generate (suggested limits are 6-10; Earth has
     * 8)
     */
    size_t numMajorPlates = 8;
    /**
     * number of minor plates to generate (suggested limits are 15-25; Earth has
     * 10)
     */
    size_t numMinorPlates = 20;
    /**
     * minimum angular separation between major and any other plate centers
     * (suggest pi/8 radians)
     */
    float minMajorPlateAngle = M_PIf / 8.f;
    /**
     * minimum angular separation between minor plate and any minor plate
     * centers (suggest pi/16)
     */
    float minMinorPlateAngle = M_PIf / 16.f;
    /**
     * base size of major plates, in radians (suggested pi/10 radians)
     */
    float majorPlateSizeBonus = M_PIf / 10.f;
    /**
     * max plate roughness, as fraction of base size (suggested 0.15 to 0.25)
     */
    float maxPlateRoughness = 0.2f;
    /**
     * max plate perlin roughness, in radians (suggested pi/30 radians)
     */
    float maxPlatePerlinRoughness = M_PIf / 30.f;
    /**
     * fraction of plates that are oceanic (suggested 0.4 to 0.6)
     */
    float oceanicFraction = 0.5f;
    /**
     * fraction of plate movement from rotation about core (suggested about 2/3)
     */
    float coreMovementFraction = 2.f / 3.f;
    /**
     * max feature size (for terrain noise generator) (suggested ~ 1/2 radius)
     */
    float maxFeatureSize = 3000e3f;
    /**
     * min feature size (for terrain noise generator) (suggested ~ resolution)
     */
    float minFeatureSize = 50e3f;
    /**
     * octave decay factor (for terrain noise generator)
     */
    float octaveDecay = 1.2f;
    /**
     * lowest bias (for terrain noise generator) (suggested -2km)
     */
    float minNoiseElevation = -2e3f;
    /**
     * highest bias (for terrain noise generator) (suggested 4km)
     */
    float maxNoiseElevation = 4e3f;
    /**
     * continental elevation baseline (suggested 500m)
     */
    float continentalElevationBaseline = 500.f;
    /**
     * oceanic elevation baseline (suggested -4km)
     */
    float oceanicElevationBaseline = -4e3f;
    /**
     * Oceanic-continental convergent width (suggested 1000km)
     */
    float oceanicContinentalSize = 1000e3f;
    /**
     * Oceanic-continental convergent additional elevation (suggested 4km -
     * continentalElevationBaseline)
     */
    float oceanicContinentalElevation = 3500.f;
    /**
     * Oceanic-oceanic convergent width (suggested 500km)
     */
    float oceanicOceanicSize = 500e3f;
    /**
     * Oceanic-oceanic convergent additional elevation (suggested 1km -
     * oceanicElevationBaseline)
     */
    float oceanicOceanicElevation = 5e3f;
    /**
     * Width of the thinning part of a divergent boundary
     */
    float divergentOuterSize = 750e3f;
    /**
     * Width of the rift part of a divergent boundary
     */
    float divergentInnerSize = 100e3f;
    /**
     * Additional elevation of the thinning part of a divergent boundary
     */
    float divergentOuterElevation = 2e3f;
    /**
     * Additional elevation for the rift part of a divergent boundary
     */
    float divergentInnerElevation = 1e3f;
    /**
     * Continental-continental convergent width (suggested 750km)
     */
    float continentalContinentalSize = 750e3f;
    /**
     * Continental-continental convergent additional elevation (suggested 10km -
     * continentalElevationBaseline)
     */
    float continentalContinentalElevation = 9500.f;
    /**
     * continental shelf size (suggested 1000km)
     */
    float continentalShelfSize = 1000e3f;
    /**
     * continental shelf set elevation (suggested -100m)
     */
    float continentalShelfElevation = -100.f;
    /**
     * minimum number of hotspot island arcs to generate
     */
    size_t minHotspotIslandArcs = 10;
    /**
     * maximum number of hotspot island arcs to generate
     */
    size_t maxHotspotIslandArcs = 20;
    /**
     * minimum length of hotspot island arcs
     */
    float minHotspotIslandArcLength = 2000e3f;
    /**
     * maximum length of hotspot island arcs
     */
    float maxHotspotIslandArcLength = 4000e3f;
    /**
     * minimum spacing between hotspot islands
     */
    float minHotspotIslandSpacing = 150e3f;
    /**
     * maximum spacing between hotspot islands
     */
    float maxHotspotIslandSpacing = 300e3f;
    /**
     * minimum size of hotspot islands
     */
    float minHotspotIslandSize = 25e3f;
    /**
     * maximum size of hotspot islands
     */
    float maxHotspotIslandSize = 150e3f;
    /**
     * minimum additional elevation of hotspot islands
     */
    float minHotspotIslandElevation = 2e3f;
    /**
     * maximum additional elevation of hotspot islands
     */
    float maxHotspotIslandElevation = 8e3f;
    /**
     * decay rate for successive hotspot island sizes
     */
    float hotspotIslandSizeDecay = 1.5f;
  };

  /**
   * Create an earthlike planet's terrain
   *
   * Note that the planet in question is modelled as a sphere, not a spheroid;
   * this may change in later versions (and a second constructor will be
   * introduced taking the equatorial and polar radii)
   *
   * @param statusReport output reference that will be updated with status
   * changes as construction progresses - this constructor will take a while to
   * run
   * @param seed random number generator seed
   */
  EarthlikePlanet(std::atomic<GenerationStatus> &statusReport, uint64_t seed,
                  Config const &config) noexcept;

  EarthlikePlanet(EarthlikePlanet const &) noexcept = delete;
  EarthlikePlanet(EarthlikePlanet &&) noexcept = default;

  ~EarthlikePlanet() noexcept = default;

  EarthlikePlanet &operator=(EarthlikePlanet const &) noexcept = delete;
  EarthlikePlanet &operator=(EarthlikePlanet &&) noexcept = default;

  TerrainData &operator[](glm::vec2 const &) noexcept;
  TerrainData const &operator[](glm::vec2 const &) const noexcept;
  TerrainData &operator[](glm::vec3 const &) noexcept;
  TerrainData const &operator[](glm::vec3 const &) const noexcept;

  std::vector<Plate> plates;

 private:
  std::unique_ptr<IcosahedronTerrainTreeNode> data;

  std::vector<TerrainData const *> neighbourhoodOf(
      TerrainData const &center, float searchRadius,
      float planetRadius) const noexcept;
};
}  // namespace planetgen

#endif  // PLANETGEN_PLANET_H_
