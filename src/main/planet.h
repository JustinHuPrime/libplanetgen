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
#include <glm/glm.hpp>
#include <memory>
#include <random>

namespace planetgen {
struct TerrainData final {
  TerrainData() noexcept = default;
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

  virtual TerrainData &operator[](glm::vec2 const &) noexcept = 0;

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

  virtual bool contains(glm::vec2 const &) noexcept;
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

  TerrainData &operator[](glm::vec2 const &) noexcept override;

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

  TerrainData &operator[](glm::vec2 const &) noexcept override;

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

  TerrainData &operator[](glm::vec2 const &) noexcept override;

 private:
  std::array<std::unique_ptr<TriangleTerrainTreeNode>, 20> children;
};

enum class GenerationStatus {
  TRIANGULATING,
  DONE,
};

class EarthlikePlanet final {
 public:
  /**
   * Create an earthlike planet's terrain
   *
   * Note that the planet in question is modelled as a sphere, not a spheroid;
   * this may change in later versions (and a second constructor will be
   * introduced taking the equatorial and polar radii)
   *
   * @param seed random number generator seed
   * @param radius planetary radius, in meters (suggested limits are 0.5 -
   * 2 earth radii, or 3.1855e6 to 12.742e6 meters - for reference, Earth has a
   * radius of 6.371e6 meters; exceeding these limits may lead to unrealistic
   * results)
   * @param resolution maximum side length of a face, in meters (suggested value
   * of 50e3 meters) - note; must be significantly less than radius
   */
  EarthlikePlanet(uint64_t seed, float radius, float resolution,
                  std::atomic<GenerationStatus> &statusReport) noexcept;

  EarthlikePlanet(EarthlikePlanet const &) noexcept = delete;
  EarthlikePlanet(EarthlikePlanet &&) noexcept = default;

  ~EarthlikePlanet() noexcept = default;

  EarthlikePlanet &operator=(EarthlikePlanet const &) noexcept = delete;
  EarthlikePlanet &operator=(EarthlikePlanet &&) noexcept = default;

  TerrainData &operator[](glm::vec2 const &) noexcept;

 private:
  std::mt19937_64 rngEngine;

  std::unique_ptr<TerrainTreeNode> data;
  float radius;
  float resolution;
};
}  // namespace planetgen

#endif  // PLANETGEN_PLANET_H_
