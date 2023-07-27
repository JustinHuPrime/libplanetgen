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

#ifndef PLANETGEN_PERLIN_H_
#define PLANETGEN_PERLIN_H_

#include <array>
#include <glm/glm.hpp>
#include <random>
#include <utility>
#include <vector>

namespace planetgen {
class PerlinOctave {
 public:
  PerlinOctave(std::mt19937_64 &rng, float featureSize) noexcept;

  PerlinOctave(PerlinOctave const &) noexcept = default;
  PerlinOctave(PerlinOctave &&) noexcept = default;

  ~PerlinOctave() noexcept = default;

  PerlinOctave &operator=(PerlinOctave const &) noexcept = default;
  PerlinOctave &operator=(PerlinOctave &&) noexcept = default;

  /**
   * Generates value between -1 and 1
   */
  float operator()(glm::vec3 const &) const noexcept;

 private:
  float featureSize;
  std::array<uint8_t, 512> hasher;

  static constexpr float fade(float t) noexcept;
  static constexpr float grad(uint8_t hash, float x, float y, float z) noexcept;
};
class Perlin {
 public:
  Perlin(std::mt19937_64 &rng, float maxFeatureSize, float minFeatureSize,
         float octaveDecayFactor) noexcept;

  Perlin(Perlin const &) noexcept = default;
  Perlin(Perlin &&) noexcept = default;

  ~Perlin() noexcept = default;

  Perlin &operator=(Perlin const &) noexcept = default;
  Perlin &operator=(Perlin &&) noexcept = default;

  /**
   * Generates value between -1 and 1
   */
  float operator()(glm::vec3 const &) const noexcept;

 private:
  std::vector<std::pair<PerlinOctave, float>> octaves;
  float amplitudeSum;
};
}  // namespace planetgen

#endif  // PLANETGEN_PERLIN_H_
