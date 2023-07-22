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

#include "perlin.h"

#include <algorithm>

using namespace std;
using namespace glm;

namespace planetgen {
// One Perlin octave call is equivalent to a call to the reference
// implementation: https://mrl.cs.nyu.edu/~perlin/noise/
PerlinOctave::PerlinOctave(mt19937_64 &rng, float featureSize) noexcept
    : featureSize(featureSize) {
  for (size_t idx = 0; idx < hasher.size() / 2; ++idx) {
    hasher[idx] = idx;
  }
  shuffle(hasher.begin(), hasher.begin() + hasher.size() / 2, rng);
  copy(hasher.begin(), hasher.begin() + hasher.size() / 2,
       hasher.begin() + hasher.size() / 2);
}
float PerlinOctave::operator()(vec3 const &location) const noexcept {
  // scale to feature size
  vec3 scaled = location / featureSize;
  vec3 floored = floor(scaled);

  // find unit cube containing point
  uint32_t xInt = static_cast<uint32_t>(static_cast<int32_t>(floored.x) & 255);
  uint32_t yInt = static_cast<uint32_t>(static_cast<int32_t>(floored.y) & 255);
  uint32_t zInt = static_cast<uint32_t>(static_cast<int32_t>(floored.z) & 255);

  // find relative offset within cube
  vec3 offset = scaled - floored;

  // compute fade curves
  float xFade = fade(offset.x);
  float yFade = fade(offset.y);
  float zFade = fade(offset.z);

  // compute hashes
  int a = hasher[xInt] + yInt;
  int aa = hasher[a] + zInt;
  int ab = hasher[a + 1] + zInt;
  int b = hasher[xInt + 1] + yInt;
  int ba = hasher[b] + zInt;
  int bb = hasher[b + 1] + zInt;

  // add blended results from cube corners

  return lerp(
             lerp(lerp(grad(hasher[aa], offset.x, offset.y, offset.z),
                       grad(hasher[ba], offset.x - 1.f, offset.y, offset.z),
                       xFade),
                  lerp(grad(hasher[ab], offset.x, offset.y - 1.f, offset.z),
                       grad(hasher[bb], offset.x - 1.f, offset.y - 1.f,
                            offset.z),
                       xFade),
                  yFade),
             lerp(lerp(grad(hasher[aa + 1], offset.x, offset.y, offset.z - 1.f),
                       grad(hasher[ba + 1], offset.x - 1.f, offset.y,
                            offset.z - 1.f),
                       xFade),
                  lerp(grad(hasher[ab + 1], offset.x, offset.y - 1.f,
                            offset.z - 1.f),
                       grad(hasher[bb + 1], offset.x - 1.f, offset.y - 1.f,
                            offset.z - 1.f),
                       xFade),
                  yFade),
             zFade) /
         2.f;
}
constexpr float PerlinOctave::fade(float t) noexcept {
  return t * t * t * (t * (t * 6.f - 15.f) + 10.f);
}
constexpr float PerlinOctave::grad(uint8_t hash, float x, float y,
                                   float z) noexcept {
  // convert low 4 bits of hash code into 12 gradient directions
  int h = hash & 15;
  float u = h < 8 ? x : y;
  float v = h < 4 ? y : h == 12 || h == 14 ? x : z;
  return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

Perlin::Perlin(mt19937_64 &rng, float maxFeatureSize, float minFeatureSize,
               float octaveDecayFactor) noexcept
    : octaves(), amplitudeSum(0.f) {
  assert(maxFeatureSize > minFeatureSize);
  assert(octaveDecayFactor != 0.f);

  float featureSize = maxFeatureSize;
  float amplitude = 1.f;
  while (featureSize > minFeatureSize) {
    octaves.emplace_back(PerlinOctave(rng, featureSize), amplitude);
    amplitudeSum += amplitude;
    featureSize /= 2;
    amplitude /= octaveDecayFactor;
  }
}
float Perlin::operator()(vec3 const &location) const noexcept {
  float unscaled = std::accumulate(
      octaves.begin(), octaves.end(), 0.f,
      [&location](float rsf, pair<PerlinOctave, float> const &octave) {
        return rsf + (octave.second * octave.first(location));
      });
  return unscaled / amplitudeSum;
}
}  // namespace planetgen
