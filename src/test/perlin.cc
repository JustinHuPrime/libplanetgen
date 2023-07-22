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

#include <stb_image_write.h>

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <execution>
#include <glm/glm.hpp>
#include <memory>
#include <random>

#include "imgOutput.h"

using namespace planetgen;
using namespace std;
using namespace glm;

constexpr size_t HEIGHT = 4096;
constexpr size_t WIDTH = HEIGHT;

TEST_CASE("Generate simple Perlin noise", "[.long]") {
  mt19937_64 rng = mt19937_64(0);

  PerlinOctave octave = PerlinOctave(rng, 128.f);

  unique_ptr<Pixel[]> bitmap = make_unique<Pixel[]>(WIDTH * HEIGHT);
  for_each(
      execution::par_unseq, CountingIterator(0), CountingIterator(HEIGHT),
      [&](size_t y) {
        for_each(CountingIterator(0), CountingIterator(WIDTH), [&](size_t x) {
          vec3 location =
              vec3{static_cast<float>(x), static_cast<float>(y), 0.f};
          float noise = octave(location);
          uint8_t value = static_cast<uint8_t>(floor(noise * 128.f + 128.f));
          bitmap[y * WIDTH + x] = Pixel{value, value, value, 255};
        });
      });

  stbi_write_png("simple-perlin.png", WIDTH, HEIGHT, 4, bitmap.get(),
                 WIDTH * sizeof(Pixel));
}

TEST_CASE("Generate fractal Perlin noise", "[.long]") {
  mt19937_64 rng = mt19937_64(0);

  Perlin perlin = Perlin(rng, 512.f, 128.f, 1.0f);

  unique_ptr<Pixel[]> bitmap = make_unique<Pixel[]>(WIDTH * HEIGHT);
  for_each(
      execution::par_unseq, CountingIterator(0), CountingIterator(HEIGHT),
      [&](size_t y) {
        for_each(CountingIterator(0), CountingIterator(WIDTH), [&](size_t x) {
          vec3 location =
              vec3{static_cast<float>(x), static_cast<float>(y), 0.f};
          float noise = perlin(location);
          uint8_t value = static_cast<uint8_t>(floor(noise * 128.f + 128.f));
          bitmap[y * WIDTH + x] = Pixel{value, value, value, 255};
        });
      });

  stbi_write_png("fractal-perlin.png", WIDTH, HEIGHT, 4, bitmap.get(),
                 WIDTH * sizeof(Pixel));
}
