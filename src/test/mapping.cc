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

#include <stb_image_write.h>

#include <array>
#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <glm/glm.hpp>
#include <memory>
#include <stdexcept>

#include "planet.h"

using namespace planetgen;
using namespace std;
using namespace glm;

struct Pixel {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t a;
};

constexpr size_t HEIGHT = 2048;
constexpr size_t WIDTH = HEIGHT * 2;

TEST_CASE("Check for no-intersection locations", "[.mapping]") {
  atomic<GenerationStatus> status;
  EarthlikePlanet p = EarthlikePlanet(0, 6.371e6f, 50e3f, status);

  unique_ptr<Pixel[]> bitmap = make_unique<Pixel[]>(WIDTH * HEIGHT);

  for (size_t latIdx = 0; latIdx < HEIGHT; ++latIdx) {
    for (size_t lonIdx = 0; lonIdx < WIDTH; ++lonIdx) {
      vec2 location = {
          lerp(M_PI_2f, -M_PI_2f,
               static_cast<float>(latIdx) / static_cast<float>(HEIGHT - 1)),
          lerp(0, 2 * M_PIf,
               static_cast<float>(lonIdx) / static_cast<float>(WIDTH - 1))};
      try {
        p[location];
        bitmap[latIdx * WIDTH + lonIdx] = Pixel{0, 0, 0, 255};
      } catch (runtime_error const &e) {
        if (e.what() == "icosa"s) {
          bitmap[latIdx * WIDTH + lonIdx] = Pixel{255, 0, 255, 255};
        } else {
          bitmap[latIdx * WIDTH + lonIdx] = Pixel{0, 0, 0, 255};
        }
      }
    }
  }

  stbi_write_png("no-intersections.png", WIDTH, HEIGHT, 4, bitmap.get(),
                 WIDTH * sizeof(Pixel));
}