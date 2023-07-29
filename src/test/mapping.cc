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
#include <cmath>
#include <cstdint>
#include <execution>
#include <glm/glm.hpp>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>

#include "imgOutput.h"
#include "planet.h"
#include "util/forEachParallel.h"
#include "util/geometry.h"

using namespace planetgen;
using namespace std;
using namespace glm;
using namespace planetgen::util;

constexpr size_t HEIGHT = 2048;
constexpr size_t WIDTH = HEIGHT * 2;

TEST_CASE("Check for no-intersection locations", "[.long]") {
  atomic<GenerationStatus> status;
  EarthlikePlanet::Config config;
  EarthlikePlanet p = EarthlikePlanet(status, 0, config);

  unique_ptr<Pixel[]> bitmap = make_unique<Pixel[]>(WIDTH * HEIGHT);
  forEachParallel(
      CountingIterator(0), CountingIterator(HEIGHT), [&](size_t latIdx) {
        for_each(CountingIterator(0), CountingIterator(WIDTH),
                 [&](size_t lonIdx) {
                   vec2 location = {lerp(M_PI_2f, -M_PI_2f,
                                         static_cast<float>(latIdx) /
                                             static_cast<float>(HEIGHT - 1)),
                                    lerp(0, 2 * M_PIf,
                                         static_cast<float>(lonIdx) /
                                             static_cast<float>(WIDTH - 1))};
                   p[location];
                   bitmap[latIdx * WIDTH + lonIdx] = Pixel{0, 0, 0, 255};
                 });
      });

  stbi_write_png("no-intersections.png", WIDTH, HEIGHT, 4, bitmap.get(),
                 WIDTH * sizeof(Pixel));
}

TEST_CASE("Lookup errors", "[.long]") {
  atomic<GenerationStatus> status;
  EarthlikePlanet::Config config;
  EarthlikePlanet p = EarthlikePlanet(status, 0, config);

  float maxError = 0.f;
  mutex maxErrorMutex;
  unique_ptr<Pixel[]> bitmap = make_unique<Pixel[]>(WIDTH * HEIGHT);
  forEachParallel(
      CountingIterator(0), CountingIterator(HEIGHT), [&](size_t latIdx) {
        float localMaxError = 0.f;
        for_each(
            CountingIterator(0), CountingIterator(WIDTH), [&](size_t lonIdx) {
              vec2 location = {lerp(M_PI_2f, -M_PI_2f,
                                    static_cast<float>(latIdx) /
                                        static_cast<float>(HEIGHT - 1)),
                               lerp(0, 2 * M_PIf,
                                    static_cast<float>(lonIdx) /
                                        static_cast<float>(WIDTH - 1))};

              vec3 expected = sphericalToCartesian(location) * config.radius;
              float error = length(p[location].centroid - expected);
              float linearError =
                  (glm::clamp(error, 50e3f, config.radius * 2.f * M_PIf / 5.f) -
                   50e3f) /
                  (config.radius * 2.f * M_PIf / 5.f - 50e3f);
              bitmap[latIdx * WIDTH + lonIdx] =
                  Pixel{static_cast<uint8_t>(floor(lerp(0, 255, linearError))),
                        0, 0, 255};

              if (error > localMaxError) {
                localMaxError = error;
              }
            });

        scoped_lock _ = scoped_lock(maxErrorMutex);
        if (localMaxError > maxError) {
          maxError = localMaxError;
        }
      });

  stbi_write_png("lookup-errors.png", WIDTH, HEIGHT, 4, bitmap.get(),
                 WIDTH * sizeof(Pixel));
  cout << "Max error: " << maxError << "\n";
}

array<Pixel, 12> const continentalColours = {
    Pixel{255, 0, 0},   Pixel{191, 0, 0},   Pixel{127, 0, 0},
    Pixel{63, 0, 0},    Pixel{0, 255, 0},   Pixel{0, 191, 0},
    Pixel{0, 127, 0},   Pixel{0, 63, 0},    Pixel{255, 255, 0},
    Pixel{191, 191, 0}, Pixel{127, 127, 0}, Pixel{63, 63, 0},
};
array<Pixel, 11> const oceanicColours = {
    Pixel{0, 0, 255}, Pixel{0, 0, 232}, Pixel{0, 0, 207}, Pixel{0, 0, 185},
    Pixel{0, 0, 162}, Pixel{0, 0, 139}, Pixel{0, 0, 116}, Pixel{0, 0, 93},
    Pixel{0, 0, 70},  Pixel{0, 0, 46},  Pixel{0, 0, 23},
};

TEST_CASE("Continental plate map", "[.long][.mapping]") {
  atomic<GenerationStatus> status;
  EarthlikePlanet::Config config;
  EarthlikePlanet p = EarthlikePlanet(status, 0, config);

  vector<Plate const *> continentalPlates;
  vector<Plate const *> oceanicPlates;
  for_each(p.plates.begin(), p.plates.end(),
           [&continentalPlates, &oceanicPlates](Plate const &p) {
             if (p.continental) {
               continentalPlates.push_back(&p);
             } else {
               oceanicPlates.push_back(&p);
             }
           });

  unique_ptr<Pixel[]> bitmap = make_unique<Pixel[]>(WIDTH * HEIGHT);
  forEachParallel(
      CountingIterator(0), CountingIterator(HEIGHT), [&](size_t latIdx) {
        for_each(CountingIterator(0), CountingIterator(WIDTH),
                 [&](size_t lonIdx) {
                   vec2 location = {lerp(M_PI_2f, -M_PI_2f,
                                         static_cast<float>(latIdx) /
                                             static_cast<float>(HEIGHT - 1)),
                                    lerp(0, 2 * M_PIf,
                                         static_cast<float>(lonIdx) /
                                             static_cast<float>(WIDTH - 1))};
                   TerrainData const &data = p[location];
                   if (data.plate->continental) {
                     bitmap[latIdx * WIDTH + lonIdx] =
                         continentalColours[(find(continentalPlates.begin(),
                                                  continentalPlates.end(),
                                                  data.plate) -
                                             continentalPlates.begin()) %
                                            continentalColours.size()];
                   } else {
                     bitmap[latIdx * WIDTH + lonIdx] =
                         oceanicColours[(find(oceanicPlates.begin(),
                                              oceanicPlates.end(), data.plate) -
                                         oceanicPlates.begin()) %
                                        oceanicColours.size()];
                   }
                 });
      });

  stbi_write_png("plates.png", WIDTH, HEIGHT, 4, bitmap.get(),
                 WIDTH * sizeof(Pixel));
}

constexpr float MOVEMENT_SCALE = 1.f;
TEST_CASE("Plate movement map", "[.long][.mapping]") {
  atomic<GenerationStatus> status;
  EarthlikePlanet::Config config;
  EarthlikePlanet p = EarthlikePlanet(status, 0, config);

  unique_ptr<Pixel[]> bitmap = make_unique<Pixel[]>(WIDTH * HEIGHT);
  forEachParallel(
      CountingIterator(0), CountingIterator(HEIGHT), [&](size_t latIdx) {
        for_each(
            CountingIterator(0), CountingIterator(WIDTH), [&](size_t lonIdx) {
              vec2 location = {lerp(M_PI_2f, -M_PI_2f,
                                    static_cast<float>(latIdx) /
                                        static_cast<float>(HEIGHT - 1)),
                               lerp(0, 2 * M_PIf,
                                    static_cast<float>(lonIdx) /
                                        static_cast<float>(WIDTH - 1))};
              TerrainData const &data = p[location];
              vec3 scaled =
                  clamp(data.plateMovement, -MOVEMENT_SCALE, MOVEMENT_SCALE) /
                      (2 * MOVEMENT_SCALE) +
                  0.5f;
              bitmap[latIdx * WIDTH + lonIdx] =
                  Pixel{static_cast<uint8_t>(floor(scaled.x * 255.f)),
                        static_cast<uint8_t>(floor(scaled.y * 255.f)),
                        static_cast<uint8_t>(floor(scaled.z * 255.f)), 255};
            });
      });

  stbi_write_png("plate-movement.png", WIDTH, HEIGHT, 4, bitmap.get(),
                 WIDTH * sizeof(Pixel));
}

TEST_CASE("Elevation map", "[.long][.mapping]") {
  atomic<GenerationStatus> status;
  EarthlikePlanet::Config config;
  EarthlikePlanet p = EarthlikePlanet(status, 0, config);

  float minElevation = 0.f;
  float maxElevation = 0.f;
  mutex elevationMutex;
  unique_ptr<float[]> elevationMap = make_unique<float[]>(WIDTH * HEIGHT);
  forEachParallel(
      CountingIterator(0), CountingIterator(HEIGHT), [&](size_t latIdx) {
        float localMinElevation = 0.f;
        float localMaxElevation = 0.f;
        for_each(CountingIterator(0), CountingIterator(WIDTH),
                 [&](size_t lonIdx) {
                   vec2 location = {lerp(M_PI_2f, -M_PI_2f,
                                         static_cast<float>(latIdx) /
                                             static_cast<float>(HEIGHT - 1)),
                                    lerp(0, 2 * M_PIf,
                                         static_cast<float>(lonIdx) /
                                             static_cast<float>(WIDTH - 1))};
                   TerrainData const &data = p[location];
                   if (data.elevation < localMinElevation) {
                     localMinElevation = data.elevation;
                   }
                   if (data.elevation > localMaxElevation) {
                     localMaxElevation = data.elevation;
                   }
                   elevationMap[latIdx * WIDTH + lonIdx] = data.elevation;
                 });

        scoped_lock _ = scoped_lock(elevationMutex);
        if (localMinElevation < minElevation) {
          minElevation = localMinElevation;
        }
        if (localMaxElevation > maxElevation) {
          maxElevation = localMaxElevation;
        }
      });

  unique_ptr<Pixel[]> bitmap = make_unique<Pixel[]>(WIDTH * HEIGHT);
  forEachParallel(
      CountingIterator(0), CountingIterator(HEIGHT), [&](size_t latIdx) {
        for_each(CountingIterator(0), CountingIterator(WIDTH),
                 [&](size_t lonIdx) {
                   float elevation = elevationMap[latIdx * WIDTH + lonIdx];
                   if (elevation < 0.f) {
                     bitmap[latIdx * WIDTH + lonIdx] = Pixel{
                         0, 0,
                         static_cast<uint8_t>(
                             floor(lerp(255.f, 0.f, elevation / minElevation))),
                         255};
                   } else {
                     bitmap[latIdx * WIDTH + lonIdx] = Pixel{
                         0,
                         static_cast<uint8_t>(
                             floor(lerp(255.f, 0.f, elevation / maxElevation))),
                         0, 255};
                   }
                 });
      });

  stbi_write_png("elevation.png", WIDTH, HEIGHT, 4, bitmap.get(),
                 WIDTH * sizeof(Pixel));
  cout << "Min elevation: " << minElevation
       << "\nMax elevation: " << maxElevation << "\n";
}
