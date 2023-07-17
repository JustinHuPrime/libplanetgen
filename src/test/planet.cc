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

#include <catch2/catch_test_macros.hpp>

using namespace planetgen;
using namespace std;

TEST_CASE("Can construct planet", "[earthlike][constructor][.long]") {
  atomic<GenerationStatus> status;
  EarthlikePlanet::Config config;
  EarthlikePlanet planet = EarthlikePlanet(status, 0, config);
  REQUIRE(status == GenerationStatus::DONE);
}

TEST_CASE("Can get a terrain tile", "[earthlike][constructor][.long]") {
  atomic<GenerationStatus> status;
  EarthlikePlanet::Config config;
  EarthlikePlanet planet = EarthlikePlanet(status, 0, config);
  planet[glm::vec2{0.f, 0.f}];
  REQUIRE(status == GenerationStatus::DONE);
}

TEST_CASE("Close-by terrain tiles are identical",
          "[earthlike][constructor][.long]") {
  atomic<GenerationStatus> status;
  EarthlikePlanet::Config config;
  EarthlikePlanet planet = EarthlikePlanet(status, 0, config);
  TerrainData const *tile1 = &planet[glm::vec2{1e-4f, 1e-4f}];
  TerrainData const *tile2 = &planet[glm::vec2{1e-4f, 2e-4f}];
  REQUIRE(tile1 == tile2);
  REQUIRE(status == GenerationStatus::DONE);
}

TEST_CASE("Far-away terrain tiles are not identical",
          "[earthlike][constructor][.long]") {
  atomic<GenerationStatus> status;
  EarthlikePlanet::Config config;
  EarthlikePlanet planet = EarthlikePlanet(status, 0, config);
  TerrainData const *tile1 = &planet[glm::vec2{0.f, 0.f}];
  TerrainData const *tile2 = &planet[glm::vec2{1e-2f, 0.f}];
  REQUIRE(tile1 != tile2);
  REQUIRE(status == GenerationStatus::DONE);
}

TEST_CASE("Can get a terrain tile near an edge",
          "[earthlike][constructor][.long]") {
  atomic<GenerationStatus> status;
  EarthlikePlanet::Config config;
  EarthlikePlanet planet = EarthlikePlanet(status, 0, config);
  TerrainData const *tile1 = &planet[glm::vec2{M_PIf - 1e-4f, 0}];
  REQUIRE(status == GenerationStatus::DONE);
}

TEST_CASE("Can get a terrain tile near a corner",
          "[earthlike][constructor][.long]") {
  atomic<GenerationStatus> status;
  EarthlikePlanet::Config config;
  EarthlikePlanet planet = EarthlikePlanet(status, 0, config);
  TerrainData const *tile1 = &planet[glm::vec2{M_PIf, 0}];
  REQUIRE(status == GenerationStatus::DONE);
}
