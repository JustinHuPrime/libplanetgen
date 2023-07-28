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

#ifndef PLANETGEN_UTIL_GEOMETRY_H_
#define PLANETGEN_UTIL_GEOMETRY_H_

#include <cmath>
#include <glm/glm.hpp>

namespace planetgen::util {
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
constexpr glm::vec3 sphericalToCartesian(glm::vec2 const &latLon) noexcept {
  float lat = latLon.s;
  float lon = latLon.t;
  return glm::vec3{cos(lat) * sin(lon), sin(lat), cos(lat) * cos(lon)};
}
}  // namespace planetgen::util

#endif  // PLANETGEN_UTIL_GEOMETRY_H_
