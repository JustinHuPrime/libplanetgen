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

#ifndef LIBPLANETGEN_TEST_IMGOUTPUT_H_
#define LIBPLANETGEN_TEST_IMGOUTPUT_H_

#include <cstddef>
#include <cstdint>
#include <utility>

namespace planetgen {
struct Pixel {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t a = 255;
};

struct CountingIterator final {
  size_t value;

  explicit CountingIterator(size_t value) noexcept : value(value) {}
  CountingIterator(CountingIterator const &) noexcept = default;
  CountingIterator(CountingIterator &&) noexcept = default;

  ~CountingIterator() noexcept = default;

  CountingIterator &operator=(CountingIterator const &) noexcept = default;
  CountingIterator &operator=(CountingIterator &&) noexcept = default;

  std::strong_ordering operator<=>(
      CountingIterator const &other) const noexcept {
    return this->value <=> other.value;
  }
  bool operator==(CountingIterator const &) const noexcept = default;
  bool operator!=(CountingIterator const &) const noexcept = default;

  size_t operator*() const noexcept { return value; }

  CountingIterator &operator++() noexcept {
    ++value;
    return *this;
  }

  CountingIterator operator++(int) noexcept {
    CountingIterator retval = *this;
    ++*this;
    return retval;
  }
};
}  // namespace planetgen

#endif  // LIBPLANETGEN_TEST_IMGOUTPUT_H_
