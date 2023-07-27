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

#ifndef PLANETGEN_UTIL_FOREACHPARALLEL_H_
#define PLANETGEN_UTIL_FOREACHPARALLEL_H_

#include <functional>
#include <list>
#include <thread>

namespace planetgen::util {
template <typename Iterator, typename Function>
void forEachParallel(Iterator begin, Iterator end, Function fn,
                     size_t limit = 20) {
  std::list<std::thread> threads;
  for (Iterator curr = begin; curr != end; ++curr) {
    while (threads.size() >= limit) {
      threads.front().join();
      threads.pop_front();
    }
    threads.push_back(std::thread([&fn, curr]() { fn(*curr); }));
  }
  for_each(threads.begin(), threads.end(), [](std::thread &t) { t.join(); });
}
}  // namespace planetgen::util

#endif  // PLANETGEN_UTIL_FOREACHPARALLEL_H_
