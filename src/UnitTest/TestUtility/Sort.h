// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#pragma once

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <algorithm>
#include <vector>

namespace open3d {
namespace unit_test {

template <class T, int M, int N, int A>
std::vector<Eigen::Matrix<T, M, N, A>> Sort(
        const std::vector<Eigen::Matrix<T, M, N, A>>& vals) {
    std::vector<Eigen::Matrix<T, M, N, A>> ret_vals = vals;
    std::sort(ret_vals.begin(), ret_vals.end(),
              [](const Eigen::Matrix<T, M, N, A>& lhs,
                 const Eigen::Matrix<T, M, N, A>& rhs) -> bool {
                  for (int i = 0; i < lhs.size(); i++) {
                      if (lhs(i) == rhs(i)) {
                          continue;
                      } else {
                          return lhs(i) < rhs(i);
                      }
                  }
                  return false;
              });
    return ret_vals;
};

}  // namespace unit_test
}  // namespace open3d
