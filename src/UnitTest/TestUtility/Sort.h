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
#include <numeric>
#include <vector>

namespace open3d {
namespace unit_test {

template <class T, int M, int N, int A>
std::vector<Eigen::Matrix<T, M, N, A>> SortApplyIndices(
        const std::vector<Eigen::Matrix<T, M, N, A>>& vals,
        const std::vector<size_t>& indices) {
    std::vector<Eigen::Matrix<T, M, N, A>> vals_sorted;
    for (const size_t& i : indices) {
        vals_sorted.push_back(vals[i]);
    }
    return vals_sorted;
};

template <class T, int M, int N, int A>
std::pair<std::vector<Eigen::Matrix<T, M, N, A>>, std::vector<size_t>>
SortWithIndices(const std::vector<Eigen::Matrix<T, M, N, A>>& vals) {
    // https://stackoverflow.com/a/12399290/1255535
    std::vector<size_t> indices(vals.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::stable_sort(indices.begin(), indices.end(),
                     [&vals](size_t lhs, size_t rhs) -> bool {
                         for (int i = 0; i < vals[lhs].size(); i++) {
                             if (vals[lhs](i) == vals[rhs](i)) {
                                 continue;
                             } else {
                                 return vals[lhs](i) < vals[rhs](i);
                             }
                         }
                         return false;
                     });
    return std::make_pair(SortApplyIndices(vals, indices), indices);
};

template <class T, int M, int N, int A>
std::vector<Eigen::Matrix<T, M, N, A>> Sort(
        const std::vector<Eigen::Matrix<T, M, N, A>>& vals) {
    return SortWithIndices(vals).first;
};

}  // namespace unit_test
}  // namespace open3d
