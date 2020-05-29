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

#include "UnitTest/TestUtility/Sort.h"

#include <algorithm>

namespace open3d {
namespace unit_test {

template <class T, int M, int N, int A>
std::vector<Eigen::Matrix<T, M, N, A>> Sort(
        const std::vector<Eigen::Matrix<T, M, N, A>>& vals) {
    std::vector<Eigen::Matrix<T, M, N, A>> ret_vals = vals;
    std::sort(
            ret_vals.begin(), ret_vals.end(),
            [](const Eigen::Matrix<T, M, N, A>& lhs,
               const Eigen::Matrix<T, M, N, A>& rhs) {
                ASSERT_EQ(lhs.size(), rhs.size())
                        << "lhs and rhs have different sizes, cannot be sorted";
                for (int i = 0; i < lhs.size(); i++) {
                    if (lhs(i) == rhs(i)) {
                        continue;
                    } else {
                        return lhs(i) < rhs(i);
                    }
                    return false;
                }
            });
    return ret_vals;
};
// //
// ----------------------------------------------------------------------------
// // Greater than or Equal for sorting Eigen::Vector3d elements.
// //
// ----------------------------------------------------------------------------
// bool Sort::GE(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1) {
//     if (v0(0, 0) > v1(0, 0)) return true;

//     if (v0(0, 0) == v1(0, 0)) {
//         if (v0(1, 0) > v1(1, 0)) return true;

//         if (v0(1, 0) == v1(1, 0)) {
//             if (v0(2, 0) >= v1(2, 0)) return true;
//         }
//     }

//     return false;
// }

// //
// ----------------------------------------------------------------------------
// // Sort a vector of Eigen::Vector3d elements.
// //
// ----------------------------------------------------------------------------
// void Sort::Do(std::vector<Eigen::Vector3d>& v) {
//     Eigen::Vector3d temp(0.0, 0.0, 0.0);
//     for (size_t i = 0; i < v.size(); i++) {
//         for (size_t j = 0; j < v.size(); j++) {
//             if (GE(v[i], v[j])) continue;
//             temp = v[j];
//             v[j] = v[i];
//             v[i] = temp;
//         }
//     }
// }

}  // namespace unit_test
}  // namespace open3d
