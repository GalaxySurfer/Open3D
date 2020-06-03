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

#include <algorithm>

#include "Open3D/Camera/PinholeCameraIntrinsic.h"
#include "Open3D/Geometry/BoundingVolume.h"
#include "Open3D/Geometry/Image.h"
#include "Open3D/Geometry/PointCloud.h"
#include "Open3D/Geometry/RGBDImage.h"
#include "UnitTest/UnitTest.h"

namespace open3d {
namespace unit_test {

TEST(PointCloud, ConstructorDefault) {
    geometry::PointCloud pc;

    EXPECT_EQ(geometry::Geometry::GeometryType::PointCloud,
              pc.GetGeometryType());
    EXPECT_EQ(pc.Dimension(), 3);

    EXPECT_EQ(pc.points_.size(), 0);
    EXPECT_EQ(pc.normals_.size(), 0);
    EXPECT_EQ(pc.colors_.size(), 0);
}

TEST(PointCloud, ConstructorFromPoints) {
    std::vector<Eigen::Vector3d> points = {{0, 1, 2}, {3, 4, 5}};
    geometry::PointCloud pc(points);

    EXPECT_EQ(pc.points_.size(), 2);
    EXPECT_EQ(pc.normals_.size(), 0);
    EXPECT_EQ(pc.colors_.size(), 0);

    ExpectEQ(pc.points_, points);
}

TEST(PointCloud, Clear_IsEmpty) {
    std::vector<Eigen::Vector3d> points = {{0, 1, 2}, {3, 4, 5}};
    std::vector<Eigen::Vector3d> normals = {{0, 1, 2}, {3, 4, 5}};
    std::vector<Eigen::Vector3d> colors = {{0.0, 0.1, 0.2}, {0.3, 0.4, 0.5}};

    geometry::PointCloud pc;
    pc.points_ = points;
    pc.normals_ = normals;
    pc.colors_ = colors;

    EXPECT_FALSE(pc.IsEmpty());
    EXPECT_EQ(pc.points_.size(), 2);
    EXPECT_EQ(pc.normals_.size(), 2);
    EXPECT_EQ(pc.colors_.size(), 2);

    pc.Clear();
    EXPECT_TRUE(pc.IsEmpty());
    EXPECT_EQ(pc.points_.size(), 0);
    EXPECT_EQ(pc.normals_.size(), 0);
    EXPECT_EQ(pc.colors_.size(), 0);
}

TEST(PointCloud, GetMinBound) {
    geometry::PointCloud pc({{1, 10, 20}, {30, 2, 40}, {50, 60, 3}});
    ExpectEQ(pc.GetMinBound(), Eigen::Vector3d(1, 2, 3));

    geometry::PointCloud pc_empty;
    ExpectEQ(pc_empty.GetMinBound(), Eigen::Vector3d(0, 0, 0));
}

TEST(PointCloud, GetMaxBound) {
    geometry::PointCloud pc({{1, 10, 20}, {30, 2, 40}, {50, 60, 3}});
    ExpectEQ(pc.GetMaxBound(), Eigen::Vector3d(50, 60, 40));

    geometry::PointCloud pc_empty;
    ExpectEQ(pc_empty.GetMaxBound(), Eigen::Vector3d(0, 0, 0));
}

TEST(PointCloud, GetCenter) {
    geometry::PointCloud pc({{0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {9, 10, 11}});
    ExpectEQ(pc.GetCenter(), Eigen::Vector3d(4.5, 5.5, 6.5));

    geometry::PointCloud pc_empty;
    ExpectEQ(pc_empty.GetCenter(), Eigen::Vector3d(0, 0, 0));
}

TEST(PointCloud, GetAxisAlignedBoundingBox) {
    geometry::PointCloud pc;
    geometry::AxisAlignedBoundingBox aabb;

    pc = geometry::PointCloud();
    aabb = pc.GetAxisAlignedBoundingBox();
    EXPECT_EQ(aabb.min_bound_, Eigen::Vector3d(0, 0, 0));
    EXPECT_EQ(aabb.max_bound_, Eigen::Vector3d(0, 0, 0));
    EXPECT_EQ(aabb.color_, Eigen::Vector3d(0, 0, 0));

    pc = geometry::PointCloud({{0, 0, 0}});
    aabb = pc.GetAxisAlignedBoundingBox();
    EXPECT_EQ(aabb.min_bound_, Eigen::Vector3d(0, 0, 0));
    EXPECT_EQ(aabb.max_bound_, Eigen::Vector3d(0, 0, 0));
    EXPECT_EQ(aabb.color_, Eigen::Vector3d(0, 0, 0));

    pc = geometry::PointCloud({{0, 2, 0}, {1, 1, 2}, {1, 0, 3}});
    aabb = pc.GetAxisAlignedBoundingBox();
    EXPECT_EQ(aabb.min_bound_, Eigen::Vector3d(0, 0, 0));
    EXPECT_EQ(aabb.max_bound_, Eigen::Vector3d(1, 2, 3));
    EXPECT_EQ(aabb.color_, Eigen::Vector3d(0, 0, 0));

    pc = geometry::PointCloud({{0, 0, 0}, {1, 2, 3}});
    aabb = pc.GetAxisAlignedBoundingBox();
    EXPECT_EQ(aabb.min_bound_, Eigen::Vector3d(0, 0, 0));
    EXPECT_EQ(aabb.max_bound_, Eigen::Vector3d(1, 2, 3));
    EXPECT_EQ(aabb.color_, Eigen::Vector3d(0, 0, 0));
}

TEST(PointCloud, GetOrientedBoundingBox) {
    geometry::PointCloud pc;
    geometry::OrientedBoundingBox obb;

    // Emtpy (GetOrientedBoundingBox requires >=4 points)
    pc = geometry::PointCloud();
    EXPECT_ANY_THROW(pc.GetOrientedBoundingBox());

    // Point
    pc = geometry::PointCloud({{0, 0, 0}});
    EXPECT_ANY_THROW(pc.GetOrientedBoundingBox());
    pc = geometry::PointCloud({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}});
    EXPECT_ANY_THROW(pc.GetOrientedBoundingBox());

    // Line
    pc = geometry::PointCloud({{0, 0, 0}, {1, 1, 1}});
    EXPECT_ANY_THROW(pc.GetOrientedBoundingBox());
    pc = geometry::PointCloud({{0, 0, 0}, {1, 1, 1}, {2, 2, 2}, {3, 3, 3}});
    EXPECT_ANY_THROW(pc.GetOrientedBoundingBox());

    // Plane
    pc = geometry::PointCloud({{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {0, 1, 1}});
    EXPECT_ANY_THROW(pc.GetOrientedBoundingBox());

    // Valid 4 points
    pc = geometry::PointCloud({{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 1, 1}});
    pc.GetOrientedBoundingBox();

    // 8 points with known ground truth
    pc = geometry::PointCloud({{0, 0, 0},
                               {0, 0, 1},
                               {0, 2, 0},
                               {0, 2, 1},
                               {3, 0, 0},
                               {3, 0, 1},
                               {3, 2, 0},
                               {3, 2, 1}});
    obb = pc.GetOrientedBoundingBox();
    EXPECT_EQ(obb.center_, Eigen::Vector3d(1.5, 1, 0.5));
    EXPECT_EQ(obb.extent_, Eigen::Vector3d(3, 2, 1));
    EXPECT_EQ(obb.color_, Eigen::Vector3d(0, 0, 0));
    EXPECT_EQ(obb.R_, Eigen::Matrix3d::Identity());
    ExpectEQ(Sort(obb.GetBoxPoints()),
             Sort(std::vector<Eigen::Vector3d>({{0, 0, 0},
                                                {0, 0, 1},
                                                {0, 2, 0},
                                                {0, 2, 1},
                                                {3, 0, 0},
                                                {3, 0, 1},
                                                {3, 2, 0},
                                                {3, 2, 1}})));
}

TEST(PointCloud, Transform) {
    std::vector<Eigen::Vector3d> points = {
            {0, 0, 0},
            {1, 2, 4},
    };
    std::vector<Eigen::Vector3d> normals = {
            {4, 2, 1},
            {0, 0, 0},
    };
    Eigen::Matrix4d transformation;
    transformation << 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0,
            5.5, 6.0, 6.5, 7.0, 7.5;

    std::vector<Eigen::Vector3d> points_transformed = {
            {0.20000, 0.46666, 0.73333},
            {0.11926, 0.41284, 0.70642},
    };
    std::vector<Eigen::Vector3d> normals_transformed = {
            {2, 16, 30},
            {0, 0, 0},
    };

    geometry::PointCloud pc;
    pc.points_ = points;
    pc.normals_ = normals;
    pc.Transform(transformation);
    ExpectEQ(pc.points_, points_transformed, 1e-4);
    ExpectEQ(pc.normals_, normals_transformed, 1e-4);
}

TEST(PointCloud, Translate) {
    std::vector<Eigen::Vector3d> points = {
            {0, 1, 2},
            {6, 7, 8},
    };
    Eigen::Vector3d translation(10, 20, 30);
    std::vector<Eigen::Vector3d> points_translated = {
            {10, 21, 32},
            {16, 27, 38},
    };
    std::vector<Eigen::Vector3d> points_translated_non_relative = {
            {7, 17, 27},
            {13, 23, 33},
    };

    // Relative: direct translation.
    geometry::PointCloud pc;
    pc.points_ = points;
    pc.Translate(translation);
    ExpectEQ(pc.points_, points_translated);

    // Non-relative: new center is the translation.
    pc.points_ = points;
    pc.Translate(translation, /*relative=*/false);
    ExpectEQ(pc.points_, points_translated_non_relative);
    ExpectEQ(pc.GetCenter(), translation);
}

TEST(PointCloud, Scale) {
    std::vector<Eigen::Vector3d> points = {{0, 1, 2}, {6, 7, 8}};
    double scale = 10;
    geometry::PointCloud pc;

    pc.points_ = points;
    pc.Scale(scale, Eigen::Vector3d(0, 0, 0));
    ExpectEQ(pc.points_,
             std::vector<Eigen::Vector3d>({{0, 10, 20}, {60, 70, 80}}));

    pc.points_ = points;
    pc.Scale(scale, Eigen::Vector3d(1, 1, 1));
    ExpectEQ(pc.points_,
             std::vector<Eigen::Vector3d>({{-9, 1, 11}, {51, 61, 71}}));
}

TEST(PointCloud, Rotate) {
    std::vector<Eigen::Vector3d> points = {{0, 1, 2}, {3, 4, 5}};
    std::vector<Eigen::Vector3d> normals = {{5, 4, 3}, {2, 1, 0}};
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.33 * M_PI, Eigen::Vector3d::UnitZ());

    geometry::PointCloud pc;
    pc.points_ = points;
    pc.normals_ = normals;
    Eigen::Vector3d center = pc.GetCenter();
    pc.Rotate(R, center);

    ExpectEQ(pc.points_, std::vector<Eigen::Vector3d>({{0, 1.42016, 1.67409},
                                                       {3, 3.57984, 5.32591}}));
    ExpectEQ(pc.normals_,
             std::vector<Eigen::Vector3d>(
                     {{3, 3.84816, 5.11778}, {0, 1.688476, 1.465963}}));
    ExpectEQ(pc.GetCenter(), center);  // Rotate relative to the original center
}

TEST(PointCloud, OperatorPlusEqual) {
    std::vector<Eigen::Vector3d> points_a = {{0, 1, 2}, {3, 4, 5}};
    std::vector<Eigen::Vector3d> normals_a = {{0, 1, 2}, {3, 4, 5}};
    std::vector<Eigen::Vector3d> colors_a = {{0, 1, 2}, {3, 4, 5}};

    std::vector<Eigen::Vector3d> points_b = {{6, 7, 8}, {9, 10, 11}};
    std::vector<Eigen::Vector3d> normals_b = {{6, 7, 8}, {9, 10, 11}};
    std::vector<Eigen::Vector3d> colors_b = {{6, 7, 8}, {9, 10, 11}};

    std::vector<Eigen::Vector3d> empty(0);

    std::vector<Eigen::Vector3d> points_a_b = {
            {0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {9, 10, 11}};
    std::vector<Eigen::Vector3d> normals_a_b = {
            {0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {9, 10, 11}};
    std::vector<Eigen::Vector3d> colors_a_b = {
            {0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {9, 10, 11}};

    geometry::PointCloud pc_a_full;
    geometry::PointCloud pc_b_full;
    pc_a_full.points_ = points_a;
    pc_a_full.normals_ = normals_a;
    pc_a_full.colors_ = colors_a;
    pc_b_full.points_ = points_b;
    pc_b_full.normals_ = normals_b;
    pc_b_full.colors_ = colors_b;

    geometry::PointCloud pc_a = pc_a_full;
    geometry::PointCloud pc_b = pc_b_full;
    pc_a += pc_b;
    ExpectEQ(pc_a.points_, points_a_b);
    ExpectEQ(pc_a.normals_, normals_a_b);
    ExpectEQ(pc_a.colors_, colors_a_b);

    pc_a = pc_a_full;
    pc_b = pc_b_full;
    pc_a.normals_.clear();
    pc_a += pc_b;
    ExpectEQ(pc_a.points_, points_a_b);
    ExpectEQ(pc_a.normals_, empty);
    ExpectEQ(pc_a.colors_, colors_a_b);

    pc_a = pc_a_full;
    pc_b = pc_b_full;
    pc_b.normals_.clear();
    pc_a += pc_b;
    ExpectEQ(pc_a.points_, points_a_b);
    ExpectEQ(pc_a.normals_, empty);
    ExpectEQ(pc_a.colors_, colors_a_b);

    pc_a = pc_a_full;
    pc_b = pc_b_full;
    pc_a.colors_.clear();
    pc_a += pc_b;
    ExpectEQ(pc_a.points_, points_a_b);
    ExpectEQ(pc_a.normals_, normals_a_b);
    ExpectEQ(pc_a.colors_, empty);

    pc_a = pc_a_full;
    pc_b = pc_b_full;
    pc_b.colors_.clear();
    pc_a += pc_b;
    ExpectEQ(pc_a.points_, points_a_b);
    ExpectEQ(pc_a.normals_, normals_a_b);
    ExpectEQ(pc_a.colors_, empty);

    pc_a.Clear();
    pc_a += pc_b_full;
    ExpectEQ(pc_a.points_, pc_b_full.points_);
    ExpectEQ(pc_a.normals_, pc_b_full.normals_);
    ExpectEQ(pc_a.colors_, pc_b_full.colors_);

    pc_a = pc_a_full;
    pc_b.Clear();
    pc_a += pc_b;
    ExpectEQ(pc_a.points_, pc_a_full.points_);
    ExpectEQ(pc_a.normals_, pc_a_full.normals_);
    ExpectEQ(pc_a.colors_, pc_a_full.colors_);
}

TEST(PointCloud, OperatorPlus) {
    std::vector<Eigen::Vector3d> points_a = {{0, 1, 2}};
    std::vector<Eigen::Vector3d> normals_a = {{0, 1, 2}};
    std::vector<Eigen::Vector3d> colors_a = {{0, 1, 2}};
    std::vector<Eigen::Vector3d> points_b = {{3, 4, 5}};
    std::vector<Eigen::Vector3d> normals_b = {{3, 4, 5}};
    std::vector<Eigen::Vector3d> colors_b = {{3, 4, 5}};

    geometry::PointCloud pc_a;
    geometry::PointCloud pc_b;
    pc_a.points_ = points_a;
    pc_a.normals_ = normals_a;
    pc_a.colors_ = colors_a;
    pc_b.points_ = points_b;
    pc_b.normals_ = normals_b;
    pc_b.colors_ = colors_b;

    geometry::PointCloud pc_c = pc_a + pc_b;
    ExpectEQ(pc_c.points_,
             std::vector<Eigen::Vector3d>({{0, 1, 2}, {3, 4, 5}}));
    ExpectEQ(pc_c.normals_,
             std::vector<Eigen::Vector3d>({{0, 1, 2}, {3, 4, 5}}));
    ExpectEQ(pc_c.colors_,
             std::vector<Eigen::Vector3d>({{0, 1, 2}, {3, 4, 5}}));
}

TEST(PointCloud, HasPoints) {
    geometry::PointCloud pc;
    EXPECT_FALSE(pc.HasPoints());
    pc.points_.resize(5);
    EXPECT_TRUE(pc.HasPoints());
}

TEST(PointCloud, HasNormals) {
    geometry::PointCloud pc;
    EXPECT_FALSE(pc.HasNormals());

    // False if #points == 0
    pc.points_.resize(0);
    pc.normals_.resize(5);
    EXPECT_FALSE(pc.HasNormals());

    // False if not consistant
    pc.points_.resize(4);
    pc.normals_.resize(5);
    EXPECT_FALSE(pc.HasNormals());

    // True if non-zero and consistant
    pc.points_.resize(5);
    pc.normals_.resize(5);
    EXPECT_TRUE(pc.HasNormals());
}

TEST(PointCloud, HasColors) {
    geometry::PointCloud pc;
    EXPECT_FALSE(pc.HasNormals());

    // False if #points == 0
    pc.points_.resize(0);
    pc.colors_.resize(5);
    EXPECT_FALSE(pc.HasColors());

    // False if not consistant
    pc.points_.resize(4);
    pc.colors_.resize(5);
    EXPECT_FALSE(pc.HasColors());

    // True if non-zero and consistant
    pc.points_.resize(5);
    pc.colors_.resize(5);
    EXPECT_TRUE(pc.HasColors());
}

TEST(PointCloud, NormalizeNormals) {
    geometry::PointCloud pc;
    pc.normals_ = {{2, 2, 2}, {1, 1, 1}, {-1, -1, -1}, {0, 0, 1},
                   {0, 1, 0}, {1, 0, 0}, {0, 0, 0}};
    pc.NormalizeNormals();
    ExpectEQ(pc.normals_, std::vector<Eigen::Vector3d>({
                                  {0.57735, 0.57735, 0.57735},
                                  {0.57735, 0.57735, 0.57735},
                                  {-0.57735, -0.57735, -0.57735},
                                  {0, 0, 1},
                                  {0, 1, 0},
                                  {1, 0, 0},
                                  {0, 0, 0},
                          }));
}

TEST(PointCloud, PaintUniformColor) {
    geometry::PointCloud pc;
    pc.points_.resize(2);
    EXPECT_EQ(pc.points_.size(), 2);
    EXPECT_EQ(pc.colors_.size(), 0);

    Eigen::Vector3d color(0.125, 0.25, 0.5);
    pc.PaintUniformColor(color);
    EXPECT_EQ(pc.colors_.size(), 2);

    EXPECT_EQ(pc.colors_, std::vector<Eigen::Vector3d>({color, color}));
}

TEST(PointCloud, SelectByIndex) {
    std::vector<Eigen::Vector3d> points({
            {0, 0, 0},
            {1, 1, 1},
            {2, 2, 2},
            {3, 3, 3},
    });
    std::vector<Eigen::Vector3d> colors({
            {0.0, 0.0, 0.0},
            {0.1, 0.1, 0.1},
            {0.2, 0.2, 0.2},
            {0.3, 0.3, 0.3},
    });
    std::vector<Eigen::Vector3d> normals({
            {10, 10, 10},
            {11, 11, 11},
            {12, 12, 12},
            {13, 13, 13},
    });

    std::vector<size_t> indices{0, 1, 3};

    geometry::PointCloud pc;
    pc.points_ = points;
    pc.colors_ = colors;
    pc.normals_ = normals;

    std::shared_ptr<geometry::PointCloud> pc0 = pc.SelectByIndex(indices);
    ExpectEQ(pc0->points_, std::vector<Eigen::Vector3d>({
                                   {0, 0, 0},
                                   {1, 1, 1},
                                   {3, 3, 3},
                           }));
    ExpectEQ(pc0->colors_, std::vector<Eigen::Vector3d>({
                                   {0.0, 0.0, 0.0},
                                   {0.1, 0.1, 0.1},
                                   {0.3, 0.3, 0.3},
                           }));
    ExpectEQ(pc0->normals_, std::vector<Eigen::Vector3d>({
                                    {10, 10, 10},
                                    {11, 11, 11},
                                    {13, 13, 13},
                            }));

    std::shared_ptr<geometry::PointCloud> pc1 =
            pc.SelectByIndex(indices, /*invert=*/true);
    ExpectEQ(pc1->points_, std::vector<Eigen::Vector3d>({
                                   {2, 2, 2},
                           }));
    ExpectEQ(pc1->colors_, std::vector<Eigen::Vector3d>({
                                   {0.2, 0.2, 0.2},
                           }));
    ExpectEQ(pc1->normals_, std::vector<Eigen::Vector3d>({
                                    {12, 12, 12},
                            }));
}

TEST(PointCloud, VoxelDownSample) {
    // voxel_size: 1
    // points_min_bound: (0.5, 0.5, 0.5)
    // voxel_min_bound: (0, 0, 0)
    // points coordinates: range from [0.5, 2.5)
    // voxel_{i,j,k}: 0 <= i, j, k <= 2; 27 possible voxels
    std::vector<Eigen::Vector3d> points{
            // voxel_{0, 0, 0}
            {0.5, 0.7, 0.6},
            {0.6, 0.5, 0.7},
            {0.7, 0.6, 0.5},
            {0.8, 0.8, 0.8},
            // voxel_{0, 1, 2}
            {0.5, 1.6, 2.4},
            {0.6, 1.5, 2.3},
    };
    std::vector<Eigen::Vector3d> normals{
            // voxel_{0, 0, 0}
            {0, 0, 1},
            {0, 2, 3},
            {0, 4, 5},
            {0, 6, 7},
            // voxel_{0, 1, 2}
            {1, 0, 1},
            {1, 2, 3},
    };
    std::vector<Eigen::Vector3d> colors{
            // voxel_{0, 0, 0}
            {0.0, 0.0, 0.1},
            {0.0, 0.2, 0.3},
            {0.0, 0.4, 0.5},
            {0.0, 0.6, 0.7},
            // voxel_{0, 1, 2}
            {0.1, 0.0, 0.1},
            {0.1, 0.2, 0.3},
    };
    geometry::PointCloud pc;
    pc.points_ = points;
    pc.normals_ = normals;
    pc.colors_ = colors;

    // Ground-truth reference
    std::vector<Eigen::Vector3d> points_down{
            {0.65, 0.65, 0.65},
            {0.55, 1.55, 2.35},
    };
    std::vector<Eigen::Vector3d> normals_down{
            {0, 3, 4},
            {1, 1, 2},
    };
    std::vector<Eigen::Vector3d> colors_down{
            {0.0, 0.3, 0.4},
            {0.1, 0.1, 0.2},
    };

    std::shared_ptr<geometry::PointCloud> pc_down = pc.VoxelDownSample(1.0);
    std::vector<size_t> sort_indices =
            GetIndicesAToB(pc_down->points_, points_down);

    ExpectEQ(ApplyIndices(pc_down->points_, sort_indices), points_down);
    ExpectEQ(ApplyIndices(pc_down->normals_, sort_indices), normals_down);
    ExpectEQ(ApplyIndices(pc_down->colors_, sort_indices), colors_down);
}

TEST(PointCloud, UniformDownSample) {
    std::vector<Eigen::Vector3d> points({
            {0, 0, 0},
            {1, 0, 0},
            {2, 0, 0},
            {3, 0, 0},
            {4, 0, 0},
            {5, 0, 0},
            {6, 0, 0},
            {7, 0, 0},
    });
    std::vector<Eigen::Vector3d> normals({
            {0, 0, 0},
            {0, 1, 0},
            {0, 2, 0},
            {0, 3, 0},
            {0, 4, 0},
            {0, 5, 0},
            {0, 6, 0},
            {0, 7, 0},
    });
    std::vector<Eigen::Vector3d> colors({
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 0.1},
            {0.0, 0.0, 0.2},
            {0.0, 0.0, 0.3},
            {0.0, 0.0, 0.4},
            {0.0, 0.0, 0.5},
            {0.0, 0.0, 0.6},
            {0.0, 0.0, 0.7},
    });
    geometry::PointCloud pc;
    pc.points_ = points;
    pc.normals_ = normals;
    pc.colors_ = colors;

    std::shared_ptr<geometry::PointCloud> pc_down = pc.UniformDownSample(3);
    ExpectEQ(pc_down->points_, std::vector<Eigen::Vector3d>({
                                       {0, 0, 0},
                                       {3, 0, 0},
                                       {6, 0, 0},

                               }));
    ExpectEQ(pc_down->normals_, std::vector<Eigen::Vector3d>({
                                        {0, 0, 0},
                                        {0, 3, 0},
                                        {0, 6, 0},
                                }));
    ExpectEQ(pc_down->colors_, std::vector<Eigen::Vector3d>({
                                       {0.0, 0.0, 0.0},
                                       {0.0, 0.0, 0.3},
                                       {0.0, 0.0, 0.6},
                               }));
}

TEST(PointCloud, Crop_AxisAlignedBoundingBox) {
    geometry::AxisAlignedBoundingBox aabb({0, 0, 0}, {2, 2, 2});
    geometry::PointCloud pc({{0, 0, 0},
                             {2, 2, 2},
                             {1, 1, 1},
                             {1, 1, 2},
                             {3, 1, 1},
                             {-1, 1, 1}});
    pc.normals_ = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0},
                   {3, 0, 0}, {4, 0, 0}, {5, 0, 0}};
    pc.colors_ = {{0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}, {0.2, 0.0, 0.0},
                  {0.3, 0.0, 0.0}, {0.4, 0.0, 0.0}, {0.5, 0.0, 0.0}};

    std::shared_ptr<geometry::PointCloud> pc_crop = pc.Crop(aabb);
    ExpectEQ(pc_crop->points_, std::vector<Eigen::Vector3d>({
                                       {0, 0, 0},
                                       {2, 2, 2},
                                       {1, 1, 1},
                                       {1, 1, 2},
                               }));
    ExpectEQ(pc_crop->normals_, std::vector<Eigen::Vector3d>({
                                        {0, 0, 0},
                                        {1, 0, 0},
                                        {2, 0, 0},
                                        {3, 0, 0},
                                }));
    ExpectEQ(pc_crop->colors_, std::vector<Eigen::Vector3d>({
                                       {0.0, 0.0, 0.0},
                                       {0.1, 0.0, 0.0},
                                       {0.2, 0.0, 0.0},
                                       {0.3, 0.0, 0.0},
                               }));
}

TEST(PointCloud, Crop_OrientedBoundingBox) {
    geometry::OrientedBoundingBox obb(Eigen::Vector3d{1, 1, 1},
                                      Eigen::Matrix3d::Identity(),
                                      Eigen::Vector3d{2, 2, 2});
    geometry::PointCloud pc({
            {0, 0, 0},
            {2, 2, 2},
            {1, 1, 1},
            {1, 1, 2},
            {3, 1, 1},
            {-1, 1, 1},
    });
    pc.normals_ = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0},
                   {3, 0, 0}, {4, 0, 0}, {5, 0, 0}};
    pc.colors_ = {{0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}, {0.2, 0.0, 0.0},
                  {0.3, 0.0, 0.0}, {0.4, 0.0, 0.0}, {0.5, 0.0, 0.0}};

    std::shared_ptr<geometry::PointCloud> pc_crop = pc.Crop(obb);
    ExpectEQ(pc_crop->points_, std::vector<Eigen::Vector3d>({
                                       {0, 0, 0},
                                       {2, 2, 2},
                                       {1, 1, 1},
                                       {1, 1, 2},
                               }));
    ExpectEQ(pc_crop->normals_, std::vector<Eigen::Vector3d>({
                                        {0, 0, 0},
                                        {1, 0, 0},
                                        {2, 0, 0},
                                        {3, 0, 0},
                                }));
    ExpectEQ(pc_crop->colors_, std::vector<Eigen::Vector3d>({
                                       {0.0, 0.0, 0.0},
                                       {0.1, 0.0, 0.0},
                                       {0.2, 0.0, 0.0},
                                       {0.3, 0.0, 0.0},
                               }));
}

TEST(PointCloud, EstimateNormals) {
    geometry::PointCloud pc({
            {0, 0, 0},
            {0, 0, 1},
            {0, 1, 0},
            {0, 1, 1},
            {1, 0, 0},
            {1, 0, 1},
            {1, 1, 0},
            {1, 1, 1},
    });
    pc.EstimateNormals(geometry::KDTreeSearchParamKNN(/*knn=*/4));
    pc.NormalizeNormals();
    double v = 1.0 / std::sqrt(3.0);
    ExpectEQ(pc.normals_, std::vector<Eigen::Vector3d>({{v, v, v},
                                                        {-v, -v, v},
                                                        {v, -v, v},
                                                        {-v, v, v},
                                                        {-v, v, v},
                                                        {v, -v, v},
                                                        {-v, -v, v},
                                                        {v, v, v}}));
}

TEST(PointCloud, OrientNormalsToAlignWithDirection) {
    geometry::PointCloud pc({
            {0, 0, 0},
            {0, 0, 1},
            {0, 1, 0},
            {0, 1, 1},
            {1, 0, 0},
            {1, 0, 1},
            {1, 1, 0},
            {1, 1, 1},
    });
    pc.EstimateNormals(geometry::KDTreeSearchParamKNN(/*knn=*/4));
    pc.NormalizeNormals();
    double v = 1.0 / std::sqrt(3.0);
    pc.OrientNormalsToAlignWithDirection(Eigen::Vector3d{0, 0, -1});
    ExpectEQ(pc.normals_, std::vector<Eigen::Vector3d>({{-v, -v, -v},
                                                        {v, v, -v},
                                                        {-v, v, -v},
                                                        {v, -v, -v},
                                                        {v, -v, -v},
                                                        {-v, v, -v},
                                                        {v, v, -v},
                                                        {-v, -v, -v}}));

    // normal.norm() == 0 case
    pc.points_ = std::vector<Eigen::Vector3d>{{10, 10, 10}};
    pc.normals_ = std::vector<Eigen::Vector3d>{{0, 0, 0}};
    pc.OrientNormalsToAlignWithDirection(Eigen::Vector3d{0, 0, -1});
    pc.normals_ = std::vector<Eigen::Vector3d>{{0, 0, -1}};
}

TEST(PointCloud, OrientNormalsTowardsCameraLocation) {
    geometry::PointCloud pc({
            {0, 0, 0},
            {0, 1, 0},
            {1, 0, 0},
            {1, 1, 0},
    });
    pc.EstimateNormals(geometry::KDTreeSearchParamKNN(/*knn=*/4));
    pc.NormalizeNormals();
    std::vector<Eigen::Vector3d> ref_normals(
            {{0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}});
    std::vector<Eigen::Vector3d> ref_normals_rev(
            {{0, 0, -1}, {0, 0, -1}, {0, 0, -1}, {0, 0, -1}});

    // Initial
    ExpectEQ(pc.normals_, ref_normals);
    // Camera outside
    pc.OrientNormalsTowardsCameraLocation(Eigen::Vector3d{2, 3, 4});
    ExpectEQ(pc.normals_, ref_normals);
    // Camera inside
    pc.OrientNormalsTowardsCameraLocation(Eigen::Vector3d{-2, -3, -4});
    ExpectEQ(pc.normals_, ref_normals_rev);
}

TEST(PointCloud, OrientNormalsConsistentTangentPlane) {
    geometry::PointCloud pc({
            {0, 0, 0},
            {0, 0, 1},
            {0, 1, 0},
            {0, 1, 1},
            {1, 0, 0},
            {1, 0, 1},
            {1, 1, 0},
            {1, 1, 1},
            {0.5, 0.5, 0},
            {0.5, 0.5, 1},
            {0.5, 0, 0.5},
            {0.5, 1, 0.5},
            {0, 0.5, 0.5},
            {1, 0.5, 0.5},
    });
    pc.EstimateNormals(geometry::KDTreeSearchParamKNN(/*knn=*/4));
    ExpectEQ(pc.normals_, std::vector<Eigen::Vector3d>({{0, 0, 1},
                                                        {0, 0, 1},
                                                        {0, 0, 1},
                                                        {0, 0, 1},
                                                        {0, 0, 1},
                                                        {0, 0, 1},
                                                        {0, 0, 1},
                                                        {0, 0, 1},
                                                        {0, 0, -1},
                                                        {0, 0, -1},
                                                        {0, 1, 0},
                                                        {0, 1, 0},
                                                        {1, 0, 0},
                                                        {1, 0, 0}}));

    pc.OrientNormalsConsistentTangentPlane(/*k=*/4);
    ExpectEQ(pc.normals_, std::vector<Eigen::Vector3d>({{0, 0, -1},
                                                        {0, 0, 1},
                                                        {0, 0, 1},
                                                        {0, 0, 1},
                                                        {0, 0, 1},
                                                        {0, 0, 1},
                                                        {0, 0, 1},
                                                        {0, 0, 1},
                                                        {0, 0, -1},
                                                        {0, 0, 1},
                                                        {0, 1, 0},
                                                        {0, 1, 0},
                                                        {1, 0, 0},
                                                        {1, 0, 0}}));
}

TEST(PointCloud, ComputePointCloudToPointCloudDistance) {
    geometry::PointCloud pc0({{0, 0, 0}, {1, 2, 0}, {2, 2, 0}});
    geometry::PointCloud pc1({{-1, 0, 0}, {-2, 0, 0}, {-1, 2, 0}});

    pc0.ComputePointCloudDistance(pc1);
    ExpectEQ(pc0.ComputePointCloudDistance(pc1),
             std::vector<double>({1, 2, 3}));
}

TEST(PointCloud, ComputeMeanAndCovariance) {
    geometry::PointCloud pc({
            {0, 0, 0},
            {0, 0, 1},
            {0, 1, 0},
            {0, 1, 1},
            {1, 0, 0},
            {1, 0, 1},
            {1, 1, 0},
            {1, 1, 1},
    });

    Eigen::Vector3d ref_mean(0.5, 0.5, 0.5);
    Eigen::Matrix3d ref_covariance;
    ref_covariance << 0.25, 0, 0, 0, 0.25, 0, 0, 0, 0.25;

    Eigen::Vector3d mean;
    Eigen::Matrix3d covariance;
    std::tie(mean, covariance) = pc.ComputeMeanAndCovariance();
    ExpectEQ(mean, ref_mean);
    ExpectEQ(covariance, ref_covariance);

    pc.points_ = {};
    ref_mean = Eigen::Vector3d::Zero();
    ref_covariance = Eigen::Matrix3d::Identity();
    std::tie(mean, covariance) = pc.ComputeMeanAndCovariance();
    ExpectEQ(mean, ref_mean);
    ExpectEQ(covariance, ref_covariance);

    pc.points_ = {{1, 1, 1}};
    ref_mean = Eigen::Vector3d({1, 1, 1});
    ref_covariance = Eigen::Matrix3d::Zero();
    std::tie(mean, covariance) = pc.ComputeMeanAndCovariance();
    ExpectEQ(mean, ref_mean);
    ExpectEQ(covariance, ref_covariance);
}

TEST(PointCloud, ComputeMahalanobisDistance) {
    geometry::PointCloud pc({
            {0, 0, 0},
            {0, 0, 1},
            {0, 1, 0},
            {0, 0, 2},
            {1, 1, 1},
    });
    std::vector<double> distance = pc.ComputeMahalanobisDistance();
    ExpectEQ(distance,
             std::vector<double>({1.77951, 0.81650, 2.00000, 1.77951, 2.00000}),
             1e-4);

    // Empty
    pc.points_ = {};
    distance = pc.ComputeMahalanobisDistance();
    ExpectEQ(distance, std::vector<double>({}));

    // Nan if the covariance is not inversable
    pc.points_ = {{1, 1, 1}};
    distance = pc.ComputeMahalanobisDistance();
    EXPECT_EQ(distance.size(), 1);
    EXPECT_TRUE(std::isnan(distance[0]));

    pc.points_ = {{0, 0, 0}, {1, 1, 1}};
    distance = pc.ComputeMahalanobisDistance();
    EXPECT_EQ(distance.size(), 2);
    EXPECT_TRUE(std::isnan(distance[0]) && std::isnan(distance[1]));
}

TEST(PointCloud, ComputeNearestNeighborDistance) {
    geometry::PointCloud pc;

    // Regular case
    pc.points_ = {{0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {1, 2, 0}};
    ExpectEQ(pc.ComputeNearestNeighborDistance(),
             std::vector<double>({0, 0, 1, 2}));

    // < 2 points
    pc.points_ = {};
    ExpectEQ(pc.ComputeNearestNeighborDistance(), std::vector<double>({}));
    pc.points_ = {{10, 10, 10}};
    ExpectEQ(pc.ComputeNearestNeighborDistance(), std::vector<double>({0}));

    // 2 points
    pc.points_ = {{0, 0, 0}, {1, 0, 0}};
    ExpectEQ(pc.ComputeNearestNeighborDistance(), std::vector<double>({1, 1}));
}

TEST(PointCloud, ComputeConvexHull) {
    geometry::PointCloud pc;

    // Needs at least 4 points
    pc.points_ = {};
    EXPECT_ANY_THROW(pc.ComputeConvexHull());
    pc.points_ = {{0, 0, 0}, {0, 0, 1}, {0, 1, 0}};
    EXPECT_ANY_THROW(pc.ComputeConvexHull());

    // // Regular case
    // pc.points_ = std::vector<Eigen::Vector3d>(
    //         {{0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {1, 2, 0}});
    // ExpectEQ(pc.ComputeNearestNeighborDistance(),
    //          std::vector<double>({0, 0, 1, 2}));

    // // < 2 points
    // pc.points_ = std::vector<Eigen::Vector3d>({});
    // ExpectEQ(pc.ComputeNearestNeighborDistance(), std::vector<double>({}));
    // pc.points_ = std::vector<Eigen::Vector3d>({{10, 10, 10}});
    // ExpectEQ(pc.ComputeNearestNeighborDistance(), std::vector<double>({0}));

    // // 2 points
    // pc.points_ = std::vector<Eigen::Vector3d>({{0, 0, 0}, {1, 0, 0}});
    // ExpectEQ(pc.ComputeNearestNeighborDistance(), std::vector<double>({1,
    // 1}));
}

TEST(PointCloud, DISABLED_CreatePointCloudFromDepthImage) {
    std::vector<Eigen::Vector3d> ref = {{-15.709662, -11.776101, 25.813999},
                                        {-31.647980, -23.798088, 52.167000},
                                        {-7.881257, -5.945074, 13.032000},
                                        {-30.145872, -22.811805, 50.005001},
                                        {-21.734044, -16.498585, 36.166000},
                                        {-25.000724, -18.662512, 41.081001},
                                        {-20.246287, -15.160878, 33.373001},
                                        {-36.219190, -27.207171, 59.889999},
                                        {-28.185984, -21.239675, 46.754002},
                                        {-23.713580, -17.926114, 39.459999},
                                        {-9.505886, -7.066190, 15.620000},
                                        {-31.858493, -23.756333, 52.514000},
                                        {-15.815128, -11.830214, 26.150999},
                                        {-4.186843, -3.141786, 6.945000},
                                        {-8.614051, -6.484428, 14.334000},
                                        {-33.263298, -24.622128, 54.658001},
                                        {-11.742641, -8.719418, 19.356001},
                                        {-20.688904, -15.410790, 34.209999},
                                        {-38.349551, -28.656141, 63.612999},
                                        {-30.197857, -22.636429, 50.250000},
                                        {-30.617229, -22.567629, 50.310001},
                                        {-35.316494, -26.113137, 58.214001},
                                        {-13.822439, -10.252549, 22.856001},
                                        {-36.237141, -26.963181, 60.109001},
                                        {-37.240419, -27.797524, 61.969002}};

    geometry::Image image;

    // test image dimensions
    int local_width = 5;
    int local_height = 5;
    int local_num_of_channels = 1;
    int local_bytes_per_channel = 2;

    image.Prepare(local_width, local_height, local_num_of_channels,
                  local_bytes_per_channel);

    Rand(image.data_, 0, 255, 0);

    camera::PinholeCameraIntrinsic intrinsic = camera::PinholeCameraIntrinsic(
            camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);

    auto output_pc =
            geometry::PointCloud::CreateFromDepthImage(image, intrinsic);

    ExpectEQ(ref, output_pc->points_);
}

// ----------------------------------------------------------------------------
// Test CreatePointCloudFromRGBDImage for the following configurations:
// index | color_num_of_channels | color_bytes_per_channel
//     1 |          3            |            1
//     0 |          1            |            4
// ----------------------------------------------------------------------------
void TEST_CreatePointCloudFromRGBDImage(
        const int& color_num_of_channels,
        const int& color_bytes_per_channel,
        const std::vector<Eigen::Vector3d>& ref_points,
        const std::vector<Eigen::Vector3d>& ref_colors) {
    geometry::Image image;
    geometry::Image color;

    int size = 5;

    // test image dimensions
    int width = size;
    int height = size;
    int num_of_channels = 1;
    int bytes_per_channel = 1;

    image.Prepare(width, height, num_of_channels, bytes_per_channel);

    color.Prepare(width, height, color_num_of_channels,
                  color_bytes_per_channel);

    Rand(image.data_, 100, 150, 0);
    Rand(color.data_, 130, 200, 0);

    auto depth = image.ConvertDepthToFloatImage();

    geometry::RGBDImage rgbd_image(color, *depth);

    camera::PinholeCameraIntrinsic intrinsic = camera::PinholeCameraIntrinsic(
            camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);

    auto output_pc =
            geometry::PointCloud::CreateFromRGBDImage(rgbd_image, intrinsic);

    ExpectEQ(ref_points, output_pc->points_);
    ExpectEQ(ref_colors, output_pc->colors_);
}

// ----------------------------------------------------------------------------
// Test CreatePointCloudFromRGBDImage for the following configuration:
// color_num_of_channels = 3
// color_bytes_per_channel = 1
// ----------------------------------------------------------------------------
TEST(PointCloud, DISABLED_CreatePointCloudFromRGBDImage_3_1) {
    std::vector<Eigen::Vector3d> ref_points = {
            {-0.000337, -0.000252, 0.000553}, {-0.000283, -0.000213, 0.000467},
            {-0.000330, -0.000249, 0.000545}, {-0.000329, -0.000249, 0.000545},
            {-0.000342, -0.000259, 0.000569}, {-0.000260, -0.000194, 0.000427},
            {-0.000276, -0.000207, 0.000455}, {-0.000327, -0.000246, 0.000541},
            {-0.000267, -0.000201, 0.000443}, {-0.000299, -0.000226, 0.000498},
            {-0.000294, -0.000218, 0.000482}, {-0.000312, -0.000232, 0.000514},
            {-0.000280, -0.000209, 0.000463}, {-0.000296, -0.000222, 0.000490},
            {-0.000346, -0.000261, 0.000576}, {-0.000346, -0.000256, 0.000569},
            {-0.000312, -0.000231, 0.000514}, {-0.000320, -0.000238, 0.000529},
            {-0.000253, -0.000189, 0.000420}, {-0.000306, -0.000230, 0.000510},
            {-0.000239, -0.000176, 0.000392}, {-0.000264, -0.000195, 0.000435},
            {-0.000251, -0.000186, 0.000416}, {-0.000331, -0.000246, 0.000549},
            {-0.000252, -0.000188, 0.000420}};

    std::vector<Eigen::Vector3d> ref_colors = {
            {0.737255, 0.615686, 0.721569}, {0.725490, 0.756863, 0.560784},
            {0.600000, 0.717647, 0.584314}, {0.658824, 0.639216, 0.678431},
            {0.607843, 0.647059, 0.768627}, {0.756863, 0.682353, 0.701961},
            {0.545098, 0.674510, 0.513725}, {0.572549, 0.545098, 0.729412},
            {0.549020, 0.619608, 0.545098}, {0.537255, 0.780392, 0.568627},
            {0.647059, 0.737255, 0.674510}, {0.588235, 0.682353, 0.650980},
            {0.643137, 0.776471, 0.588235}, {0.717647, 0.650980, 0.717647},
            {0.619608, 0.752941, 0.584314}, {0.603922, 0.729412, 0.760784},
            {0.525490, 0.768627, 0.650980}, {0.529412, 0.560784, 0.690196},
            {0.752941, 0.603922, 0.525490}, {0.513725, 0.631373, 0.525490},
            {0.572549, 0.772549, 0.756863}, {0.741176, 0.580392, 0.654902},
            {0.611765, 0.713725, 0.647059}, {0.690196, 0.654902, 0.517647},
            {0.627451, 0.764706, 0.764706}};

    int color_num_of_channels = 3;
    int color_bytes_per_channel = 1;

    TEST_CreatePointCloudFromRGBDImage(color_num_of_channels,
                                       color_bytes_per_channel, ref_points,
                                       ref_colors);
}

// ----------------------------------------------------------------------------
// Test CreatePointCloudFromRGBDImage for the following configuration:
// color_num_of_channels = 1
// color_bytes_per_channel = 4
// ----------------------------------------------------------------------------
TEST(PointCloud, DISABLED_CreatePointCloudFromRGBDImage_1_4) {
    std::vector<Eigen::Vector3d> ref_points = {
            {-0.000337, -0.000252, 0.000553}, {-0.000283, -0.000213, 0.000467},
            {-0.000330, -0.000249, 0.000545}, {-0.000329, -0.000249, 0.000545},
            {-0.000342, -0.000259, 0.000569}, {-0.000260, -0.000194, 0.000427},
            {-0.000276, -0.000207, 0.000455}, {-0.000327, -0.000246, 0.000541},
            {-0.000267, -0.000201, 0.000443}, {-0.000299, -0.000226, 0.000498},
            {-0.000294, -0.000218, 0.000482}, {-0.000312, -0.000232, 0.000514},
            {-0.000280, -0.000209, 0.000463}, {-0.000296, -0.000222, 0.000490},
            {-0.000346, -0.000261, 0.000576}, {-0.000346, -0.000256, 0.000569},
            {-0.000312, -0.000231, 0.000514}, {-0.000320, -0.000238, 0.000529},
            {-0.000253, -0.000189, 0.000420}, {-0.000306, -0.000230, 0.000510},
            {-0.000239, -0.000176, 0.000392}, {-0.000264, -0.000195, 0.000435},
            {-0.000251, -0.000186, 0.000416}, {-0.000331, -0.000246, 0.000549},
            {-0.000252, -0.000188, 0.000420}};

    std::vector<Eigen::Vector3d> ref_colors = {
            {-0.000352, -0.000352, -0.000352},
            {-0.000018, -0.000018, -0.000018},
            {-0.000000, -0.000000, -0.000000},
            {-24.580862, -24.580862, -24.580862},
            {-0.000000, -0.000000, -0.000000},
            {-0.001065, -0.001065, -0.001065},
            {-0.000000, -0.000000, -0.000000},
            {-0.020211, -0.020211, -0.020211},
            {-0.000000, -0.000000, -0.000000},
            {-0.000018, -0.000018, -0.000018},
            {-4.959918, -4.959918, -4.959918},
            {-93.301918, -93.301918, -93.301918},
            {-0.000000, -0.000000, -0.000000},
            {-0.000000, -0.000000, -0.000000},
            {-0.000000, -0.000000, -0.000000},
            {-0.094615, -0.094615, -0.094615},
            {-0.000005, -0.000005, -0.000005},
            {-0.000000, -0.000000, -0.000000},
            {-0.000000, -0.000000, -0.000000},
            {-0.000000, -0.000000, -0.000000},
            {-1.254324, -1.254324, -1.254324},
            {-4.550016, -4.550016, -4.550016},
            {-0.000000, -0.000000, -0.000000},
            {-80.370476, -80.370476, -80.370476},
            {-22.216120, -22.216120, -22.216120}};

    int color_num_of_channels = 1;
    int color_bytes_per_channel = 4;

    TEST_CreatePointCloudFromRGBDImage(color_num_of_channels,
                                       color_bytes_per_channel, ref_points,
                                       ref_colors);
}

TEST(PointCloud, DISABLED_SegmentPlane) {
    // Points sampled from the plane x + y + z + 1 = 0
    std::vector<Eigen::Vector3d> ref = {{1.0, 1.0, -3.0},
                                        {2.0, 2.0, -5.0},
                                        {-1.0, -1.0, 1.0},
                                        {-2.0, -2.0, 3.0},
                                        {10.0, 10.0, -21.0}};

    geometry::PointCloud pc;

    for (size_t i = 0; i < ref.size(); i++) {
        pc.points_.emplace_back(ref[i]);
    }

    Eigen::Vector4d plane_model;
    std::vector<size_t> inliers;
    std::tie(plane_model, inliers) = pc.SegmentPlane(0.01, 3, 10);
    auto output_pc = pc.SelectByIndex(inliers);

    ExpectEQ(ref, output_pc->points_);
}

}  // namespace unit_test
}  // namespace open3d
