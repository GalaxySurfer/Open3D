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
#include "TestUtility/UnitTest.h"

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
    std::vector<Eigen::Vector3d> points = {
            {1, 10, 20}, {30, 2, 40}, {50, 60, 3}};
    geometry::PointCloud pc(points);
    ExpectEQ(pc.GetMinBound(), Eigen::Vector3d(1, 2, 3));

    geometry::PointCloud pc_empty;
    ExpectEQ(pc_empty.GetMinBound(), Eigen::Vector3d(0, 0, 0));
}

TEST(PointCloud, GetMaxBound) {
    std::vector<Eigen::Vector3d> points = {
            {1, 10, 20}, {30, 2, 40}, {50, 60, 3}};
    geometry::PointCloud pc(points);
    ExpectEQ(pc.GetMaxBound(), Eigen::Vector3d(50, 60, 40));

    geometry::PointCloud pc_empty;
    ExpectEQ(pc_empty.GetMaxBound(), Eigen::Vector3d(0, 0, 0));
}

TEST(PointCloud, GetCenter) {
    std::vector<Eigen::Vector3d> points = {
            {0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {9, 10, 11}};
    geometry::PointCloud pc(points);
    ExpectEQ(pc.GetCenter(), Eigen::Vector3d(4.5, 5.5, 6.5));

    geometry::PointCloud pc_empty;
    ExpectEQ(pc_empty.GetCenter(), Eigen::Vector3d(0, 0, 0));
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

TEST(PointCloud, DISABLED_HasPoints) {
    int size = 100;

    geometry::PointCloud pc;

    EXPECT_FALSE(pc.HasPoints());

    pc.points_.resize(size);

    EXPECT_TRUE(pc.HasPoints());
}

TEST(PointCloud, DISABLED_HasNormals) {
    int size = 100;

    geometry::PointCloud pc;

    EXPECT_FALSE(pc.HasNormals());

    pc.points_.resize(size);
    pc.normals_.resize(size);

    EXPECT_TRUE(pc.HasNormals());
}

TEST(PointCloud, DISABLED_HasColors) {
    int size = 100;

    geometry::PointCloud pc;

    EXPECT_FALSE(pc.HasColors());

    pc.points_.resize(size);
    pc.colors_.resize(size);

    EXPECT_TRUE(pc.HasColors());
}

TEST(PointCloud, DISABLED_NormalizeNormals) {
    std::vector<Eigen::Vector3d> ref = {
            {0.692861, 0.323767, 0.644296}, {0.650010, 0.742869, 0.160101},
            {0.379563, 0.870761, 0.312581}, {0.575046, 0.493479, 0.652534},
            {0.320665, 0.448241, 0.834418}, {0.691127, 0.480526, 0.539850},
            {0.227557, 0.973437, 0.025284}, {0.281666, 0.156994, 0.946582},
            {0.341869, 0.894118, 0.289273}, {0.103335, 0.972118, 0.210498},
            {0.441745, 0.723783, 0.530094}, {0.336903, 0.727710, 0.597441},
            {0.434917, 0.862876, 0.257471}, {0.636619, 0.435239, 0.636619},
            {0.393717, 0.876213, 0.277918}, {0.275051, 0.633543, 0.723167},
            {0.061340, 0.873191, 0.483503}, {0.118504, 0.276510, 0.953677},
            {0.930383, 0.360677, 0.065578}, {0.042660, 0.989719, 0.136513}};

    int size = 20;

    Eigen::Vector3d vmin(0.0, 0.0, 0.0);
    Eigen::Vector3d vmax(1000.0, 1000.0, 1000.0);

    geometry::PointCloud pc;

    pc.normals_.resize(size);

    Rand(pc.normals_, vmin, vmax, 0);

    pc.NormalizeNormals();

    ExpectEQ(ref, pc.normals_);
}

TEST(PointCloud, DISABLED_PaintUniformColor) {
    size_t size = 100;

    Eigen::Vector3d vmin(0.0, 0.0, 0.0);
    Eigen::Vector3d vmax(1000.0, 1000.0, 1000.0);

    geometry::PointCloud pc;

    EXPECT_TRUE(pc.IsEmpty());

    pc.points_.resize(size);

    Rand(pc.points_, vmin, vmax, 0);

    EXPECT_FALSE(pc.HasColors());

    Eigen::Vector3d color(233. / 255., 171. / 255., 53.0 / 255.);
    pc.PaintUniformColor(color);

    EXPECT_TRUE(pc.HasColors());

    for (size_t i = 0; i < pc.colors_.size(); i++)
        ExpectEQ(color, pc.colors_[i]);
}

TEST(PointCloud, DISABLED_OperatorAppend) {
    size_t size = 100;

    geometry::PointCloud pc0;
    geometry::PointCloud pc1;

    pc0.points_.resize(size);
    pc0.normals_.resize(size);
    pc0.colors_.resize(size);

    pc1.points_.resize(size);
    pc1.normals_.resize(size);
    pc1.colors_.resize(size);

    Rand(pc0.points_, Zero3d, Eigen::Vector3d(1000.0, 1000.0, 1000.0), 0);
    Rand(pc0.normals_, Eigen::Vector3d(-1.0, -1.0, -1.0),
         Eigen::Vector3d(1.0, 1.0, 1.0), 0);
    Rand(pc0.colors_, Zero3d, Eigen::Vector3d(1.0, 1.0, 1.0), 0);

    Rand(pc1.points_, Zero3d, Eigen::Vector3d(1000.0, 1000.0, 1000.0), 0);
    Rand(pc1.normals_, Eigen::Vector3d(-1.0, -1.0, -1.0),
         Eigen::Vector3d(1.0, 1.0, 1.0), 0);
    Rand(pc1.colors_, Zero3d, Eigen::Vector3d(1.0, 1.0, 1.0), 1);

    std::vector<Eigen::Vector3d> p;
    p.insert(p.end(), pc0.points_.begin(), pc0.points_.end());
    p.insert(p.end(), pc1.points_.begin(), pc1.points_.end());

    std::vector<Eigen::Vector3d> n;
    n.insert(n.end(), pc0.normals_.begin(), pc0.normals_.end());
    n.insert(n.end(), pc1.normals_.begin(), pc1.normals_.end());

    std::vector<Eigen::Vector3d> c;
    c.insert(c.end(), pc0.colors_.begin(), pc0.colors_.end());
    c.insert(c.end(), pc1.colors_.begin(), pc1.colors_.end());

    geometry::PointCloud pc(pc0);
    pc += pc1;

    EXPECT_EQ(2 * size, pc.points_.size());
    for (size_t i = 0; i < size; i++) {
        ExpectEQ(pc0.points_[i], pc.points_[0 + i]);
        ExpectEQ(pc1.points_[i], pc.points_[size + i]);
    }

    EXPECT_EQ(2 * size, pc.normals_.size());
    for (size_t i = 0; i < size; i++) {
        ExpectEQ(pc0.normals_[i], pc.normals_[0 + i]);
        ExpectEQ(pc1.normals_[i], pc.normals_[size + i]);
    }

    EXPECT_EQ(2 * size, pc.colors_.size());
    for (size_t i = 0; i < size; i++) {
        ExpectEQ(pc0.colors_[i], pc.colors_[0 + i]);
        ExpectEQ(pc1.colors_[i], pc.colors_[size + i]);
    }
}

TEST(PointCloud, DISABLED_OperatorADD) {
    size_t size = 100;

    geometry::PointCloud pc0;
    geometry::PointCloud pc1;

    pc0.points_.resize(size);
    pc0.normals_.resize(size);
    pc0.colors_.resize(size);

    pc1.points_.resize(size);
    pc1.normals_.resize(size);
    pc1.colors_.resize(size);

    Rand(pc0.points_, Zero3d, Eigen::Vector3d(1000.0, 1000.0, 1000.0), 0);
    Rand(pc0.normals_, Eigen::Vector3d(-1.0, -1.0, -1.0),
         Eigen::Vector3d(1.0, 1.0, 1.0), 0);
    Rand(pc0.colors_, Zero3d, Eigen::Vector3d(1.0, 1.0, 1.0), 0);

    Rand(pc1.points_, Zero3d, Eigen::Vector3d(1000.0, 1000.0, 1000.0), 0);
    Rand(pc1.normals_, Eigen::Vector3d(-1.0, -1.0, -1.0),
         Eigen::Vector3d(1.0, 1.0, 1.0), 0);
    Rand(pc1.colors_, Zero3d, Eigen::Vector3d(1.0, 1.0, 1.0), 1);

    std::vector<Eigen::Vector3d> p;
    p.insert(p.end(), pc0.points_.begin(), pc0.points_.end());
    p.insert(p.end(), pc1.points_.begin(), pc1.points_.end());

    std::vector<Eigen::Vector3d> n;
    n.insert(n.end(), pc0.normals_.begin(), pc0.normals_.end());
    n.insert(n.end(), pc1.normals_.begin(), pc1.normals_.end());

    std::vector<Eigen::Vector3d> c;
    c.insert(c.end(), pc0.colors_.begin(), pc0.colors_.end());
    c.insert(c.end(), pc1.colors_.begin(), pc1.colors_.end());

    geometry::PointCloud pc = pc0 + pc1;

    EXPECT_EQ(2 * size, pc.points_.size());
    for (size_t i = 0; i < size; i++) {
        ExpectEQ(pc0.points_[i], pc.points_[0 + i]);
        ExpectEQ(pc1.points_[i], pc.points_[size + i]);
    }

    EXPECT_EQ(2 * size, pc.normals_.size());
    for (size_t i = 0; i < size; i++) {
        ExpectEQ(pc0.normals_[i], pc.normals_[0 + i]);
        ExpectEQ(pc1.normals_[i], pc.normals_[size + i]);
    }

    EXPECT_EQ(2 * size, pc.colors_.size());
    for (size_t i = 0; i < size; i++) {
        ExpectEQ(pc0.colors_[i], pc.colors_[0 + i]);
        ExpectEQ(pc1.colors_[i], pc.colors_[size + i]);
    }
}

TEST(PointCloud, DISABLED_DISABLED_CreatePointCloudFromFile) {
    unit_test::NotImplemented();
}

TEST(PointCloud, DISABLED_SelectByIndex) {
    std::vector<Eigen::Vector3d> ref = {{796.078431, 909.803922, 196.078431},
                                        {768.627451, 525.490196, 768.627451},
                                        {400.000000, 890.196078, 282.352941},
                                        {349.019608, 803.921569, 917.647059},
                                        {19.607843, 454.901961, 62.745098},
                                        {666.666667, 529.411765, 39.215686},
                                        {164.705882, 439.215686, 878.431373},
                                        {909.803922, 482.352941, 215.686275},
                                        {615.686275, 278.431373, 784.313725},
                                        {415.686275, 168.627451, 905.882353},
                                        {949.019608, 50.980392, 517.647059},
                                        {639.215686, 756.862745, 90.196078},
                                        {203.921569, 886.274510, 121.568627},
                                        {356.862745, 549.019608, 576.470588},
                                        {529.411765, 756.862745, 301.960784},
                                        {992.156863, 576.470588, 874.509804},
                                        {227.450980, 698.039216, 313.725490},
                                        {470.588235, 592.156863, 941.176471},
                                        {431.372549, 0.000000, 341.176471},
                                        {596.078431, 831.372549, 231.372549},
                                        {674.509804, 482.352941, 478.431373},
                                        {694.117647, 670.588235, 635.294118},
                                        {109.803922, 360.784314, 576.470588},
                                        {592.156863, 662.745098, 286.274510},
                                        {823.529412, 329.411765, 184.313725}};

    size_t size = 100;
    geometry::PointCloud pc;

    Eigen::Vector3d vmin(0.0, 0.0, 0.0);
    Eigen::Vector3d vmax(1000.0, 1000.0, 1000.0);

    pc.points_.resize(size);
    Rand(pc.points_, vmin, vmax, 0);

    std::vector<size_t> indices(size / 4);
    Rand(indices, 0, size, 0);

    // remove duplicates
    std::vector<size_t>::iterator it;
    it = unique(indices.begin(), indices.end());
    indices.resize(distance(indices.begin(), it));

    auto output_pc = pc.SelectByIndex(indices);

    ExpectEQ(ref, output_pc->points_);
}

TEST(PointCloud, DISABLED_VoxelDownSample) {
    std::vector<Eigen::Vector3d> ref_points = {
            {19.607843, 454.901961, 62.745098},
            {66.666667, 949.019608, 525.490196},
            {82.352941, 192.156863, 662.745098},
            {105.882353, 996.078431, 215.686275},
            {141.176471, 603.921569, 15.686275},
            {152.941176, 400.000000, 129.411765},
            {239.215686, 133.333333, 803.921569},
            {294.117647, 635.294118, 521.568627},
            {333.333333, 764.705882, 274.509804},
            {349.019608, 803.921569, 917.647059},
            {364.705882, 509.803922, 949.019608},
            {400.000000, 890.196078, 282.352941},
            {490.196078, 972.549020, 290.196078},
            {509.803922, 835.294118, 611.764706},
            {552.941176, 474.509804, 627.450980},
            {768.627451, 525.490196, 768.627451},
            {796.078431, 909.803922, 196.078431},
            {839.215686, 392.156863, 780.392157},
            {890.196078, 345.098039, 62.745098},
            {913.725490, 635.294118, 713.725490}};

    std::vector<Eigen::Vector3d> ref_normals = {
            {0.042660, 0.989719, 0.136513}, {0.061340, 0.873191, 0.483503},
            {0.103335, 0.972118, 0.210498}, {0.118504, 0.276510, 0.953677},
            {0.227557, 0.973437, 0.025284}, {0.275051, 0.633543, 0.723167},
            {0.281666, 0.156994, 0.946582}, {0.320665, 0.448241, 0.834418},
            {0.336903, 0.727710, 0.597441}, {0.341869, 0.894118, 0.289273},
            {0.379563, 0.870761, 0.312581}, {0.393717, 0.876213, 0.277918},
            {0.434917, 0.862876, 0.257471}, {0.441745, 0.723783, 0.530094},
            {0.575046, 0.493479, 0.652534}, {0.636619, 0.435239, 0.636619},
            {0.650010, 0.742869, 0.160101}, {0.691127, 0.480526, 0.539850},
            {0.692861, 0.323767, 0.644296}, {0.930383, 0.360677, 0.065578}};

    std::vector<Eigen::Vector3d> ref_colors = {
            {5.000000, 116.000000, 16.000000},
            {17.000000, 242.000000, 134.000000},
            {21.000000, 49.000000, 169.000000},
            {27.000000, 254.000000, 55.000000},
            {36.000000, 154.000000, 4.000000},
            {39.000000, 102.000000, 33.000000},
            {61.000000, 34.000000, 205.000000},
            {75.000000, 162.000000, 133.000000},
            {85.000000, 195.000000, 70.000000},
            {89.000000, 205.000000, 234.000000},
            {93.000000, 130.000000, 242.000000},
            {102.000000, 227.000000, 72.000000},
            {125.000000, 248.000000, 74.000000},
            {130.000000, 213.000000, 156.000000},
            {141.000000, 121.000000, 160.000000},
            {196.000000, 134.000000, 196.000000},
            {203.000000, 232.000000, 50.000000},
            {214.000000, 100.000000, 199.000000},
            {227.000000, 88.000000, 16.000000},
            {233.000000, 162.000000, 182.000000}};

    size_t size = 20;
    geometry::PointCloud pc;

    pc.points_.resize(size);
    pc.normals_.resize(size);
    pc.colors_.resize(size);

    Rand(pc.points_, Zero3d, Eigen::Vector3d(1000.0, 1000.0, 1000.0), 0);
    Rand(pc.normals_, Zero3d, Eigen::Vector3d(10.0, 10.0, 10.0), 0);
    Rand(pc.colors_, Zero3d, Eigen::Vector3d(255.0, 255.0, 255.0), 0);

    double voxel_size = 0.5;
    auto output_pc = pc.VoxelDownSample(voxel_size);

    // sometimes the order of these Eigen::Vector3d values can be mixed-up
    // sort these vectors in order to match the expected order.
    Sort::Do(output_pc->points_);
    Sort::Do(output_pc->normals_);
    Sort::Do(output_pc->colors_);

    ExpectEQ(ref_points, output_pc->points_);
    ExpectEQ(ref_normals, output_pc->normals_);
    ExpectEQ(ref_colors, output_pc->colors_);
}

TEST(PointCloud, DISABLED_UniformDownSample) {
    std::vector<Eigen::Vector3d> ref = {{839.215686, 392.156863, 780.392157},
                                        {364.705882, 509.803922, 949.019608},
                                        {152.941176, 400.000000, 129.411765},
                                        {490.196078, 972.549020, 290.196078},
                                        {66.666667, 949.019608, 525.490196},
                                        {235.294118, 968.627451, 901.960784},
                                        {435.294118, 929.411765, 929.411765},
                                        {827.450980, 329.411765, 227.450980},
                                        {396.078431, 811.764706, 682.352941},
                                        {615.686275, 278.431373, 784.313725},
                                        {101.960784, 125.490196, 494.117647},
                                        {584.313725, 243.137255, 149.019608},
                                        {172.549020, 239.215686, 796.078431},
                                        {66.666667, 203.921569, 458.823529},
                                        {996.078431, 50.980392, 866.666667},
                                        {356.862745, 549.019608, 576.470588},
                                        {745.098039, 627.450980, 35.294118},
                                        {666.666667, 494.117647, 160.784314},
                                        {325.490196, 231.372549, 70.588235},
                                        {470.588235, 592.156863, 941.176471},
                                        {674.509804, 482.352941, 478.431373},
                                        {345.098039, 184.313725, 607.843137},
                                        {529.411765, 86.274510, 258.823529},
                                        {772.549020, 286.274510, 329.411765},
                                        {764.705882, 698.039216, 117.647059}};

    size_t size = 100;
    geometry::PointCloud pc;

    Eigen::Vector3d vmin(0.0, 0.0, 0.0);
    Eigen::Vector3d vmax(1000.0, 1000.0, 1000.0);

    pc.points_.resize(size);
    Rand(pc.points_, vmin, vmax, 0);

    size_t every_k_points = 4;
    auto output_pc = pc.UniformDownSample(every_k_points);

    ExpectEQ(ref, output_pc->points_);
}

TEST(PointCloud, DISABLED_CropPointCloud) {
    size_t size = 100;
    geometry::PointCloud pc;

    Eigen::Vector3d vmin(0.0, 0.0, 0.0);
    Eigen::Vector3d vmax(1000.0, 1000.0, 1000.0);

    pc.points_.resize(size);
    Rand(pc.points_, vmin, vmax, 0);

    Eigen::Vector3d minBound(200.0, 200.0, 200.0);
    Eigen::Vector3d maxBound(800.0, 800.0, 800.0);
    auto output_pc =
            pc.Crop(geometry::AxisAlignedBoundingBox(minBound, maxBound));

    ExpectLE(minBound, output_pc->points_);
    ExpectGE(maxBound, output_pc->points_);
}

TEST(PointCloud, DISABLED_EstimateNormals) {
    std::vector<Eigen::Vector3d> ref = {
            {0.282003, 0.866394, 0.412111},   {0.550791, 0.829572, -0.091869},
            {0.076085, -0.974168, 0.212620},  {0.261265, 0.825182, 0.500814},
            {0.035397, 0.428362, 0.902913},   {0.711421, 0.595291, 0.373508},
            {0.519141, 0.552592, 0.652024},   {0.490520, 0.573293, -0.656297},
            {0.324029, 0.744177, 0.584128},   {0.120589, -0.989854, 0.075152},
            {0.370700, 0.767066, 0.523632},   {0.874692, -0.158725, -0.457952},
            {0.238700, 0.937064, -0.254819},  {0.518237, 0.540189, 0.663043},
            {0.238700, 0.937064, -0.254819},  {0.080943, -0.502095, -0.861016},
            {0.753661, -0.527376, -0.392261}, {0.721099, 0.542859, -0.430489},
            {0.159997, -0.857801, -0.488446}, {0.445869, 0.725107, 0.524805},
            {0.019474, -0.592041, -0.805672}, {0.024464, 0.856206, 0.516056},
            {0.478041, 0.869593, -0.123631},  {0.104534, -0.784980, -0.610638},
            {0.073901, 0.570353, 0.818069},   {0.178678, 0.974506, 0.135693},
            {0.178678, 0.974506, 0.135693},   {0.581675, 0.167795, -0.795926},
            {0.069588, -0.845043, -0.530150}, {0.626448, 0.486534, 0.608973},
            {0.670665, 0.657002, 0.344321},   {0.588868, 0.011829, 0.808143},
            {0.081974, 0.638039, 0.765628},   {0.159997, -0.857801, -0.488446},
            {0.559499, 0.824271, -0.086826},  {0.612885, 0.727999, 0.307229},
            {0.178678, 0.974506, 0.135693},   {0.268803, 0.796616, 0.541431},
            {0.604933, 0.787776, -0.116044},  {0.111998, 0.869999, -0.480165}};

    size_t size = 40;
    geometry::PointCloud pc;

    Eigen::Vector3d vmin(0.0, 0.0, 0.0);
    Eigen::Vector3d vmax(1000.0, 1000.0, 1000.0);

    pc.points_.resize(size);
    Rand(pc.points_, vmin, vmax, 0);

    pc.EstimateNormals(geometry::KDTreeSearchParamKNN(), true);
    for (size_t idx = 0; idx < ref.size(); ++idx) {
        if ((ref[idx](0) < 0 && pc.normals_[idx](0) > 0) ||
            (ref[idx](0) > 0 && pc.normals_[idx](0) < 0)) {
            pc.normals_[idx] *= -1;
        }
    }
    ExpectEQ(ref, pc.normals_);

    pc.EstimateNormals(geometry::KDTreeSearchParamKNN(), false);
    for (size_t idx = 0; idx < ref.size(); ++idx) {
        if ((ref[idx](0) < 0 && pc.normals_[idx](0) > 0) ||
            (ref[idx](0) > 0 && pc.normals_[idx](0) < 0)) {
            pc.normals_[idx] *= -1;
        }
    }
    ExpectEQ(ref, pc.normals_);
}

TEST(PointCloud, DISABLED_OrientNormalsToAlignWithDirection) {
    std::vector<Eigen::Vector3d> ref = {
            {0.282003, 0.866394, 0.412111},   {0.550791, 0.829572, -0.091869},
            {0.076085, -0.974168, 0.212620},  {0.261265, 0.825182, 0.500814},
            {0.035397, 0.428362, 0.902913},   {0.711421, 0.595291, 0.373508},
            {0.519141, 0.552592, 0.652024},   {-0.490520, -0.573293, 0.656297},
            {0.324029, 0.744177, 0.584128},   {-0.120589, 0.989854, -0.075152},
            {0.370700, 0.767066, 0.523632},   {-0.874692, 0.158725, 0.457952},
            {-0.238700, -0.937064, 0.254819}, {0.518237, 0.540189, 0.663043},
            {-0.238700, -0.937064, 0.254819}, {-0.080943, 0.502095, 0.861016},
            {-0.753661, 0.527376, 0.392261},  {-0.721099, -0.542859, 0.430489},
            {-0.159997, 0.857801, 0.488446},  {0.445869, 0.725107, 0.524805},
            {-0.019474, 0.592041, 0.805672},  {0.024464, 0.856206, 0.516056},
            {0.478041, 0.869593, -0.123631},  {-0.104534, 0.784980, 0.610638},
            {0.073901, 0.570353, 0.818069},   {0.178678, 0.974506, 0.135693},
            {0.178678, 0.974506, 0.135693},   {-0.581675, -0.167795, 0.795926},
            {-0.069588, 0.845043, 0.530150},  {0.626448, 0.486534, 0.608973},
            {0.670665, 0.657002, 0.344321},   {0.588868, 0.011829, 0.808143},
            {0.081974, 0.638039, 0.765628},   {-0.159997, 0.857801, 0.488446},
            {0.559499, 0.824271, -0.086826},  {0.612885, 0.727999, 0.307229},
            {0.178678, 0.974506, 0.135693},   {0.268803, 0.796616, 0.541431},
            {0.604933, 0.787776, -0.116044},  {-0.111998, -0.869999, 0.480165}};

    int size = 40;
    geometry::PointCloud pc;

    Eigen::Vector3d vmin(0.0, 0.0, 0.0);
    Eigen::Vector3d vmax(1000.0, 1000.0, 1000.0);

    pc.points_.resize(size);
    Rand(pc.points_, vmin, vmax, 0);

    pc.EstimateNormals();
    pc.OrientNormalsToAlignWithDirection(Eigen::Vector3d(1.5, 0.5, 3.3));

    ExpectEQ(ref, pc.normals_);
}

TEST(PointCloud, DISABLED_OrientNormalsTowardsCameraLocation) {
    std::vector<Eigen::Vector3d> ref = {{-0.282003, -0.866394, -0.412111},
                                        {-0.550791, -0.829572, 0.091869},
                                        {0.076085, -0.974168, 0.212620},
                                        {-0.261265, -0.825182, -0.500814},
                                        {-0.035397, -0.428362, -0.902913},
                                        {-0.711421, -0.595291, -0.373508},
                                        {-0.519141, -0.552592, -0.652024},
                                        {0.490520, 0.573293, -0.656297},
                                        {-0.324029, -0.744177, -0.584128},
                                        {0.120589, -0.989854, 0.075152},
                                        {-0.370700, -0.767066, -0.523632},
                                        {0.874692, -0.158725, -0.457952},
                                        {-0.238700, -0.937064, 0.254819},
                                        {-0.518237, -0.540189, -0.663043},
                                        {-0.238700, -0.937064, 0.254819},
                                        {0.080943, -0.502095, -0.861016},
                                        {0.753661, -0.527376, -0.392261},
                                        {0.721099, 0.542859, -0.430489},
                                        {0.159997, -0.857801, -0.488446},
                                        {-0.445869, -0.725107, -0.524805},
                                        {0.019474, -0.592041, -0.805672},
                                        {-0.024464, -0.856206, -0.516056},
                                        {-0.478041, -0.869593, 0.123631},
                                        {0.104534, -0.784980, -0.610638},
                                        {-0.073901, -0.570353, -0.818069},
                                        {-0.178678, -0.974506, -0.135693},
                                        {-0.178678, -0.974506, -0.135693},
                                        {0.581675, 0.167795, -0.795926},
                                        {0.069588, -0.845043, -0.530150},
                                        {-0.626448, -0.486534, -0.608973},
                                        {-0.670665, -0.657002, -0.344321},
                                        {-0.588868, -0.011829, -0.808143},
                                        {-0.081974, -0.638039, -0.765628},
                                        {0.159997, -0.857801, -0.488446},
                                        {-0.559499, -0.824271, 0.086826},
                                        {-0.612885, -0.727999, -0.307229},
                                        {-0.178678, -0.974506, -0.135693},
                                        {-0.268803, -0.796616, -0.541431},
                                        {-0.604933, -0.787776, 0.116044},
                                        {0.111998, 0.869999, -0.480165}};

    int size = 40;
    geometry::PointCloud pc;

    Eigen::Vector3d vmin(0.0, 0.0, 0.0);
    Eigen::Vector3d vmax(1000.0, 1000.0, 1000.0);

    pc.points_.resize(size);
    Rand(pc.points_, vmin, vmax, 0);

    pc.EstimateNormals();
    pc.OrientNormalsTowardsCameraLocation(Eigen::Vector3d(1.5, 0.5, 3.3));

    ExpectEQ(ref, pc.normals_);
}

TEST(PointCloud, DISABLED_ComputePointCloudToPointCloudDistance) {
    std::vector<double> ref = {
            157.498711, 127.737235, 113.386920, 192.476725, 134.367386,
            119.720294, 104.713960, 228.597516, 131.299365, 174.718976,
            248.300645, 119.976930, 23.200313,  71.130812,  134.995408,
            149.534713, 206.804657, 191.395049, 139.532917, 130.417954,
            183.393615, 219.957705, 179.623255, 125.612685, 29.865777,
            110.013805, 100.287936, 303.180627, 43.315141,  227.214211,
            166.239360, 199.730691, 168.855295, 178.377497, 144.034256,
            261.542463, 122.388130, 239.857705, 116.398605, 177.686443,
            92.966820,  96.138437,  23.529412,  177.902686, 68.149597,
            148.191715, 158.520650, 346.210329, 248.114768, 182.342399};

    int size = 100;

    geometry::PointCloud pc0;
    geometry::PointCloud pc1;

    Eigen::Vector3d vmin(0.0, 0.0, 0.0);
    Eigen::Vector3d vmax(1000.0, 1000.0, 1000.0);

    std::vector<Eigen::Vector3d> points(size);
    Rand(points, vmin, vmax, 0);

    for (int i = 0; i < (size / 2); i++) {
        pc0.points_.push_back(points[0 + i]);
        pc1.points_.push_back(points[(size / 2) + i]);
    }

    std::vector<double> distance = pc0.ComputePointCloudDistance(pc1);

    ExpectEQ(ref, distance);
}

TEST(PointCloud, DISABLED_ComputePointCloudMeanAndCovariance) {
    int size = 40;
    geometry::PointCloud pc;

    Eigen::Vector3d vmin(0.0, 0.0, 0.0);
    Eigen::Vector3d vmax(1000.0, 1000.0, 1000.0);

    pc.points_.resize(size);
    Rand(pc.points_, vmin, vmax, 0);

    auto output = pc.ComputeMeanAndCovariance();

    Eigen::Vector3d mean = std::get<0>(output);
    Eigen::Matrix3d covariance = std::get<1>(output);

    ExpectEQ(Eigen::Vector3d(514.215686, 566.666666, 526.568627), mean);

    Eigen::Matrix3d ref_covariance;
    ref_covariance << 86747.549019, -9480.776624, 1416.234140, -9480.776624,
            64536.716647, -12861.399461, 1416.234140, -12861.399461,
            85923.096885;

    ExpectEQ(ref_covariance, covariance);
}

TEST(PointCloud, DISABLED_ComputePointCloudMahalanobisDistance) {
    std::vector<double> ref = {
            1.439881, 1.872615, 1.232338, 0.437462, 1.617472, 1.556793,
            2.019575, 1.984814, 1.845557, 2.340981, 1.177214, 0.960687,
            1.629667, 1.165109, 1.443769, 1.926848, 2.299014, 2.096105,
            2.208119, 2.279263, 2.473191, 1.520355, 0.999211, 1.627128,
            2.151709, 1.274612, 0.898971, 1.899552, 1.705108, 1.527357,
            1.594832, 1.724115, 1.337456, 1.771533, 2.323719, 1.352616,
            1.234500, 1.233509, 1.568047, 1.804558, 2.123548, 2.340709,
            1.036468, 1.387644, 1.668290, 1.780892, 1.934451, 2.334212,
            1.918557, 1.701092, 1.544488, 1.918934, 2.054735, 1.286736,
            2.717411, 1.984941, 2.560241, 2.818804, 1.824099, 1.899621,
            0.710997, 1.358864, 0.957766, 1.996037, 1.761726, 1.921903,
            2.229389, 2.221731, 1.281107, 2.112245, 1.135575, 1.329718,
            1.957294, 1.208041, 1.624905, 1.358104, 1.500636, 1.311195,
            2.056985, 1.318757, 0.579100, 1.370243, 1.871690, 0.875355,
            1.431115, 0.921470, 1.807282, 1.054253, 1.871363, 1.939949,
            1.681727, 0.831352, 1.466078, 2.420450, 1.787704, 2.197718,
            1.620004, 1.087840, 2.319767, 1.569915};

    int size = 100;

    geometry::PointCloud pc;

    Eigen::Vector3d vmin(0.0, 0.0, 0.0);
    Eigen::Vector3d vmax(1000.0, 1000.0, 1000.0);

    pc.points_.resize(size);
    Rand(pc.points_, vmin, vmax, 0);

    std::vector<double> distance = pc.ComputeMahalanobisDistance();

    ExpectEQ(ref, distance);
}

TEST(PointCloud, DISABLED_ComputePointCloudNearestNeighborDistance) {
    std::vector<double> ref = {
            115.403443, 127.737235, 113.386920, 160.257386, 134.367386,
            84.927090,  104.713960, 125.367587, 131.299365, 174.718976,
            135.903713, 119.976930, 23.200313,  71.130812,  122.388130,
            149.534713, 206.804657, 167.712979, 139.532917, 130.417954,
            183.393615, 177.513260, 145.151024, 125.612685, 29.865777,
            104.860721, 80.940264,  216.433643, 43.315141,  115.403443,
            84.927090,  149.637522, 135.903713, 174.101748, 144.034256,
            241.072830, 112.365088, 161.880529, 116.398605, 177.686443,
            92.966820,  96.138437,  23.529412,  161.880529, 68.149597,
            148.191715, 150.610701, 235.849019, 125.367587, 182.342399,
            103.829038, 104.713960, 92.966820,  71.130812,  171.251808,
            156.960754, 285.736804, 171.251808, 68.149597,  118.493686,
            119.976930, 99.903837,  100.823217, 188.316976, 110.223289,
            91.968936,  137.030629, 91.968936,  126.588332, 127.737235,
            36.787575,  131.123557, 177.902686, 36.787575,  23.200313,
            99.903837,  134.367386, 177.686443, 154.392311, 122.388130,
            192.476725, 113.386920, 183.686877, 197.990675, 192.356839,
            83.465869,  148.191715, 96.218385,  147.254771, 114.265116,
            116.398605, 83.465869,  123.576088, 156.960754, 43.315141,
            29.865777,  110.223289, 23.529412,  162.070418, 179.623255};

    int size = 100;

    geometry::PointCloud pc;

    Eigen::Vector3d vmin(0.0, 0.0, 0.0);
    Eigen::Vector3d vmax(1000.0, 1000.0, 1000.0);

    pc.points_.resize(size);
    Rand(pc.points_, vmin, vmax, 0);

    std::vector<double> distance = pc.ComputeNearestNeighborDistance();

    ExpectEQ(ref, distance);
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
