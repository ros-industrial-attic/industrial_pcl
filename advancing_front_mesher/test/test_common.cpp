/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */
#include <advancing_front_mesher/utils/normals.h>
#include <advancing_front_mesher/utils/distances.h>
#include <advancing_front_mesher/utils/intersections.h>

#include <gtest/gtest.h>
#include <pcl/pcl_tests.h>
#include <pcl/common/common.h>
//#include <pcl/common/distances.h>
//#include <pcl/common/intersections.h>
//#include <pcl/common/io.h>
//#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(PCL, Common)
{
  PointXYZ p1, p2, p3;
  p1.x = 1;
  p1.y = p1.z = 0;
  p2.y = 1;
  p2.x = p2.z = 0;
  p3.z = 1;
  p3.x = p3.y = 0;
  double radius = getCircumcircleRadius(p1, p2, p3);
  EXPECT_NEAR(radius, 0.816497, 1e-4);

  Eigen::Vector4f pt(1, 0, 0, 0), line_pt(0, 0, 0, 0), line_dir(1, 1, 0, 0);
  double point2line_disance = sqrt(sqrPointToLineDistance(pt, line_pt, line_dir));
  EXPECT_NEAR(point2line_disance, sqrt(2.0) / 2, 1e-4);

  Eigen::Vector3f n1(0.5, 0.5, 0.0), n1_neg(-0.5, -0.5, 0.0), n2(0.75, 0.5, 0.0);
  EXPECT_FALSE(alignNormals(n1, n2));
  EXPECT_TRUE(n1.isApprox(Eigen::Vector3f(0.5, 0.5, 0.0)));

  EXPECT_TRUE(alignNormals(n1_neg, n2));
  EXPECT_TRUE(n1_neg.isApprox(Eigen::Vector3f(0.5, 0.5, 0.0)));

  EXPECT_TRUE(checkNormalsEqual(n1, n2, 0.2094));
  EXPECT_FALSE(checkNormalsEqual(n1, n2, 0.1920));

  EXPECT_TRUE(checkNormalsEqual(-1.0 * n1, n2, 2.9496));
  EXPECT_FALSE(checkNormalsEqual(-1.0 * n1, n2, 2.9321));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(PCL, pointToLineSegmentDistance)
{
  using pcl::pointToLineSegmentDistance;
  using pcl::PointToLineSegmentDistanceResults;
  Eigen::Vector3f lp1(0.0, 0.0, 0.0);
  Eigen::Vector3f lp2(1.0, 0.0, 0.0);
  Eigen::Vector3f p(0.5, 1.0, 0.0);
  PointToLineSegmentDistanceResults results = pointToLineSegmentDistance(lp1, lp2, p);
  EXPECT_FLOAT_EQ(results.d, 1.0);
  EXPECT_FLOAT_EQ(results.mu, 0.5);
  EXPECT_TRUE(results.p.isApprox(Eigen::Vector3f(0.5, 0.0, 0.0), 1e-10));
}

TEST(PCL, pointToLineSegmentDistanceLeftBound)
{
  using pcl::pointToLineSegmentDistance;
  using pcl::PointToLineSegmentDistanceResults;
  Eigen::Vector3f lp1(0.0, 0.0, 0.0);
  Eigen::Vector3f lp2(1.0, 0.0, 0.0);
  Eigen::Vector3f p(-0.5, 1.0, 0.0);
  PointToLineSegmentDistanceResults results = pointToLineSegmentDistance(lp1, lp2, p);
  EXPECT_FLOAT_EQ(results.d, std::sqrt(1.0 * 1.0 + 0.5 * 0.5));
  EXPECT_FLOAT_EQ(results.mu, 0.0);
  EXPECT_TRUE(results.p.isApprox(lp1, 1e-10));
}

TEST(PCL, pointToLineSegmentDistanceRightBound)
{
  using pcl::pointToLineSegmentDistance;
  using pcl::PointToLineSegmentDistanceResults;
  Eigen::Vector3f lp1(0.0, 0.0, 0.0);
  Eigen::Vector3f lp2(1.0, 0.0, 0.0);
  Eigen::Vector3f p(1.5, 1.0, 0.0);
  PointToLineSegmentDistanceResults results = pointToLineSegmentDistance(lp1, lp2, p);
  EXPECT_FLOAT_EQ(results.d, std::sqrt(1.0 * 1.0 + 0.5 * 0.5));
  EXPECT_FLOAT_EQ(results.mu, 1.0);
  EXPECT_TRUE(results.p.isApprox(lp2, 1e-10));
}

TEST(PCL, lineSegmentToLineSegmentDistance)
{
  using pcl::lineSegmentToLineSegmentDistance;
  using pcl::LineSegmentToLineSegmentDistanceResults;

  Eigen::Vector3f l1[2], l2[2];

  l1[0] << 0.0, 0.0, 0.0;
  l1[1] << 1.0, 0.0, 0.0;

  l2[0] << 0.0, -0.5, 0.5;
  l2[1] << 0.0, 0.5, 0.5;

  LineSegmentToLineSegmentDistanceResults results = lineSegmentToLineSegmentDistance(l1[0], l1[1], l2[0], l2[1]);
  EXPECT_FLOAT_EQ(results.mu[0], 0.0);
  EXPECT_FLOAT_EQ(results.mu[1], 0.5);

  EXPECT_TRUE(results.p[0].isApprox(l1[0], 1e-10));
  EXPECT_TRUE(results.p[1].isApprox(Eigen::Vector3f(0.0, 0.0, 0.5), 1e-10));
  EXPECT_FALSE(results.parallel);
}

TEST(PCL, lineSegmentToLineSegmentDistanceParallel)
{
  using pcl::lineSegmentToLineSegmentDistance;
  using pcl::LineSegmentToLineSegmentDistanceResults;

  Eigen::Vector3f l1[2], l2[2];

  l1[0] << 0.0, 0.0, 0.0;
  l1[1] << 1.0, 0.0, 0.0;

  l2[0] << -0.5, 0.0, 0.5;
  l2[1] << 0.5, 0.0, 0.5;

  LineSegmentToLineSegmentDistanceResults results = lineSegmentToLineSegmentDistance(l1[0], l1[1], l2[0], l2[1]);
  EXPECT_FLOAT_EQ(results.mu[0], 0.0);
  EXPECT_FLOAT_EQ(results.mu[1], 0.5);

  EXPECT_TRUE(results.p[0].isApprox(l1[0], 1e-10));
  EXPECT_TRUE(results.p[1].isApprox(Eigen::Vector3f(0.0, 0.0, 0.5), 1e-10));
  EXPECT_TRUE(results.parallel);
}

/* ---[ */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
/* ]--- */
