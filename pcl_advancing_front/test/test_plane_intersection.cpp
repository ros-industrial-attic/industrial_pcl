/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: test_plane_intersection.cpp 5686 2012-05-11 20:59:00Z gioia $
 */

#include <pcl_advancing_front/utils/intersections.h>

#include <gtest/gtest.h>
#include <pcl/common/common.h>
#include <pcl/pcl_tests.h>

using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(PCL, lineWithPlaneIntersection)
{
  using pcl::lineWithPlaneIntersection;
  using pcl::LineWithPlaneIntersectionResults;

  Eigen::Vector3f l[2], u, v, origin;
  l[0] << 0.5, 0.5, -0.5;
  l[1] << 0.5, 0.5, 0.5;

  origin << 0.0, 0.0, 0.0;
  u << 1.0, 0.0, 0.0;
  v << 0.0, 1.0, 0.0;

  LineWithPlaneIntersectionResults results = lineWithPlaneIntersection(l[0], l[1], origin, u, v);

  EXPECT_FLOAT_EQ(results.mu, 0.5);
  EXPECT_FLOAT_EQ(results.mv, 0.5);
  EXPECT_FLOAT_EQ(results.mw, 0.5);
  EXPECT_TRUE(results.p.isApprox(Eigen::Vector3f(0.5, 0.5, 0.0), 1e-10));
  EXPECT_FALSE(results.parallel);
}

TEST(PCL, lineWithPlaneIntersectionParallel)
{
  using pcl::lineWithPlaneIntersection;
  using pcl::LineWithPlaneIntersectionResults;

  Eigen::Vector3f l[2], u, v, origin;
  l[0] << 0.0, 0.0, 0.5;
  l[1] << 1.0, 0.0, 0.5;

  origin << 0.0, 0.0, 0.0;
  u << 1.0, 0.0, 0.0;
  v << 0.0, 1.0, 0.0;

  LineWithPlaneIntersectionResults results = lineWithPlaneIntersection(l[0], l[1], origin, u, v);

  EXPECT_TRUE(results.parallel);
}

//* ---[ */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
/* ]--- */
