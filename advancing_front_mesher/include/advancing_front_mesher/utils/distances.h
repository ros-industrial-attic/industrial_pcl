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
 * $Id$
 *
 */

#pragma once

#include <pcl/common/common.h>

/**
 * \file pcl/common/distances.h
 * Define standard C methods to do distance calculations
 * \ingroup common
 */

/*@{*/
namespace industrial_pcl
{
struct PointToLineSegmentDistanceResults
{
  Eigen::Vector3f points[3]; /**< \brief The points. Note: Seg = (points[1] - points[0]) */
  Eigen::Vector3f p;         /**< \brief The closest point on line segment. */
  float mu;                  /**< \brief The corresponding line paramentric value resulting in the minimum distance. */
  double d;                  /**< \brief The distance between point and line segments */
};

/**
 * \brief Get the distance between a point and line
 * \param[in] p1 Origin point of the line
 * \param[in] p2 Terminating point of the line
 * \param[in] p The point for calulating the distance to the line
 * \return DistPoint2LineResults
 */
PCL_EXPORTS inline PointToLineSegmentDistanceResults pointToLineSegmentDistance(const Eigen::Vector3f& p1,
                                                                                const Eigen::Vector3f& p2,
                                                                                const Eigen::Vector3f& p)
{
  PointToLineSegmentDistanceResults results;

  results.points[0] = p1;
  results.points[1] = p2;
  results.points[2] = p;

  Eigen::Vector3f v = results.points[1] - results.points[0];
  Eigen::Vector3f w = p - results.points[0];

  double c1 = w.dot(v);
  double c2 = v.dot(v);

  if (c1 <= 0)
    results.mu = 0.0;
  else if (c2 <= c1)
    results.mu = 1.0;
  else
    results.mu = c1 / c2;

  results.p = results.points[0] + results.mu * v;
  results.d = (p - results.p).norm();

  return results;
}

struct LineSegmentToLineSegmentDistanceResults
{
  Eigen::Vector3f points[4]; /**< \brief The line segment points. Note: Seg1 = (points[1] - points[0]) and Seg2 =
                                (points[3] - points[2]) */
  Eigen::Vector3f p[2]; /**< \brief The closest point on each line segment. Note: p[0] is the closet point on Seg1. */
  float mu[2];          /**< \brief The corresponding line paramentric value resulting in the minimum distance. */
  double d;             /**< \brief The distance between both line segments */
  bool parallel;        /**< \brief Indicate that both lines are parallel. */
};

/**
 * \brief Distance between two line segments
 *
 * Follow the link for more details http://paulbourke.net/geometry/pointlineplane/
 * \param[in] p1 Origin point of the first line
 * \param[in] p2 Terminating point of the first line
 * \param[in] p3 Origin point of the second line
 * \param[in] p4 Terminating point of the second line
 * \return DistLine2LineResults
 */
PCL_EXPORTS inline LineSegmentToLineSegmentDistanceResults lineSegmentToLineSegmentDistance(const Eigen::Vector3f& p1,
                                                                                            const Eigen::Vector3f& p2,
                                                                                            const Eigen::Vector3f& p3,
                                                                                            const Eigen::Vector3f& p4)
{
  LineSegmentToLineSegmentDistanceResults result;
  result.points[0] = p1;
  result.points[1] = p2;
  result.points[2] = p3;
  result.points[3] = p4;

  double d1321 = (p1 - p3).dot((p2 - p1));
  double d2121 = (p2 - p1).dot((p2 - p1));
  double d4321 = (p4 - p3).dot((p2 - p1));
  double d1343 = (p1 - p3).dot((p4 - p3));
  double d4343 = (p4 - p3).dot((p4 - p3));

  double denom = d2121 * d4343 - d4321 * d4321;
  if (denom < 1.0e-8)  // The lines are parallel
  {
    PointToLineSegmentDistanceResults temp, dist;
    result.parallel = true;

    dist = pointToLineSegmentDistance(p1, p2, p3);
    result.mu[0] = dist.mu;

    temp = pointToLineSegmentDistance(p1, p2, p4);
    if (temp.mu < dist.mu)
      result.mu[0] = temp.mu;

    result.p[0] = p1 + result.mu[0] * (p2 - p1);
    temp = pointToLineSegmentDistance(p3, p4, result.p[0]);

    result.mu[1] = temp.mu;
    result.p[1] = p3 + result.mu[1] * (p4 - p3);
  }
  else
  {
    result.mu[0] = (d1343 * d4321 - d1321 * d4343) / denom;
    result.mu[1] = (d1343 + result.mu[0] * d4321) / d4343;

    if (result.mu[0] > 1.0)
      result.mu[0] = 1.0;
    else if (result.mu[0] < 0.0)
      result.mu[0] = 0.0;

    if (result.mu[1] > 1.0)
      result.mu[1] = 1.0;
    else if (result.mu[1] < 0.0)
      result.mu[1] = 0.0;

    result.p[0] = p1 + result.mu[0] * (p2 - p1);
    result.p[1] = p3 + result.mu[1] * (p4 - p3);
  }

  result.d = (result.p[1] - result.p[0]).norm();
  return result;
}
}  // namespace industrial_pcl
