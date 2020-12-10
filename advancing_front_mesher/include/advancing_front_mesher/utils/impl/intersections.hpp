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

#include <pcl/pcl_macros.h>
#include <pcl/console/print.h>

//////////////////////////////////////////////////////////////////////////////////////////

namespace industrial_pcl
{
LineWithPlaneIntersectionResults lineWithPlaneIntersection(const Eigen::Vector3f& p1,
                                                           const Eigen::Vector3f& p2,
                                                           const Eigen::Vector3f& origin,
                                                           const Eigen::Vector3f& u,
                                                           const Eigen::Vector3f& v)
{
  LineWithPlaneIntersectionResults results;
  results.points[0] = p1;
  results.points[1] = p2;
  results.w = p2 - p1;
  results.parallel = false;

  results.origin = origin;
  results.u = u;
  results.v = v;
  Eigen::Vector3f normal = u.cross(v).normalized();

  if (std::abs(normal.dot(results.w.normalized())) < 1.0e-8)
  {
    results.parallel = true;
  }
  else
  {
    Eigen::Matrix3f m;
    m << u, v, -results.w;

    Eigen::Vector3f t = p1 - origin;
    Eigen::Vector3f muvw = m.lu().solve(t);
    results.mu = muvw[0];
    results.mv = muvw[1];
    results.mw = muvw[2];

    results.p = results.points[0] + results.mw * results.w;
  }
  return results;
}
}  // namespace industrial_pcl
