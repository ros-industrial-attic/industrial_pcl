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

#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>

/**
 * \file pcl/common/intersections.h
 * Define line with line intersection functions
 * \ingroup common
 */

/*@{*/
namespace industrial_pcl
{
struct LineWithPlaneIntersectionResults
{
  Eigen::Vector3f points[2]; /**< \brief The points defining the line. */
  Eigen::Vector3f w;         /**< \brief The vector defining the line direction. */
  Eigen::Vector3f origin;    /**< \brief The origin of the plane. */
  Eigen::Vector3f u;         /**< \brief The vector defining the planes relative u directions. */
  Eigen::Vector3f v;         /**< \brief The vector defining the planes relative v directions. */
  Eigen::Vector3f p; /**< \brief The point of intersection p = origin + x * u + y * v  || p = points[0] + w * lw */
  float mw;          /**< \brief The parametric coeff defining the intersection location on the line. */
  float mu;          /**< \brief The parametric plane u coeff of intersetion. */
  float mv;          /**< \brief The parametric plane v coeff of intersetion. */
  bool parallel;     /**< \brief Indicate whether the line is parallel to the plane. */
};

/**
 * \brief Find the intersection between a plane and line
 * \param[in] p1     The origin point of the line
 * \param[in] p2     The terminating point of the line
 * \param[in] origin The origin of the plane
 * \param[in] u      The vector defining the u direction for the plane
 * \param[in] v      The vector defining the v direction for the plane
 * \return IntersectionLine2PlaneResults
 */
LineWithPlaneIntersectionResults lineWithPlaneIntersection(const Eigen::Vector3f& p1,
                                                           const Eigen::Vector3f& p2,
                                                           const Eigen::Vector3f& origin,
                                                           const Eigen::Vector3f& u,
                                                           const Eigen::Vector3f& v);

}  // namespace industrial_pcl
/*@}*/

#include "impl/intersections.hpp"
