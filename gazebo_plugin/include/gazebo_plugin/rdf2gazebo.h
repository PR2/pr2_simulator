/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/
#ifndef URDF2GAZEBO_HH
#define URDF2GAZEBO_HH

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <vector>
#include <string>

#include <sstream>

#include <rdf_parser/rdf.h>

#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"

namespace urdf2gazebo
{
  double rad2deg(double v) { return v * 180.0 / M_PI; };
  class URDF2Gazebo
  {
    public:
      URDF2Gazebo();
      URDF2Gazebo(std::string robot_model_name);
      ~URDF2Gazebo();

      std::string values2str(unsigned int count, const double *values, double (*conv)(double));

      void setupTransform(btTransform &transform, const double *xyz, const double *rpy);

      void addKeyValue(TiXmlElement *elem, const std::string& key, const std::string &value);

      void addTransform(TiXmlElement *elem, const::btTransform& transform);

      void copyGazeboMap(std::vector<TiXmlElement*>& data, TiXmlElement *elem, const std::vector<std::string> *tags);

      void copyOgreMap(const std::vector<TiXmlElement*>& data, TiXmlElement *elem, const std::vector<std::string> *tags);

      std::string getGeometrySize(rdf_parser::Geometry* geometry, int *sizeCount, double *sizeVals);
      
      std::string getGeometryBoundingBox(rdf_parser::Geometry* geometry, double *sizeVals);

      void convertLink(TiXmlElement *root, rdf_parser::Link *link, const btTransform &transform, bool enforce_limits);

      void convert(rdf_parser::RDF &wgxml, TiXmlDocument &doc, bool enforce_limits);

      std::string robot_model_name_;
  };
}

#endif
