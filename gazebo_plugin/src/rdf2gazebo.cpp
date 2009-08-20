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

#include <gazebo_plugin/urdf2gazebo.h>
#include "ros/node.h"

using namespace urdf2gazebo;

URDF2Gazebo::URDF2Gazebo()
{
  robot_model_name_ = "pr2_model";
}
URDF2Gazebo::URDF2Gazebo(std::string robot_model_name)
{
  robot_model_name_ = robot_model_name;
}

URDF2Gazebo::~URDF2Gazebo()
{
}


std::string URDF2Gazebo::values2str(unsigned int count, const double *values, double (*conv)(double) = NULL)
{
    std::stringstream ss;
    for (unsigned int i = 0 ; i < count ; i++)
    {
        if (i > 0)
            ss << " ";
        ss << (conv ? conv(values[i]) : values[i]);
    }
    return ss.str();
}

void URDF2Gazebo::setupTransform(btTransform &transform, const double *xyz, const double *rpy)
{
    btMatrix3x3 mat;
    mat.setEulerZYX(rpy[2],rpy[1],rpy[0]);
    transform = btTransform(mat,btVector3(xyz[0],xyz[1],xyz[2]));
}

void URDF2Gazebo::addKeyValue(TiXmlElement *elem, const std::string& key, const std::string &value)
{
    TiXmlElement *ekey      = new TiXmlElement(key);
    TiXmlText    *text_ekey = new TiXmlText(value);
    ekey->LinkEndChild(text_ekey);    
    elem->LinkEndChild(ekey); 
}

void URDF2Gazebo::addTransform(TiXmlElement *elem, const::btTransform& transform)
{
    btVector3 pz = transform.getOrigin();
    double cpos[3] = { pz.x(), pz.y(), pz.z() };
    btMatrix3x3 mat = transform.getBasis();
    double crot[3];
    mat.getEulerZYX(crot[2],crot[1],crot[0]);
    
    /* set geometry transform */
    addKeyValue(elem, "xyz", values2str(3, cpos));
    addKeyValue(elem, "rpy", values2str(3, crot, rad2deg));  
}

void URDF2Gazebo::copyGazeboMap(std::vector<TiXmlElement*>& map_data, TiXmlElement *elem, const std::vector<std::string> *tags = NULL)
{
    std::vector<std::string> gazebo_names;
    std::vector<std::string> gazebo_values;
    for( std::vector<TiXmlElement*>::iterator map = map_data.begin(); map!=map_data.end(); map++)
    {
        const char* map_flag = (*map)->Attribute("flag");
        if (map_flag != NULL)
        {
            if (strcmp(map_flag,"gazebo") == 0)
            {
                gazebo_names.push_back(std::string(map_flag));
            }
        }
    }

    for (unsigned int k = 0 ; k < gazebo_names.size() ; ++k)
    {
        std::map<std::string, std::string> m = data.getMapTagValues("gazebo", gazebo_names[k]);
        std::vector<std::string> accepted_tags;
        if (tags)
            accepted_tags = *tags;
        else
            for (std::map<std::string, std::string>::iterator it = m.begin() ; it != m.end() ; it++)
                accepted_tags.push_back(it->first);
        
        for (unsigned int i = 0 ; i < accepted_tags.size() ; ++i)
            if (m.find(accepted_tags[i]) != m.end())
                addKeyValue(elem, accepted_tags[i], m[accepted_tags[i]]);
        
        std::map<std::string, const TiXmlElement*> x = data.getMapTagXML("gazebo", gazebo_names[k]);
        for (std::map<std::string, const TiXmlElement*>::iterator it = x.begin() ; it != x.end() ; it++)
        {
            for (const TiXmlNode *child = it->second->FirstChild() ; child ; child = child->NextSibling())
                elem->LinkEndChild(child->Clone());
        }              
    }
}

void URDF2Gazebo::copyOgreMap(const std::vector<TiXmlElement*>& map_data, TiXmlElement *elem, const std::vector<std::string> *tags = NULL)
{
    std::vector<std::string> ogre_names;
    data.getMapTagNames("ogre", ogre_names);
    for (unsigned int k = 0 ; k < ogre_names.size() ; ++k)
    {
        std::map<std::string, std::string> m = data.getMapTagValues("ogre", ogre_names[k]);
        std::vector<std::string> accepted_tags;
        if (tags)
            accepted_tags = *tags;
        else
            for (std::map<std::string, std::string>::iterator it = m.begin() ; it != m.end() ; it++)
                accepted_tags.push_back(it->first);
        
        for (unsigned int i = 0 ; i < accepted_tags.size() ; ++i)
            if (m.find(accepted_tags[i]) != m.end())
                addKeyValue(elem, accepted_tags[i], m[accepted_tags[i]]);
        
        std::map<std::string, const TiXmlElement*> x = data.getMapTagXML("ogre", ogre_names[k]);
        for (std::map<std::string, const TiXmlElement*>::iterator it = x.begin() ; it != x.end() ; it++)
        {
            for (const TiXmlNode *child = it->second->FirstChild() ; child ; child = child->NextSibling())
                elem->LinkEndChild(child->Clone());
        }              
    }
}

std::string URDF2Gazebo::getGeometrySize(rdf_parser::RDF::Link::Geometry* geometry, int *sizeCount, double *sizeVals)
{
    std::string type;
    
    switch (geometry->type)
    {
    case rdf_parser::RDF::Link::Geometry::BOX:
        type = "box";
        *sizeCount = 3;
        {
            rdf_parser::RDF::Link::Geometry::Box* box = static_cast<rdf_parser::RDF::Link::Geometry::Box*>(geometry->shape);
            sizeVals[0] = box->size[0];
            sizeVals[1] = box->size[1];
            sizeVals[2] = box->size[2];
        }
        break;
    case rdf_parser::RDF::Link::Geometry::CYLINDER:
        type = "cylinder";
        *sizeCount = 2;
        {
            rdf_parser::RDF::Link::Geometry::Cylinder* cylinder = static_cast<rdf_parser::RDF::Link::Geometry::Cylinder*>(geometry->shape);
            sizeVals[0] = cylinder->radius;
            sizeVals[1] = cylinder->length;
        }
        break;
    case rdf_parser::RDF::Link::Geometry::SPHERE:
        type = "sphere";
        *sizeCount = 1;
        sizeVals[0] = static_cast<rdf_parser::RDF::Link::Geometry::Sphere*>(geometry->shape)->radius;
        break;
    case rdf_parser::RDF::Link::Geometry::MESH:
        type = "trimesh";
        *sizeCount = 3;
        {
          rdf_parser::RDF::Link::Geometry::Mesh* mesh = static_cast<rdf_parser::RDF::Link::Geometry::Mesh*>(geometry->shape);
          sizeVals[0] = mesh->scale[0];
          sizeVals[1] = mesh->scale[1];
          sizeVals[2] = mesh->scale[2];
        }
        break;
    default:
        *sizeCount = 0;
        printf("Unknown body type: %d in geometry '%s'\n", geometry->type, geometry->name.c_str());
        break;
    }
    
    return type;
}

std::string URDF2Gazebo::getGeometryBoundingBox(rdf_parser::RDF::Link::Geometry* geometry, double *sizeVals)
{
    std::string type;
    
    switch (geometry->type)
    {
    case rdf_parser::RDF::Link::Geometry::BOX:
        type = "box";
        {
            rdf_parser::RDF::Link::Geometry::Box* box = static_cast<rdf_parser::RDF::Link::Geometry::Box*>(geometry->shape);
            sizeVals[0] = box->size[0];
            sizeVals[1] = box->size[1];
            sizeVals[2] = box->size[2];
        }
        break;
    case rdf_parser::RDF::Link::Geometry::CYLINDER:
        type = "cylinder";
        {
            rdf_parser::RDF::Link::Geometry::Cylinder* cylinder = static_cast<rdf_parser::RDF::Link::Geometry::Cylinder*>(geometry->shape);
            sizeVals[0] = cylinder->radius * 2;
            sizeVals[1] = cylinder->radius * 2;
            sizeVals[2] = cylinder->length;
        }
        break;
    case rdf_parser::RDF::Link::Geometry::SPHERE:
        type = "sphere";
        sizeVals[0] = sizeVals[1] = sizeVals[2] = static_cast<rdf_parser::RDF::Link::Geometry::Sphere*>(geometry->shape)->radius * 2;
        break;
    case rdf_parser::RDF::Link::Geometry::MESH:
        type = "trimesh";
        {
          rdf_parser::RDF::Link::Geometry::Mesh* mesh = static_cast<rdf_parser::RDF::Link::Geometry::Mesh*>(geometry->shape);
          sizeVals[0] = mesh->scale[0];
          sizeVals[1] = mesh->scale[1];
          sizeVals[2] = mesh->scale[2];
        }
        break;
    default:
        sizeVals[0] = sizeVals[1] = sizeVals[2] = 0;
        printf("Unknown body type: %d in geometry '%s'\n", geometry->type, geometry->name.c_str());
        break;
    }
    
    return type;
}

void URDF2Gazebo::convertLink(TiXmlElement *root, rdf_parser::RDF::Link *link, const btTransform &transform, bool enforce_limits)
{
    btTransform currentTransform = transform;
    
    int linkGeomSize;
    double linkSize[3];
    std::string type = getGeometrySize(link->collision->geometry, &linkGeomSize, linkSize);
    
    // This should be made smarter.
    if(!link->visual) 
    {
      printf("ignoring link without visual tag: %s\n", link->name.c_str());
      return;
    }

    if(!link->inertial)
    {
      printf("ignoring link without inertial tag: %s\n", link->name.c_str());
      return;
    }

    if(link->inertial->mass == 0.0) 
    {
      printf("ignoring link with zero mass: %s\n", link->name.c_str());
      return;
    }

    if (!type.empty())
    {
        /* create new body */
        TiXmlElement *elem     = new TiXmlElement("body:" + type);
        
        /* set body name */
        elem->SetAttribute("name", link->name);


        /* set mass properties */
        addKeyValue(elem, "massMatrix", "true");
        addKeyValue(elem, "mass", values2str(1, &link->inertial->mass));
        
        static const char tagList1[6][4] = {"ixx", "ixy", "ixz", "iyy", "iyz", "izz"};
        for (int j = 0 ; j < 6 ; ++j)
            addKeyValue(elem, tagList1[j], values2str(1, link->inertial->inertia + j));
        
        static const char tagList2[3][3] = {"cx", "cy", "cz"};
        for (int j = 0 ; j < 3 ; ++j)
        {
            double tmp_value = (link->inertial->com)[j] - 0*(link->collision->xyz)[j];
            addKeyValue(elem, tagList2[j], values2str(1, &tmp_value));
        }
        
        /* compute global transform */
        btTransform localTransform;
        setupTransform(localTransform, link->xyz, link->rpy);
        currentTransform *= localTransform;
        addTransform(elem, currentTransform);
        
        if (link->collision->verbose)
            addKeyValue(elem, "reportStaticCollision", "true");
        
        /* begin create geometry node */
        TiXmlElement *geom     = new TiXmlElement("geom:" + type);
        
        {    
            /* set its name */
            geom->SetAttribute("name", link->collision->geometry->name);
            
            /* set transform */
            addKeyValue(geom, "xyz", values2str(3, link->collision->xyz));
            addKeyValue(geom, "rpy", values2str(3, link->collision->rpy, rad2deg));
            
            if (link->collision->geometry->type == rdf_parser::RDF::Link::Geometry::MESH)
            {  
                rdf_parser::RDF::Link::Geometry::Mesh* mesh = static_cast<rdf_parser::RDF::Link::Geometry::Mesh*>(link->collision->geometry->shape);
                /* set mesh size or scale */
                addKeyValue(geom, "scale", values2str(3, mesh->scale));

                /* set mesh file */
                // strip extension from filename
                std::string tmp_extension(".stl");
                int pos1 = mesh->filename.find(tmp_extension,0);
                mesh->filename.replace(pos1,mesh->filename.size()-pos1+1,std::string(""));
                // add mesh filename
                addKeyValue(geom, "mesh", mesh->filename + ".mesh");
                
            }
            else
            {
                /* set geometry size */
                addKeyValue(geom, "size", values2str(linkGeomSize, linkSize));
            }
            
            /* set additional data */      
            copyGazeboMap(link->collision->data, geom);
            
            /* begin create visual node */
            TiXmlElement *visual = new TiXmlElement("visual");
            {
                /* compute the visualisation transfrom */
                btTransform coll;
                setupTransform(coll, link->collision->xyz, link->collision->rpy);
                coll.inverse();
                
                btTransform vis;
                setupTransform(vis, link->visual->xyz, link->visual->rpy);
                coll = coll.inverseTimes(vis);
                
                addTransform(visual, coll);
                
                /* set geometry size */                
                
                if (link->visual->geometry->type == rdf_parser::RDF::Link::Geometry::MESH)
                {  
                    rdf_parser::RDF::Link::Geometry::Mesh* mesh = static_cast<rdf_parser::RDF::Link::Geometry::Mesh*>(link->visual->geometry->shape);
                    /* set mesh size or scale */
                    /*
                    if (link->visual->geometry->isSet["size"])
                        addKeyValue(visual, "size", values2str(3, mesh->size));        
                    else
                        addKeyValue(visual, "scale", values2str(3, mesh->scale));        
                    */
                    addKeyValue(visual, "scale", values2str(3, mesh->scale));

                    /* set mesh file */
                    if (mesh->filename.empty())
                        addKeyValue(visual, "mesh", "unit_" + type);
                    else
                    {
                       //  skipping this block as we test the copyOgreMap function
                       //  // strip extension from filename
                       //  std::string tmp_extension(".stl");
                       //  int pos1 = mesh->filename.find(tmp_extension,0);
                       //  mesh->filename.replace(pos1,mesh->filename.size()-pos1+1,std::string(""));
                       //  // add mesh filename
                       //  addKeyValue(visual, "mesh", mesh->filename + ".mesh");
                    }
                    
                }
                else
                {
                    double visualSize[3];
                    std::string visual_geom_type = getGeometryBoundingBox(link->visual->geometry, visualSize);
                    addKeyValue(visual, "scale", values2str(3, visualSize));
                    addKeyValue(visual, "mesh", "unit_" + visual_geom_type);
                }
                
                copyGazeboMap(link->visual->data, visual);

                // ogre mesh map for all trimeshes visualized
                copyOgreMap(link->visual->data, visual);
            }
            /* end create visual node */
            
            geom->LinkEndChild(visual);
        }
        /* end create geometry node */
        
        /* add geometry to body */
        elem->LinkEndChild(geom);      
        
        /* copy gazebo data */
        copyGazeboMap(link->data, elem);
        
        /* add body to document */
        root->LinkEndChild(elem);
        
        /* compute the joint tag */
        bool fixed = false;
        std::string jtype;
        if (link->joint != NULL)
        switch (link->joint->type)
        {
        case rdf_parser::RDF::Link::Joint::REVOLUTE:
            jtype = "hinge";
            break;
        case rdf_parser::RDF::Link::Joint::PRISMATIC:
            jtype = "slider";
            break;
        case rdf_parser::RDF::Link::Joint::FLOATING:
        case rdf_parser::RDF::Link::Joint::PLANAR:
            break;
        case rdf_parser::RDF::Link::Joint::FIXED:
            jtype = "hinge";
            fixed = true;
            break;
        default:
            printf("Unknown joint type: %d in link '%s'\n", link->joint->type, link->name.c_str());
            break;
        }
        
        if (!jtype.empty())
        {
            TiXmlElement *joint = new TiXmlElement("joint:" + jtype);
            joint->SetAttribute("name", link->joint->name);
            
            addKeyValue(joint, "body1", link->name);
            addKeyValue(joint, "body2", link->parentName);
            addKeyValue(joint, "anchor", link->name);
            
            if (fixed)
            {
                addKeyValue(joint, "lowStop", "0");
                addKeyValue(joint, "highStop", "0");
                addKeyValue(joint, "axis", "1 0 0");
            }
            else
            {
                btTransform currentTransformCopy( transform );
                currentTransformCopy.setOrigin( btVector3(0, 0, 0) );
                btVector3 rotatedJointAxis = currentTransformCopy * btVector3( link->joint->axis[0], link->joint->axis[1], link->joint->axis[2] );
                double rotatedJointAxisArray[3] = { rotatedJointAxis.x(), rotatedJointAxis.y(), rotatedJointAxis.z() };
                addKeyValue(joint, "axis", values2str(3, rotatedJointAxisArray));
                
                double tmpAnchor[3];
                
                for (int j = 0 ; j < 3 ; ++j)
                {
                    // undo Gazebo's shift of object anchor to geom cg center, stay in body cs
                    tmpAnchor[j] = (link->joint->anchor)[j]; // - (link->inertial->com)[j] - 0*(link->collision->xyz)[j];
                }
                
                addKeyValue(joint, "anchorOffset", values2str(3, tmpAnchor));
                
                if (enforce_limits && link->joint->isSet["limit"])
                {
                    if (jtype == "slider")
                    {
                        addKeyValue(joint, "lowStop",  values2str(1, link->joint->limit             ));
                        addKeyValue(joint, "highStop", values2str(1, link->joint->limit + 1         ));
                    }
                    else
                    {
                        double *lowstop  = link->joint->limit    ;
                        double *highstop = link->joint->limit + 1;
                        // enforce ode bounds, this will need to be fixed
                        if (*lowstop > 0)
                        {
                          ROS_WARN("urdf2gazebo: limiting lowStop to <= 0 degrees");
                          *lowstop = 0.0;
                        }
                        if (*lowstop < -(M_PI)*0.9)
                        {
                          ROS_WARN("urdf2gazebo: limiting lowStop to >= -(180)*0.9 degrees");
                          *lowstop = -(M_PI)*0.9;
                        }
                        if (*highstop < 0)
                        {
                          ROS_WARN("urdf2gazebo: limiting highStop to >= 0 degrees");
                          *highstop = 0.0;
                        }
                        if (*highstop > (M_PI)*0.9)
                        {
                          ROS_WARN("urdf2gazebo: limiting highStop to <= (180)*0.9 degrees");
                          *highstop = (M_PI)*0.9;
                        }
                        addKeyValue(joint, "lowStop",  values2str(1, link->joint->limit    , rad2deg));
                        addKeyValue(joint, "highStop", values2str(1, link->joint->limit + 1, rad2deg));
                    }
                }

                if (link->joint->isSet["pjointMimic"])
                    addKeyValue(joint,"mimicJoint", link->joint->pjointMimic->name                   );
                if (link->joint->isSet["fMimicMult"])
                    addKeyValue(joint,"mimicMult",   values2str(1, &(link->joint->fMimicMult)       ));
                if (link->joint->isSet["fMimicOffset"])
                    addKeyValue(joint,"mimicOffset", values2str(1, &(link->joint->fMimicOffset)     ));

            }
            
            /* copy gazebo data */
            copyGazeboMap(link->joint->data, joint);

            /* add joint to document */
            root->LinkEndChild(joint);
        }
    }
    
    for (unsigned int i = 0 ; i < link->children.size() ; ++i)
        convertLink(root, link->children[i], currentTransform, enforce_limits);
}

void URDF2Gazebo::convert(rdf_parser::RDF &rdf, TiXmlDocument &doc, bool enforce_limits)
{
    TiXmlDeclaration *decl = new TiXmlDeclaration("1.0", "", "");
    doc.LinkEndChild(decl);
    
    /* create root element and define needed namespaces */
    TiXmlElement *robot = new TiXmlElement("model:physical");
    robot->SetAttribute("xmlns:gazebo", "http://playerstage.sourceforge.net/gazebo/xmlschema/#gz");
    robot->SetAttribute("xmlns:model", "http://playerstage.sourceforge.net/gazebo/xmlschema/#model");
    robot->SetAttribute("xmlns:sensor", "http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor");
    robot->SetAttribute("xmlns:body", "http://playerstage.sourceforge.net/gazebo/xmlschema/#body");
    robot->SetAttribute("xmlns:geom", "http://playerstage.sourceforge.net/gazebo/xmlschema/#geom");
    robot->SetAttribute("xmlns:joint", "http://playerstage.sourceforge.net/gazebo/xmlschema/#joint");
    robot->SetAttribute("xmlns:controller", "http://playerstage.sourceforge.net/gazebo/xmlschema/#controller");
    robot->SetAttribute("xmlns:interface", "http://playerstage.sourceforge.net/gazebo/xmlschema/#interface");
    robot->SetAttribute("xmlns:rendering", "http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering");
    robot->SetAttribute("xmlns:physics", "http://playerstage.sourceforge.net/gazebo/xmlschema/#physics");
    
    // Create a node to enclose the robot body
    robot->SetAttribute("name", robot_model_name_);
    
    /* set the transform for the whole model to identity */
    addKeyValue(robot, "xyz", "0 0 0");
    addKeyValue(robot, "rpy", "0 0 0");
    btTransform transform;    
    transform.setIdentity();    
    
    for (unsigned int k = 0 ; k < rdf.getDisjointPartCount() ; ++k)
        convertLink(robot, rdf.getDisjointPart(k), transform, enforce_limits);
    
    /* find all data item types */
    copyGazeboMap(rdf.maps_, robot);
    
    doc.LinkEndChild(robot);
}










