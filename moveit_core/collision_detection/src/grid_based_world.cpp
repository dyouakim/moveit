////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush
///Dina Youakim

#include <moveit/collision_detection/grid_based_world.h>

// standard includes
#include <map>

// system includes
#include <Eigen/Dense>
#include <boost/make_shared.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
//#include <leatherman/utils.h>
#include <moveit/collision_detection/world.h>
#include <octomap_msgs/conversions.h>
// project includes
#include <moveit/collision_detection/voxel_operations.h>
#include <ros/console.h>
namespace collision_detection {

GridWorld::GridWorld(sbpl::OccupancyGrid* grid) :
    collision_detection::World::World(),
    m_grid(grid),
    m_object_map(),
    m_object_voxel_map(),
    m_padding(0.0)
{
}

GridWorld::GridWorld(GridWorld& world) :
    collision_detection::World::World(world)
    //,m_grid(world.grid())
{
    m_grid = new sbpl::OccupancyGrid(*world.grid());
    m_object_voxel_map = world.m_object_voxel_map;
    m_padding = world.padding();
    m_object_map = world.m_object_map;
}

GridWorld::GridWorld(const World& world, sbpl::OccupancyGrid* grid)
:
    collision_detection::World::World(world),
    m_object_map(),
    m_object_voxel_map(),
    m_padding(0.0)
    //,m_grid(grid)
{
    m_grid = new sbpl::OccupancyGrid(*grid);
}


collision_detection::GridWorld::~GridWorld()
{

}

sbpl::OccupancyGrid* GridWorld::grid()
{
    return m_grid;
}

const sbpl::OccupancyGrid* GridWorld::grid() const
{
    return m_grid;
}

void collision_detection::GridWorld::addToObject(const std::string& id, const bool is_manip_obj, const std::vector<shapes::ShapeConstPtr>& shapes,
                                             const EigenSTL::vector_Affine3d& poses)
{
    collision_detection::World::addToObject(id,is_manip_obj, shapes,poses);
    if(!is_manip_obj)
    {
        ObjectConstPtr object = collision_detection::World::getObject(id);
        if(!insertObjectInGrid(object))
            ROS_ERROR("failed to add the object into the grid!");
    }
}

void collision_detection::GridWorld::addToObject(const std::string& id, const bool is_manip_obj, const shapes::ShapeConstPtr& shape,
                                             const Eigen::Affine3d& pose)
{
    collision_detection::World::addToObject(id,is_manip_obj, shape,pose);
    if(!is_manip_obj)
    {
        ObjectConstPtr object = collision_detection::World::getObject(id);
        if(!insertObjectInGrid(object))
            ROS_ERROR("failed to add the object into the grid!");
    }
    else
        ROS_ERROR("Manipulation Object not added to grid");
    
}

bool collision_detection::GridWorld::removeObject(const std::string& id)
{
    collision_detection::World::removeObject(id);
    return removeObjectFromGrid(id);
}

bool collision_detection::GridWorld::moveShapeInObject(const std::string& id, const shapes::ShapeConstPtr& shape,
                                                   const Eigen::Affine3d& pose)
{
  ObjectConstPtr object = collision_detection::World::getObject(id);
  collision_detection::World::moveShapeInObject(id, shape,pose);
  return moveShapesInGrid(object) ;
}

bool collision_detection::GridWorld::removeShapeFromObject(const std::string& id, const shapes::ShapeConstPtr& shape)
{
  ObjectConstPtr object = collision_detection::World::getObject(id);
  collision_detection::World::removeShapeFromObject(id, shape);
  return removeShapesFromGrid(object) ;
}

void collision_detection::GridWorld::clearObjects()
{
  collision_detection::World::clearObjects();
  reset();
}


////Private util functions to process the grid////


bool GridWorld::insertObjectInGrid(const ObjectConstPtr& object)
{
    auto old_vit = m_object_voxel_map.find(object->id_);
    if( old_vit != m_object_voxel_map.end())
        m_object_voxel_map.erase(old_vit);

    const double res = m_grid->resolution();
    const Eigen::Vector3d origin(
            m_grid->originX(), m_grid->originY(), m_grid->originZ());
    
    const Eigen::Vector3d gmin(
            m_grid->originX(), m_grid->originY(), m_grid->originZ());
    
    const Eigen::Vector3d gmax(
            m_grid->originX() + m_grid->sizeX(),
            m_grid->originY() + m_grid->sizeY(),
            m_grid->originZ() + m_grid->sizeZ());
    
    std::vector<std::vector<Eigen::Vector3d>> all_voxels;
    if (!VoxelizeObject(*object, res, origin, gmin, gmax, all_voxels)) {
        ROS_ERROR("Failed to voxelize object '%s'", object->id_.c_str());
        return false;
    }
    auto vit = m_object_voxel_map.insert(
            std::make_pair(object->id_, std::vector<VoxelList>()));
    vit.first->second = std::move(all_voxels);
    //assert(vit.second);
    m_object_map.insert(std::make_pair(object->id_, object));
    for (const auto& voxel_list : vit.first->second) {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
        ROS_DEBUG("Adding %zu voxels from collision object '%s' to the distance transform",
                voxel_list.size(), object->id_.c_str());
        m_grid->addPointsToField(voxel_list);
    }
    return true;
}

void GridWorld::printObjectsSize()
{
        ROS_DEBUG("Parent objects are %zu ; Grid World with voxel size %zu and objects size %zu",collision_detection::World::getObjectIds().size(),m_object_map.size(),m_object_map.size());
}

bool GridWorld::removeObjectFromGrid(const std::string& object_name)
{
    auto oit = m_object_map.find(object_name);
    auto vit = m_object_voxel_map.find(object_name);
    
    if(oit != m_object_map.end() && vit != m_object_voxel_map.end())
    {
       for (const auto& voxel_list : vit->second) {
            ROS_DEBUG("Removing %zu voxels from the distance transform", voxel_list.size());
            m_grid->removePointsFromField(voxel_list);
        }
        m_object_voxel_map.erase(vit);
        m_object_map.erase(oit);
        return true;
    }
    else if (oit != m_object_map.end() && vit == m_object_voxel_map.end())
    {
        m_object_map.erase(oit);
        return true;

    }
    else if (oit == m_object_map.end() && vit != m_object_voxel_map.end())
    {
       for (const auto& voxel_list : vit->second) {
            m_grid->removePointsFromField(voxel_list);
        }
        m_object_voxel_map.erase(vit);
        return true;
    }
    ROS_DEBUG("after removing failure %s ", object_name.c_str());
        printObjectsSize();
    return false;
}

bool GridWorld::removeObjectFromGrid(const ObjectConstPtr& object)
{
    return removeObjectFromGrid(object->id_);
}

bool GridWorld::moveShapesInGrid(const ObjectConstPtr& object)
{
    // TODO: optimized version
    if(removeObjectFromGrid(object))
        return insertObjectInGrid(object);
    else
        return false;
}

bool GridWorld::insertShapesInGrid(const ObjectConstPtr& object)
{
    // TODO: optimized version
    //return insertObjectInGrid(object);
    return removeObjectFromGrid(object) && insertObjectInGrid(object);
}

bool GridWorld::removeShapesFromGrid(const ObjectConstPtr& object)
{
    // TODO: optimized version
    return removeObjectFromGrid(object) && insertObjectInGrid(object);
}

void GridWorld::reset()
{
    m_grid->reset();
    m_object_voxel_map.clear();
    m_object_map.clear();

    /*for (const auto& entry : m_object_voxel_map) {
        for (const auto& voxel_list : entry.second) {
            m_grid->addPointsToField(voxel_list);
        }
    }*/
}


void GridWorld::setPadding(double padding)
{
    m_padding = padding;
}

double GridWorld::padding() const
{
    return m_padding;
}

} // namespace collision_detection
