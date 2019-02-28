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
/// Dina Youakim

#ifndef grid_based_world_h
#define grid_based_world_h

// standard includes
#include <memory>
#include <string>
#include <vector>

// system includes
#include <moveit_msgs/CollisionObject.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <smpl/occupancy_grid.h>
#include <shape_msgs/MeshTriangle.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/collision_detection/world.h>

namespace collision_detection
{

MOVEIT_CLASS_FORWARD(GridWorld);


class GridWorld : public World
{
public:

    GridWorld(sbpl::OccupancyGrid* grid);

    GridWorld(GridWorld& world);

    GridWorld(const World& world, sbpl::OccupancyGrid* grid);

    ~GridWorld();

    sbpl::OccupancyGrid* grid();
    const sbpl::OccupancyGrid* grid() const;

    /** \brief Add shapes to an object in the map.
   * This function makes repeated calls to addToObjectInternal() to add the
   * shapes one by one.
   *  \note This function does NOT call the addToObject() variant that takes
   * a single shape and a single pose as input. */
    void addToObject(const std::string& id, const bool is_manip_obj, const std::vector<shapes::ShapeConstPtr>& shapes,
                   const EigenSTL::vector_Affine3d& poses);

    /** \brief Add a shape to an object.
    * If the object already exists, this call will add the shape to the object
    * at the specified pose. Otherwise, the object is created and the
    * specified shape is added. This calls addToObjectInternal(). */
    void addToObject(const std::string& id, const bool is_manip_obj, const shapes::ShapeConstPtr& shape, const Eigen::Affine3d& pose);

    /** \brief Update the pose of a shape in an object. Shape equality is
    * verified by comparing pointers. Returns true on success. */
    bool moveShapeInObject(const std::string& id, const shapes::ShapeConstPtr& shape, const Eigen::Affine3d& pose);

    /** \brief Remove shape from object.
    * Shape equality is verified by comparing pointers. Ownership of the
    * object is renounced (i.e. object is deleted if no external references
    * exist) if this was the last shape in the object.
    * Returns true on success and false if the object did not exist or did not
    * contain the shape. */
    bool removeShapeFromObject(const std::string& id, const shapes::ShapeConstPtr& shape);

    /** \brief Remove a particular object.
    * If there are no external pointers to the corresponding instance of
    * Object, the memory is freed.
    * Returns true on success and false if no such object was found. */
    bool removeObject(const std::string& id);

    /** \brief Clear all objects.
    * If there are no other pointers to corresponding instances of Objects,
    * the memory is freed. */
    void clearObjects();

    void setPadding(double padding);
    double padding() const;

    void printObjectsSize();

private:

    sbpl::OccupancyGrid* m_grid;

    // set of collision objects
    std::map<std::string, ObjectConstPtr> m_object_map;

    // voxelization of objects in the grid reference frame
    typedef std::vector<Eigen::Vector3d> VoxelList;
    std::map<std::string, std::vector<VoxelList>> m_object_voxel_map;

    double m_padding;

    bool insertObjectInGrid(const ObjectConstPtr& object);
    bool removeObjectFromGrid(const std::string& object_name);
    bool removeObjectFromGrid(const ObjectConstPtr& object);
    bool moveShapesInGrid(const ObjectConstPtr& object);
    bool insertShapesInGrid(const ObjectConstPtr& object);
    bool removeShapesFromGrid(const ObjectConstPtr& object);

    /// \brief Reset the underlying occupancy grid.
    ///
    /// Resets the WorldCollisionModel by clearing the underlying occupancy grid
    void reset();
};


} // namespace collision

#endif
