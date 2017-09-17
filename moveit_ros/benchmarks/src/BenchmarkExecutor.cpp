/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Ryan Luna */

#include <moveit/benchmarks/BenchmarkExecutor.h>
#include <moveit/version.h>
#include <eigen_conversions/eigen_msg.h>

#include <boost/regex.hpp>
#include <boost/progress.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <unistd.h>

using namespace moveit_ros_benchmarks;

static std::string getHostname()
{
  static const int BUF_SIZE = 1024;
  char buffer[BUF_SIZE];
  int err = gethostname(buffer, sizeof(buffer));
  if (err != 0)
    return std::string();
  else
  {
    buffer[BUF_SIZE - 1] = '\0';
    return std::string(buffer);
  }
}

BenchmarkExecutor::BenchmarkExecutor(std::string nodeNamespace, const std::string& robot_description_param)
{
  pss_ = NULL;
  psws_ = NULL;
  rs_ = NULL;
  cs_ = NULL;
  tcs_ = NULL;
  psm_ = new planning_scene_monitor::PlanningSceneMonitor(robot_description_param);
  planning_scene_ = psm_->getPlanningScene();
  ros::NodeHandle node_handle(nodeNamespace);
  nh_ = node_handle;
  service_client = nh_.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
  fk_service_client = nh_.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
  planningScenePub_ = nh_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
  displaypathPub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1);
  attachedCollisionObject_pub = node_handle.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object",10);
  planningSceneUpdateSub_ = node_handle.subscribe("/move_group/monitored_planning_scene",100,&BenchmarkExecutor::planningSceneCallback,this);
  try
  {
    planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  firstConnector =true;
}

BenchmarkExecutor::~BenchmarkExecutor()
{
  if (pss_)
    delete pss_;
  if (psws_)
    delete psws_;
  if (rs_)
    delete rs_;
  if (cs_)
    delete cs_;
  if (tcs_)
    delete tcs_;
  delete psm_;
}

void BenchmarkExecutor::initialize(const std::vector<std::string>& plugin_classes)
{
  planner_interfaces_.clear();

  // Load the planning plugins
  const std::vector<std::string>& classes = planner_plugin_loader_->getDeclaredClasses();

  for (std::size_t i = 0; i < plugin_classes.size(); ++i)
  {
    std::vector<std::string>::const_iterator it = std::find(classes.begin(), classes.end(), plugin_classes[i]);
    if (it == classes.end())
    {
      ROS_ERROR("Failed to find plugin_class %s", plugin_classes[i].c_str());
      return;
    }

    try
    {
      planning_interface::PlannerManagerPtr p = planner_plugin_loader_->createUniqueInstance(plugin_classes[i]);
      p->initialize(planning_scene_->getRobotModel(), "");

      const planning_interface::PlannerConfigurationMap& config_map = p->getPlannerConfigurations();

      planner_interfaces_[plugin_classes[i]] = p;
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR_STREAM("Exception while loading planner '" << plugin_classes[i] << "': " << ex.what());
    }
  }

  // error check
  if (planner_interfaces_.empty())
    ROS_ERROR("No planning plugins have been loaded. Nothing to do for the benchmarking service.");
  else
  {
    std::stringstream ss;
    for (std::map<std::string, planning_interface::PlannerManagerPtr>::const_iterator it = planner_interfaces_.begin();
         it != planner_interfaces_.end(); ++it)
      ss << it->first << " ";
    ROS_INFO("Available planner instances: %s", ss.str().c_str());
  }
}

void BenchmarkExecutor::clear()
{
  if (pss_)
  {
    delete pss_;
    pss_ = NULL;
  }
  if (psws_)
  {
    delete psws_;
    psws_ = NULL;
  }
  if (rs_)
  {
    delete rs_;
    rs_ = NULL;
  }
  if (cs_)
  {
    delete cs_;
    cs_ = NULL;
  }
  if (tcs_)
  {
    delete tcs_;
    tcs_ = NULL;
  }

  benchmark_data_.clear();
  pre_event_fns_.clear();
  post_event_fns_.clear();
  planner_start_fns_.clear();
  planner_completion_fns_.clear();
  query_start_fns_.clear();
  query_end_fns_.clear();
}

void BenchmarkExecutor::addPreRunEvent(PreRunEventFunction func)
{
  pre_event_fns_.push_back(func);
}

void BenchmarkExecutor::addPostRunEvent(PostRunEventFunction func)
{
  post_event_fns_.push_back(func);
}

void BenchmarkExecutor::addPlannerStartEvent(PlannerStartEventFunction func)
{
  planner_start_fns_.push_back(func);
}

void BenchmarkExecutor::addPlannerCompletionEvent(PlannerCompletionEventFunction func)
{
  planner_completion_fns_.push_back(func);
}

void BenchmarkExecutor::addQueryStartEvent(QueryStartEventFunction func)
{
  query_start_fns_.push_back(func);
}

void BenchmarkExecutor::addQueryCompletionEvent(QueryCompletionEventFunction func)
{
  query_end_fns_.push_back(func);
}

bool BenchmarkExecutor::runBenchmarks(const BenchmarkOptions& opts)
{
  if (planner_interfaces_.size() == 0)
  {
    ROS_ERROR("No planning interfaces configured.  Did you call BenchmarkExecutor::initialize?");
    return false;
  }

  std::vector<BenchmarkRequest> queries;
  moveit_msgs::PlanningScene scene_msg;

  if (initializeBenchmarks(opts, scene_msg, queries))
  {

    /*if (!queriesAndPlannersCompatible(queries, opts.getPlannerConfigurations()))
      return false;*/
    for (std::size_t i = 0; i < queries.size(); ++i)
    {
      // Configure planning scene
      if (scene_msg.robot_model_name != planning_scene_->getRobotModel()->getName())
      {
        // Clear all geometry from the scene
        planning_scene_->getWorldNonConst()->clearObjects();
        planning_scene_->getCurrentStateNonConst().clearAttachedBodies();
        planning_scene_->getCurrentStateNonConst().setToDefaultValues();

        planning_scene_->processPlanningSceneWorldMsg(scene_msg.world);
       
      }
      else
      {
        planning_scene_->usePlanningSceneMsg(scene_msg); 
      }
      // Calling query start events
      for (std::size_t j = 0; j < query_start_fns_.size(); ++j)
        query_start_fns_[j](queries[i].request, planning_scene_);

      //Add any attached objects here
      std::vector<moveit_msgs::CollisionObject> collisionObjects = scene_msg.world.collision_objects;
      for(int i=0;i<collisionObjects.size();i++)
        if(collisionObjects[i].id=="stick")
        {
          moveit_msgs::AttachedCollisionObject attachedObject;
          attachedObject.object = collisionObjects[i];
          attachedObject.link_name="jaw";
          attachedCollisionObject_pub.publish(attachedObject);
          scene_msg.is_diff = true;
        }
      scene_msg.is_diff = true;
      scene_msg.robot_state.is_diff = true;
      planningScenePub_.publish(scene_msg);
      sleep(15);
     
      collision_detection::AllowedCollisionMatrix& acm = planning_scene_->getAllowedCollisionMatrixNonConst();
        acm.setEntry("plug_goal.stl",false);
        acm.setEntry("plug_goal.stl","palm",true);
        acm.setEntry("panel",false);
        acm.setEntry("panel","palm",true);
        acm.setEntry("panel","forearm",true);
        moveit_msgs::PlanningScene modifiedSceneMsg;
        planning_scene_->getPlanningSceneMsg(modifiedSceneMsg);
        acm.getMessage(modifiedSceneMsg.allowed_collision_matrix);
        modifiedSceneMsg.is_diff = true;
        planningScenePub_.publish(modifiedSceneMsg);

sleep(10);
     ROS_ERROR_STREAM("Benchmarking query "<<queries[i].name.c_str()<<"("<<i + 1<<" of "<<queries.size()<<")");
      ros::WallTime start_time = ros::WallTime::now();
      runBenchmark(queries[i].request, options_.getPlannerConfigurations(), options_.getNumRuns());
      double duration = (ros::WallTime::now() - start_time).toSec();
      for (std::size_t j = 0; j < query_end_fns_.size(); ++j)
        query_end_fns_[j](queries[i].request, planning_scene_);
      //This line fails with the consistency test and multiple planners, no idea why remove for now
      //writeOutput(queries[i], boost::posix_time::to_iso_extended_string(start_time.toBoost()), duration);

    }

    return true;
  }
  return false;
}

bool BenchmarkExecutor::queriesAndPlannersCompatible(const std::vector<BenchmarkRequest>& requests,
                                                     const std::map<std::string, std::vector<std::string>>& planners)
{
  // Make sure that the planner interfaces can service the desired queries
  for (std::map<std::string, planning_interface::PlannerManagerPtr>::const_iterator it = planner_interfaces_.begin();
       it != planner_interfaces_.end(); ++it)
  {
    for (std::size_t i = 0; i < requests.size(); ++i)
    {
      if (!it->second->canServiceRequest(requests[i].request))
      {
         ROS_ERROR("Interface '%s' cannot service the benchmark request '%s'", it->first.c_str(),
                  requests[i].name.c_str());
        return false;
      }
    }
  }

  return true;
}

bool BenchmarkExecutor::initializeBenchmarks(const BenchmarkOptions& opts, moveit_msgs::PlanningScene& scene_msg,
                                             std::vector<BenchmarkRequest>& requests)
{
  if (!plannerConfigurationsExist(opts.getPlannerConfigurations(), opts.getGroupName()))
    return false;

  try
  {
    warehouse_ros::DatabaseConnection::Ptr conn = dbloader.loadDatabase();
    conn->setParams(opts.getHostName(), opts.getPort(), 20);
    if (conn->connect())
    {
      pss_ = new moveit_warehouse::PlanningSceneStorage(conn);
      psws_ = new moveit_warehouse::PlanningSceneWorldStorage(conn);
      rs_ = new moveit_warehouse::RobotStateStorage(conn);
      cs_ = new moveit_warehouse::ConstraintsStorage(conn);
      tcs_ = new moveit_warehouse::TrajectoryConstraintsStorage(conn);
    }
    else
    {
      ROS_ERROR("Failed to connect to DB");
      return false;
    }
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR("Failed to initialize benchmark server: '%s'", e.what());
    return false;
  }

  std::vector<StartState> start_states;
  std::vector<PathConstraints> path_constraints;
  std::vector<PathConstraints> goal_constraints;
  std::vector<TrajectoryConstraints> traj_constraints;
  std::vector<BenchmarkRequest> queries;



  const std::string& group_name = opts.getGroupName();
  std::vector<std::string> pathConstraints;
  pathConstraints.push_back(opts.getPathConstraintRegex());
  bool ok = loadPlanningScene(opts.getSceneName(), scene_msg) && loadStates(opts.getStartStateRegex(), start_states) &&
            loadPathConstraints(opts.getGoalConstraintRegex(), goal_constraints) &&
            loadPathConstraints(pathConstraints, path_constraints) &&
            loadTrajectoryConstraints(opts.getTrajectoryConstraintRegex(), traj_constraints) &&
            loadQueries(opts.getQueryRegex(), opts.getSceneName(), queries);

  if (!ok)
  {
    ROS_ERROR("Failed to load benchmark stuff");
    return false;
  }

  ROS_INFO("Benchmark loaded %lu starts, %lu goals, %lu path constraints, %lu trajectory constraints, and %lu queries",
           start_states.size(), goal_constraints.size(), path_constraints.size(), traj_constraints.size(),
           queries.size());

  moveit_msgs::WorkspaceParameters workspace_parameters = opts.getWorkspaceParameters();
  // Make sure that workspace_parameters are set
  if (workspace_parameters.min_corner.x == workspace_parameters.max_corner.x &&
      workspace_parameters.min_corner.x == 0.0 &&
      workspace_parameters.min_corner.y == workspace_parameters.max_corner.y &&
      workspace_parameters.min_corner.y == 0.0 &&
      workspace_parameters.min_corner.z == workspace_parameters.max_corner.z &&
      workspace_parameters.min_corner.z == 0.0)
  {
    workspace_parameters.min_corner.x = workspace_parameters.min_corner.y = workspace_parameters.min_corner.z = -5.0;

    workspace_parameters.max_corner.x = workspace_parameters.max_corner.y = workspace_parameters.max_corner.z = 5.0;
  }
  workspace_parameters.header.frame_id = opts.getWorkspaceFrameID();

  std::vector<double> goal_offset;
  opts.getGoalOffsets(goal_offset);

  // Create the combinations of BenchmarkRequests

  // 1) Create requests for combinations of start states,
  //    goal constraints, and path constraints
  for (std::size_t i = 0; i < goal_constraints.size(); ++i)
  {
    // Common benchmark request properties
    BenchmarkRequest brequest;
    brequest.name = goal_constraints[i].name;
    brequest.request.workspace_parameters = workspace_parameters;
    brequest.request.goal_constraints = goal_constraints[i].constraints;
    brequest.request.group_name = opts.getGroupName();
    brequest.request.allowed_planning_time = opts.getTimeout();
    brequest.request.num_planning_attempts = 1;

    if (brequest.request.goal_constraints.size() == 1 &&
        brequest.request.goal_constraints[0].position_constraints.size() == 1 &&
        brequest.request.goal_constraints[0].orientation_constraints.size() == 1 &&
        brequest.request.goal_constraints[0].visibility_constraints.size() == 0 &&
        brequest.request.goal_constraints[0].joint_constraints.size() == 0)
      shiftConstraintsByOffset(brequest.request.goal_constraints[0], goal_offset);
    std::vector<BenchmarkRequest> request_combos;
    std::vector<StartState> currentStart ;
    //for consistency test only one start state and multiple goals
    currentStart.push_back(start_states[0]);
    createRequestCombinations(brequest, currentStart, path_constraints, request_combos);
    requests.insert(requests.end(), request_combos.begin(), request_combos.end());
  }

  // 2) Existing queries are treated like goal constraints.
  //    Create all combos of query, start states, and path constraints
  for (std::size_t i = 0; i < queries.size(); ++i)
  {
    // Common benchmark request properties
    BenchmarkRequest brequest;
    brequest.name = queries[i].name;
    brequest.request = queries[i].request;
    brequest.request.group_name = opts.getGroupName();
    brequest.request.allowed_planning_time = opts.getTimeout();
    brequest.request.num_planning_attempts = 1;

    // Make sure that workspace_parameters are set
    if (brequest.request.workspace_parameters.min_corner.x == brequest.request.workspace_parameters.max_corner.x &&
        brequest.request.workspace_parameters.min_corner.x == 0.0 &&
        brequest.request.workspace_parameters.min_corner.y == brequest.request.workspace_parameters.max_corner.y &&
        brequest.request.workspace_parameters.min_corner.y == 0.0 &&
        brequest.request.workspace_parameters.min_corner.z == brequest.request.workspace_parameters.max_corner.z &&
        brequest.request.workspace_parameters.min_corner.z == 0.0)
    {
      // ROS_WARN("Workspace parameters are not set for request %s.  Setting defaults", queries[i].name.c_str());
      brequest.request.workspace_parameters = workspace_parameters;
    }

    // Create all combinations of start states and path constraints
    std::vector<BenchmarkRequest> request_combos;
    createRequestCombinations(brequest, start_states, path_constraints, request_combos);
    requests.insert(requests.end(), request_combos.begin(), request_combos.end());
  }

  // 3) Trajectory constraints are also treated like goal constraints
  for (std::size_t i = 0; i < traj_constraints.size(); ++i)
  {
    // Common benchmark request properties
    BenchmarkRequest brequest;
    brequest.name = traj_constraints[i].name;
    brequest.request.trajectory_constraints = traj_constraints[i].constraints;
    brequest.request.group_name = opts.getGroupName();
    brequest.request.allowed_planning_time = opts.getTimeout();
    brequest.request.num_planning_attempts = 1;

    if (brequest.request.trajectory_constraints.constraints.size() == 1 &&
        brequest.request.trajectory_constraints.constraints[0].position_constraints.size() == 1 &&
        brequest.request.trajectory_constraints.constraints[0].orientation_constraints.size() == 1 &&
        brequest.request.trajectory_constraints.constraints[0].visibility_constraints.size() == 0 &&
        brequest.request.trajectory_constraints.constraints[0].joint_constraints.size() == 0)
      shiftConstraintsByOffset(brequest.request.trajectory_constraints.constraints[0], goal_offset);

    std::vector<BenchmarkRequest> request_combos;
    std::vector<PathConstraints> no_path_constraints;
    createRequestCombinations(brequest, start_states, no_path_constraints, request_combos);
    requests.insert(requests.end(), request_combos.begin(), request_combos.end());
  }

  options_ = opts;
  for(int i=0;i<requests.size();i++)
  {
    requests[i].request.workspace_parameters = workspace_parameters;
       
  }
  return true;
}

void BenchmarkExecutor::shiftConstraintsByOffset(moveit_msgs::Constraints& constraints,
                                                 const std::vector<double> offset)
{
  Eigen::Affine3d offset_tf(Eigen::AngleAxis<double>(offset[3], Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxis<double>(offset[4], Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxis<double>(offset[5], Eigen::Vector3d::UnitZ()));
  offset_tf.translation() = Eigen::Vector3d(offset[0], offset[1], offset[2]);

  geometry_msgs::Pose constraint_pose_msg;
  constraint_pose_msg.position = constraints.position_constraints[0].constraint_region.primitive_poses[0].position;
  constraint_pose_msg.orientation = constraints.orientation_constraints[0].orientation;
  Eigen::Affine3d constraint_pose;
  tf::poseMsgToEigen(constraint_pose_msg, constraint_pose);

  Eigen::Affine3d new_pose = constraint_pose * offset_tf;
  geometry_msgs::Pose new_pose_msg;
  tf::poseEigenToMsg(new_pose, new_pose_msg);

  constraints.position_constraints[0].constraint_region.primitive_poses[0].position = new_pose_msg.position;
  constraints.orientation_constraints[0].orientation = new_pose_msg.orientation;
}

void BenchmarkExecutor::createRequestCombinations(const BenchmarkRequest& brequest,
                                                  const std::vector<StartState>& start_states,
                                                  const std::vector<PathConstraints>& path_constraints,
                                                  std::vector<BenchmarkRequest>& requests)
{
  // Use default start state
  if (start_states.empty())
  {
    // Adding path constraints
    for (std::size_t k = 0; k < path_constraints.size(); ++k)
    {
      BenchmarkRequest new_brequest = brequest;
      new_brequest.request.path_constraints = path_constraints[k].constraints[0];
      new_brequest.name = brequest.name + "_" + path_constraints[k].name;
      requests.push_back(new_brequest);
    }

    if (path_constraints.empty())
      requests.push_back(brequest);
  }
  else  // Create a request for each start state specified
  {
    for (std::size_t j = 0; j < start_states.size(); ++j)
    {
      BenchmarkRequest new_brequest = brequest;
      new_brequest.request.start_state = start_states[j].state;

      // Duplicate the request for each of the path constraints
      for (std::size_t k = 0; k < path_constraints.size(); ++k)
      {
        new_brequest.request.path_constraints = path_constraints[k].constraints[0];
        new_brequest.name = start_states[j].name + "_" + new_brequest.name + "_" + path_constraints[k].name;
        requests.push_back(new_brequest);
      }

      if (path_constraints.empty())
      {
        new_brequest.name = start_states[j].name + "_" + brequest.name;
        requests.push_back(new_brequest);
      }
    }
  }
 
}

bool BenchmarkExecutor::plannerConfigurationsExist(const std::map<std::string, std::vector<std::string>>& planners,
                                                   const std::string& group_name)
{
  // Make sure planner plugins exist
  for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end();
       ++it)
  {
    bool plugin_exists = false;
    for (std::map<std::string, planning_interface::PlannerManagerPtr>::const_iterator planner_it =
             planner_interfaces_.begin();
         planner_it != planner_interfaces_.end() && !plugin_exists; ++planner_it)
    {
      plugin_exists = planner_it->first == it->first;
    }

    if (!plugin_exists)
    {
      ROS_ERROR("Planning plugin '%s' does NOT exist", it->first.c_str());
      return false;
    }
  }

  // Make sure planning algorithms exist within those plugins
  for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end();
       ++it)
  {
    planning_interface::PlannerManagerPtr pm = planner_interfaces_[it->first];
    const planning_interface::PlannerConfigurationMap& config_map = pm->getPlannerConfigurations();

    for (std::size_t i = 0; i < it->second.size(); ++i)
    {
      bool planner_exists = false;
      if(config_map.size()>0)
      {
        for (planning_interface::PlannerConfigurationMap::const_iterator map_it = config_map.begin();
             map_it != config_map.end() && !planner_exists; ++map_it)
        {
          ROS_ERROR_STREAM("Planner name in the config is: "<<it->second[i]<<" and "<<map_it->second.name);
          std::string planner_name = group_name + "[" + it->second[i] + "]";
          planner_exists = (map_it->second.group == group_name && map_it->second.name == planner_name);
        }

        if (!planner_exists)
        {
          ROS_ERROR("Planner '%s' does NOT exist for group '%s' in pipeline '%s'", it->second[i].c_str(),
                    group_name.c_str(), it->first.c_str());
          std::cout << "There are " << config_map.size() << " planner entries: " << std::endl;
          for (planning_interface::PlannerConfigurationMap::const_iterator map_it = config_map.begin();
               map_it != config_map.end() && !planner_exists; ++map_it)
            std::cout << map_it->second.name << std::endl;
          return false;
        }
      }
      //Handle the case of chomp planner as no configuration map exists, it is only one planner
      else
      {
        std::vector<std::string> planningAlgs;
        pm->getPlanningAlgorithms(planningAlgs);
        for (int j=0; j<planningAlgs.size(); j++)
        {
          planner_exists = (it->second[i] == planningAlgs[j]);
          if(planner_exists)
            return true;
        }
        ROS_ERROR("Planner '%s' does NOT exist for group '%s' in pipeline '%s'", it->second[i].c_str(),
                    group_name.c_str(), it->first.c_str());
        return false;
      }
    }
  }

  return true;
}

bool BenchmarkExecutor::loadPlanningScene(const std::string& scene_name, moveit_msgs::PlanningScene& scene_msg)
{
  bool ok = false;
  try
  {

    if (pss_->hasPlanningScene(scene_name))  // whole planning scene
    {
      moveit_warehouse::PlanningSceneWithMetadata pswm;
      ok = pss_->getPlanningScene(pswm, scene_name);
      scene_msg = static_cast<moveit_msgs::PlanningScene>(*pswm);

      if (!ok)
        ROS_ERROR("Failed to load planning scene '%s'", scene_name.c_str());
      planningScenePub_.publish(scene_msg);
    }
    else if (psws_->hasPlanningSceneWorld(scene_name))  // Just the world (no robot)
    {
      moveit_warehouse::PlanningSceneWorldWithMetadata pswwm;
      ok = psws_->getPlanningSceneWorld(pswwm, scene_name);
      scene_msg.world = static_cast<moveit_msgs::PlanningSceneWorld>(*pswwm);
      scene_msg.robot_model_name =
          "NO ROBOT INFORMATION. ONLY WORLD GEOMETRY";  // this will be fixed when running benchmark

      if (!ok)
        ROS_ERROR("Failed to load planning scene '%s'", scene_name.c_str());
      
    }
    else
      ROS_ERROR("Failed to find planning scene '%s'", scene_name.c_str());
  }
  catch (std::runtime_error& ex)
  {
    ROS_ERROR("Error loading planning scene: %s", ex.what());
  }
  ROS_INFO("Loaded planning scene successfully");

  return ok;
}

bool BenchmarkExecutor::loadQueries(const std::string& regex, const std::string& scene_name,
                                    std::vector<BenchmarkRequest>& queries)
{
  if (regex.empty())
    return true;

  std::vector<std::string> query_names;
  try
  {
    pss_->getPlanningQueriesNames(regex, query_names, scene_name);
  }
  catch (std::runtime_error& ex)
  {
    ROS_ERROR("Error loading motion planning queries: %s", ex.what());
    return false;
  }

  if (query_names.empty())
  {
    ROS_ERROR("Scene '%s' has no associated queries", scene_name.c_str());
    return false;
  }

  for (std::size_t i = 0; i < query_names.size(); ++i)
  {
    moveit_warehouse::MotionPlanRequestWithMetadata planning_query;
    try
    {
      pss_->getPlanningQuery(planning_query, scene_name, query_names[i]);
    }
    catch (std::runtime_error& ex)
    {
      ROS_ERROR("Error loading motion planning query '%s': %s", query_names[i].c_str(), ex.what());
      continue;
    }

    BenchmarkRequest query;
    query.name = query_names[i];
    query.request = static_cast<moveit_msgs::MotionPlanRequest>(*planning_query);
    queries.push_back(query);
  }
  ROS_INFO("Loaded queries successfully");
  return true;
}

bool BenchmarkExecutor::loadStates(const std::vector<std::string>& regex, std::vector<StartState>& start_states)
{
  for(int i=0;i<regex.size();i++)
  {
    if (regex[i].size())
    {
      boost::regex start_regex(regex[i]);
      std::vector<std::string> state_names;
      rs_->getKnownRobotStates(state_names);
      for (std::size_t i = 0; i < state_names.size(); ++i)
      {
        boost::cmatch match;
        if (boost::regex_match(state_names[i].c_str(), match, start_regex))
        {
          moveit_warehouse::RobotStateWithMetadata robot_state;
          try
          {
            if (rs_->getRobotState(robot_state, state_names[i]))
            {
              StartState start_state;
              start_state.state = moveit_msgs::RobotState(*robot_state);

              /*ROS_WARN_STREAM("Start State "<<start_state.state.joint_state.position[0]<<","<<start_state.state.joint_state.position[1]<<","<<start_state.state.joint_state.position[2]<<","
                <<start_state.state.joint_state.position[3]<<","<<start_state.state.joint_state.position[4]<<","<<start_state.state.joint_state.position[5]<<","<<start_state.state.joint_state.position[6]
                <<","<<start_state.state.joint_state.position[7]<<","<<start_state.state.joint_state.position[8]<<","<<start_state.state.joint_state.position[9]);
              ROS_WARN_STREAM("joint names: "<<start_state.state.joint_state.name[0]<<","<<start_state.state.joint_state.name[1]<<","<<start_state.state.joint_state.name[2]<<","<<start_state.state.joint_state.name[3]<<","<<
                start_state.state.joint_state.name[4]<<","<<start_state.state.joint_state.name[5]<<","<<start_state.state.joint_state.name[6]<<","<<start_state.state.joint_state.name[7]<<","<<
                start_state.state.joint_state.name[8]<<","<<start_state.state.joint_state.name[9]);*/
              start_state.name = state_names[i];
              start_states.push_back(start_state);
            }
          }
          catch (std::runtime_error& ex)
          {
            ROS_ERROR("Runtime error when loading state '%s': %s", state_names[i].c_str(), ex.what());
            continue;
          }
        }
      }

      if (start_states.empty())
        ROS_WARN("No stored states matched the provided start state regex: '%s'", regex[i].c_str());
    }
    ROS_INFO("Loaded states successfully");
  }
  return true;
}

bool BenchmarkExecutor::loadPathConstraints(const std::vector<std::string>& regex, std::vector<PathConstraints>& constraints)
{
   for(int i=0;i<regex.size();i++)
  {
    if (regex[i].size())
    {
      std::vector<std::string> cnames;
      cs_->getKnownConstraints(regex[i], cnames);

      for (std::size_t i = 0; i < cnames.size(); ++i)
      {
        moveit_warehouse::ConstraintsWithMetadata constr;
        try
        {
          if (cs_->getConstraints(constr, cnames[i]))
          {
            PathConstraints constraint;
            constraint.constraints.push_back(*constr);
            constraint.name = cnames[i];
            constraints.push_back(constraint);
          }
        }
        catch (std::runtime_error& ex)
        {
          ROS_ERROR("Runtime error when loading path constraint '%s': %s", cnames[i].c_str(), ex.what());
          continue;
        }
      }

      if (constraints.empty())
        ROS_WARN("No path constraints found that match regex: '%s'", regex[i].c_str());
      else
        ROS_INFO("Loaded path constraints successfully");
    }
  }
  return true;
}

bool BenchmarkExecutor::loadTrajectoryConstraints(const std::string& regex,
                                                  std::vector<TrajectoryConstraints>& constraints)
{
  if (regex.size())
  {
    std::vector<std::string> cnames;
    tcs_->getKnownTrajectoryConstraints(regex, cnames);

    for (std::size_t i = 0; i < cnames.size(); ++i)
    {
      moveit_warehouse::TrajectoryConstraintsWithMetadata constr;
      try
      {
        if (tcs_->getTrajectoryConstraints(constr, cnames[i]))
        {
          TrajectoryConstraints constraint;
          constraint.constraints = *constr;
          constraint.name = cnames[i];
          constraints.push_back(constraint);
        }
      }
      catch (std::runtime_error& ex)
      {
        ROS_ERROR("Runtime error when loading trajectory constraint '%s': %s", cnames[i].c_str(), ex.what());
        continue;
      }
    }

    if (constraints.empty())
      ROS_WARN("No trajectory constraints found that match regex: '%s'", regex.c_str());
    else
      ROS_INFO("Loaded trajectory constraints successfully");
  }
  return true;
}


void BenchmarkExecutor::runBenchmark(moveit_msgs::MotionPlanRequest request,
                                     const std::map<std::string, std::vector<std::string>>& planners, int runs)
{
  benchmark_data_.clear();
  
  fileInfo = "_";
  boost::posix_time::ptime boostTime = ros::Time::now().toBoost();
  std::string strTime = to_simple_string (boostTime); 
  fileInfo.append(strTime);
  fileInfo.append("_");
  fileInfo.append(planning_scene_->getName());
  fileInfo.append(".csv");
  std::replace(fileInfo.begin(), fileInfo.end(), ' ', '_');
  std::replace(fileInfo.begin(), fileInfo.end(), '/', '_');
  std::replace(fileInfo.begin(), fileInfo.end(), ':', '_');
    unsigned int num_planners = 0;
    for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end();
         ++it)
      num_planners += it->second.size();
    boost::progress_display progress(num_planners * runs, std::cout);

  for (int j = 0; j < runs; ++j)
  {
    PlannerBenchmarkData planner_data(runs);
    std::vector<BenchmarkRequest> queries;
    //For consistency Test add the connector before planning
    geometry_msgs::Pose connectorPose = request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0];
    connectorPose.position.y += 0.25;
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(3.14,0,0);
    connectorPose.orientation.x = q.x;
    connectorPose.orientation.y = q.y;
    connectorPose.orientation.z = q.z;
    connectorPose.orientation.w = q.w;
    
    //q = tf::createQuaternionMsgFromRollPitchYaw(1.57,0,0);
    request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].orientation = q;
    // Iterate through all planner plugins
    for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end();
         ++it)
    {
      // Iterate through all planners associated with the plugin
      for (std::size_t i = 0; i < it->second.size(); ++i)
      {
        // This container stores all of the benchmark data for this planner
        
        request.planner_id = it->second[i];
        if(request.goal_constraints[0].joint_constraints.size()>0)
            request.goal_constraints[0].joint_constraints.clear();
        if(request.planner_id=="stomp.STOMP" || request.planner_id=="chomp.CHOMP")
        {
          moveit_msgs::PositionConstraint position;
          moveit_msgs::OrientationConstraint orientation;
          
         
          
            if(request.goal_constraints[0].position_constraints.size()>0)
            {
              position = request.goal_constraints[0].position_constraints[0];
              //request.goal_constraints[k].position_constraints.clear();
              ROS_WARN_STREAM("Pos "<<position.constraint_region.primitive_poses[0].position.x
            <<","<<position.constraint_region.primitive_poses[0].position.y
            <<","<<position.constraint_region.primitive_poses[0].position.z);
           
            }
            if(request.goal_constraints[0].orientation_constraints.size()>0)
            {
              orientation = request.goal_constraints[0].orientation_constraints[0];
              //request.goal_constraints[k].orientation_constraints.clear();
              double roll,pitch,yaw;
              const tf::Quaternion q1(orientation.orientation.x,orientation.orientation.y,orientation.orientation.z,orientation.orientation.w);
              tf::Matrix3x3(q1).getRPY(roll,pitch,yaw);
              ROS_WARN_STREAM("Orientation "<<roll<<","<<pitch<<","<<yaw);
            }

            moveit_msgs::GetPositionIK::Request ik_req;
            moveit_msgs::GetPositionIK::Response ik_res;
            
            
            ik_req.ik_request.group_name = request.group_name;
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id="world";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = position.constraint_region.primitive_poses[0].position.x;
            pose.pose.position.y = position.constraint_region.primitive_poses[0].position.y;
            pose.pose.position.z = position.constraint_region.primitive_poses[0].position.z;
            ik_req.ik_request.ik_link_name = "jaw";
            pose.pose.orientation = orientation.orientation;
            ik_req.ik_request.pose_stamped = pose;
            ros::Duration dur(20,0);
            ik_req.ik_request.timeout = dur;
            ik_req.ik_request.robot_state = request.start_state;
            ik_req.ik_request.avoid_collisions=1;
            ik_req.ik_request.attempts=20;
            service_client.call(ik_req,ik_res);
            bool result = ik_res.error_code.val==ik_res.error_code.SUCCESS?1:0;
            ROS_INFO_STREAM("Result: "<<result<<", and the code value is "<<ik_res.error_code.val<<","<<ik_res.solution.joint_state);
            if(result)
            {
              for(int j=0;j<ik_res.solution.joint_state.name.size();j++)
              {
                moveit_msgs::JointConstraint joints;
                joints.joint_name = ik_res.solution.joint_state.name[j];
                joints.position = ik_res.solution.joint_state.position[j];
                request.goal_constraints[0].joint_constraints.push_back(joints);
              }

              /*request.goal_constraints[0].joint_constraints[0].position=1.54;
              request.goal_constraints[0].joint_constraints[1].position=-0.4;
              request.goal_constraints[0].joint_constraints[2].position=3.47;
              request.goal_constraints[0].joint_constraints[3].position=0.066;
              request.goal_constraints[0].joint_constraints[4].position=0.8;
              request.goal_constraints[0].joint_constraints[5].position=-0.5;
              request.goal_constraints[0].joint_constraints[6].position=-0.46;
              request.goal_constraints[0].joint_constraints[7].position=-0.8;*/
              ROS_WARN_STREAM(request.goal_constraints[0].joint_constraints.size()<<","<<request.goal_constraints[0].position_constraints.size()<<","
          <<request.goal_constraints[0].orientation_constraints.size());
            }
        }

        request.max_velocity_scaling_factor = 1;
        BenchmarkRequest query;
        query.request = request;
        queries.push_back(query);
        

        if (!queriesAndPlannersCompatible(queries, planners)){
          break;
        }

        //Only to solve the issue with attached collision objects and since the start state is same as current state
        moveit_msgs::PlanningScene scene_msg;
        planning_scene_->getPlanningSceneMsg(scene_msg);
        moveit_msgs::RobotState current;
        moveit::core::robotStateToRobotStateMsg(planning_scene_->getCurrentStateNonConst(),current);
        current.is_diff = true;
        request.start_state = current; 

       

       // Planner start events
        for (std::size_t j = 0; j < planner_start_fns_.size(); ++j)
          planner_start_fns_[j](request, planner_data);


        planning_interface::PlanningContextPtr context =
            planner_interfaces_[it->first]->getPlanningContext(planning_scene_, request);

            
          // Pre-run events
          for (std::size_t k = 0; k < pre_event_fns_.size(); ++k)
            pre_event_fns_[k](request);
          planning_interface::MotionPlanDetailedResponse mp_res;
          ros::WallTime start = ros::WallTime::now();
          bool solved = false;
          if(request.planner_id=="ompl.Mobile_Manip[RRTstarkConfigDefault]" || request.planner_id=="ompl.Mobile_Manip[PRMstarkConfigDefault]")
          {
            std::string progressDataFile = "/home/dyouakim/Documents/survey_data/progressOmplBenchmark_";
            progressDataFile.append(request.planner_id);
            progressDataFile.append("_");
            progressDataFile.append(std::to_string(j));
            
            std::shared_ptr<ompl_interface::ModelBasedPlanningContext> modelContext = std::dynamic_pointer_cast< ompl_interface::ModelBasedPlanningContext>(context);
            solved = modelContext->benchmark(request.allowed_planning_time,1,mp_res,progressDataFile);
          }
          else
          {
             // Solve problem
              solved = context->solve(mp_res);
          }

          double total_time = (ros::WallTime::now() - start).toSec();
          if(request.planner_id=="ompl.Mobile_Manip[RRTstarkConfigDefault]" || request.planner_id=="ompl.Mobile_Manip[PRMstarkConfigDefault]")
          {
            total_time-=10;
          }
          // Collect data
          start = ros::WallTime::now();

          // Post-run events
          for (std::size_t k = 0; k < post_event_fns_.size(); ++k)
          {
            post_event_fns_[k](request, mp_res, planner_data[j]);
          }

         

          std::string metricsName = "../Documents/survey_data/metrics_";
          metricsName.append(request.planner_id);
          metricsName.append(fileInfo);
          std::string distToObstName = "../Documents/survey_data/distToObst_";
          distToObstName.append(request.planner_id);
          distToObstName.append(fileInfo);
          std::string plannedTrajName = "../Documents/survey_data/plannedTrajectory_";
          plannedTrajName.append(request.planner_id);
          plannedTrajName.append(fileInfo);
          std::string eeName = "../Documents/survey_data/EE_";
          eeName.append(request.planner_id);
          eeName.append(fileInfo);
          std::string jointsDistName = "../Documents/survey_data/jointsDist_";
          jointsDistName.append(request.planner_id);
          jointsDistName.append(fileInfo);

          std::ofstream metricsFile, trajectoryFile, distToObstFile, eeFile, jointsDistFile;
          metricsFile.open(metricsName, std::ofstream::out | std::ofstream::app);
          trajectoryFile.open(plannedTrajName, std::ofstream::out | std::ofstream::app);
          distToObstFile.open(distToObstName, std::ofstream::out | std::ofstream::app);
          eeFile.open(eeName, std::ofstream::out | std::ofstream::app);
          jointsDistFile.open(jointsDistName, std::ofstream::out | std::ofstream::app);

          metricsFile<<j<<",";
          distToObstFile<<j<<",";
          trajectoryFile<<j<<std::endl;
          eeFile<<j<<std::endl;
          jointsDistFile<<"0"<<std::endl;
          collectMetrics(planner_data[j], mp_res, solved, total_time, metricsFile, distToObstFile, trajectoryFile, eeFile, jointsDistFile);
          double metrics_time = (ros::WallTime::now() - start).toSec();
          ROS_DEBUG("Spent %lf seconds collecting metrics", metrics_time);
          
          ++progress;
        }

        // Planner completion events
        for (std::size_t j = 0; j < planner_completion_fns_.size(); ++j)
          planner_completion_fns_[j](request, planner_data);

         
        benchmark_data_.push_back(planner_data);
      }
    }
}


/*void BenchmarkExecutor::collectProgressProperties(const base::Planner::PlannerProgressProperties &properties,
                                           const time::duration &timePerUpdate)
            {
                time::point timeStart = time::now();

                std::unique_lock<std::mutex> ulock(solvedFlag_);
                while (!solved_)
                {
                    if (solvedCondition_.wait_for(ulock, timePerUpdate) == std::cv_status::no_timeout)
                        return;
                    else
                    {
                        double timeInSeconds = time::seconds(time::now() - timeStart);
                        std::string timeStamp = std::to_string(timeInSeconds);
                        std::map<std::string, std::string> data;
                        data["time REAL"] = timeStamp;
                        for (const auto &property : properties)
                        {
                            data[property.first] = property.second();
                        }
                        runProgressData_.push_back(data);
                    }
                }
            }*/

void BenchmarkExecutor::updateCollisionMatrixForGraspingObject(std::string objectID)
{
  collision_detection::AllowedCollisionMatrix& acm = planning_scene_->getAllowedCollisionMatrixNonConst();
  acm.setEntry(objectID,false);
  acm.setEntry(objectID,"palm",true);
  acm.setEntry("wall",false);
  acm.setEntry("wall","palm",true);
  acm.setEntry("wall",objectID,true);
  //acm.setEntry("panel","arm_base_link_yaw",true);
  moveit_msgs::PlanningScene modifiedSceneMsg;
  planning_scene_->getPlanningSceneMsg(modifiedSceneMsg);
  acm.getMessage(modifiedSceneMsg.allowed_collision_matrix);
  modifiedSceneMsg.is_diff = true;
  planningScenePub_.publish(modifiedSceneMsg);

  std::stringstream ss;
  acm.print(ss);
  ROS_INFO_STREAM("IN Updated ACM: "<<ss.str().c_str());
}

void BenchmarkExecutor::collectMetrics(PlannerRunData& metrics,
                                       const planning_interface::MotionPlanDetailedResponse& mp_res, bool solved,
                                       double total_time, std::ofstream& metricsFile, std::ofstream& distToObstFile, std::ofstream& trajectoryFile, std::ofstream &eeFile, std::ofstream &jointsDistFile)
{
    
  moveit_msgs::DisplayTrajectory display;
  moveit_msgs::MotionPlanDetailedResponse resp_msg;
  
  
   metrics["time REAL"] = boost::lexical_cast<std::string>(total_time);
  metrics["solved BOOLEAN"] = boost::lexical_cast<std::string>(solved);

  if (solved)
  {
    mp_res.getMessage(resp_msg);
    display.trajectory = resp_msg.trajectory;
    display.trajectory_start = resp_msg.trajectory_start;
    displaypathPub_.publish(display);
    // Analyzing the trajectory(ies) geometrically
    double L = 0.0;           // trajectory length
    double clearance = 0.0;   // trajectory clearance (average)
    double smoothness = 0.0;  // trajectory smoothness (average)
    bool correct = true;      // entire trajectory collision free and in bounds

    double process_time = total_time;
    for (std::size_t j = 0; j < mp_res.trajectory_.size(); ++j)
    {
      correct = true;
      L = 0.0;
      clearance = 0.0;
      smoothness = 0.0;
      const robot_trajectory::RobotTrajectory& p = *mp_res.trajectory_[j];
      moveit_msgs::RobotTrajectory trajectorMsg;
      p.getRobotTrajectoryMsg(trajectorMsg);

      // compute path length
      for (std::size_t k = 1; k < p.getWayPointCount(); ++k){
        L += p.getWayPoint(k - 1).distance(p.getWayPoint(k));
        if(j == mp_res.trajectory_.size()-1)
        {
          for (std::size_t i = 0; i < 8; ++i)
          {
            double current = p.getWayPoint(k-1).getVariablePosition(i);
            double next = p.getWayPoint(k).getVariablePosition(i);
            //prismaic joint
            if(i<3)
              jointsDistFile<<fabs(next - current);
              //continuous
            else if (i==3)
            {
              double d = fmod(fabs(next - current), 2.0 * boost::math::constants::pi<double>());
              jointsDistFile<<(d > boost::math::constants::pi<double>()) ? 2.0 * boost::math::constants::pi<double>() - d : d;
            }
            //revolute
            else
              jointsDistFile<<fabs(next - current);
          
          if(i == 7)
            jointsDistFile<<std::endl;
          else
            jointsDistFile<<",";
          }
        }
      }

       // compute correctness and clearance
      collision_detection::CollisionRequest req;
      for (std::size_t k = 0; k < p.getWayPointCount(); ++k)
      {
        collision_detection::CollisionResult res;
        planning_scene_->checkCollisionUnpadded(req, res, p.getWayPoint(k));
        if (res.collision)
          correct = false;
        if (!p.getWayPoint(k).satisfiesBounds())
          correct = false;
        double d = planning_scene_->distanceToCollisionUnpadded(p.getWayPoint(k));
        if(j == mp_res.trajectory_.size()-1)
        {
          //save Waypoint joint values
          moveit::core::robotStateToStream(p.getWayPoint(k),trajectoryFile,false);
          //save ee equivalen value
          moveit_msgs::GetPositionFK::Request fk_req;
          moveit_msgs::GetPositionFK::Response fk_res;
          moveit::core::robotStateToRobotStateMsg(p.getWayPoint(k),fk_req.robot_state);

          fk_req.header.frame_id = "world";
          fk_req.header.stamp = ros::Time::now(); 

          fk_req.fk_link_names.push_back("jaw");

          fk_service_client.call(fk_req,fk_res);
          bool result = fk_res.error_code.val==fk_res.error_code.SUCCESS?1:0;
          if(result)
          {
            double roll,pitch, yaw;
            const tf::Quaternion q1(fk_res.pose_stamped[0].pose.orientation.x,fk_res.pose_stamped[0].pose.orientation.y,fk_res.pose_stamped[0].pose.orientation.z,fk_res.pose_stamped[0].pose.orientation.w);       
              tf::Matrix3x3(q1).getRPY(roll, pitch, yaw);

          eeFile<<fk_res.pose_stamped[0].pose.position.x<<","<<fk_res.pose_stamped[0].pose.position.y<<","<<fk_res.pose_stamped[0].pose.position.z<<","<<roll<<","<<pitch<<","<<yaw<<std::endl;
          }
        
          distToObstFile<<d;
          if(k==p.getWayPointCount()-1)
          {
            distToObstFile<<std::endl;
          }
          else
          {
            distToObstFile<<",";
          }
        }
        if (d > 0.0 )  // in case of collision, distance is negative
          clearance += d;
      }
      
      clearance /= (double)p.getWayPointCount();
      // compute smoothness
      if (p.getWayPointCount() > 2)
      {
        double a = p.getWayPoint(0).distance(p.getWayPoint(1));
        for (std::size_t k = 2; k < p.getWayPointCount(); ++k)
        {
          // view the path as a sequence of segments, and look at the triangles it forms:
          //          s1
          //          /\          s4
          //      a  /  \ b       |
          //        /    \        |
          //       /......\_______|
          //     s0    c   s2     s3
          //

          // use Pythagoras generalized theorem to find the cos of the angle between segments a and b
          double b = p.getWayPoint(k - 1).distance(p.getWayPoint(k));
          double cdist = p.getWayPoint(k - 2).distance(p.getWayPoint(k));
          double acosValue = (a * a + b * b - cdist * cdist) / (2.0 * a * b);
          if (acosValue > -1.0 && acosValue < 1.0)
          {
            // the smoothness is actually the outside angle of the one we compute
            double angle = (boost::math::constants::pi<double>() - acos(acosValue));
            //ROS_ERROR_STREAM("Angle is "<<angle);
            // and we normalize by the length of the segments
            double u = 2.0 * angle;  /// (a + b);
            smoothness += u * u;
            //ROS_ERROR_STREAM("Smoothness is "<<smoothness<<" for "<<mp_res.description_[j]);
          }
          a = b;
        }
        smoothness /= (double)p.getWayPointCount();
        //ROS_ERROR_STREAM("Final Smoothness is "<<smoothness);
      }
     


      metrics["path_" + mp_res.description_[j] + "_correct BOOLEAN"] = boost::lexical_cast<std::string>(correct);
      metrics["path_" + mp_res.description_[j] + "_length REAL"] = boost::lexical_cast<std::string>(L);
      metrics["path_" + mp_res.description_[j] + "_clearance REAL"] = boost::lexical_cast<std::string>(clearance);
      metrics["path_" + mp_res.description_[j] + "_smoothness REAL"] = boost::lexical_cast<std::string>(smoothness);
      metrics["path_" + mp_res.description_[j] + "_time REAL"] =
      boost::lexical_cast<std::string>(mp_res.processing_time_[j]);
      ROS_ERROR_STREAM("now is "<<mp_res.description_[j]);
      if(mp_res.description_.size()==1)
      {
        metricsFile<<process_time<<","<<solved<<","<<correct<<","<<L<<","<<clearance<<","<<smoothness<<","<<p.getWayPointCount()<<","<<mp_res.iterations_[j]<<","<<mp_res.cost_[j]<<std::endl;
      }
      else if((mp_res.description_.size()>1 && j == mp_res.trajectory_.size()-1))
      {
        double totalPlannTimeSampling = 0;
        for(int k=0;k<j;k++)
        {
          totalPlannTimeSampling += mp_res.processing_time_[k];
        }
        
        metricsFile<<totalPlannTimeSampling<<","<<solved<<","<<correct<<","<<L<<","<<clearance<<","<<smoothness<<","<<p.getWayPointCount()<<",";
        if(!mp_res.iterations_.empty())
        {
          metricsFile<<mp_res.iterations_[0]<<","<<mp_res.cost_[0]<<",";
        }
        else
        {
          metricsFile<<-1<<","<<-1<<",";
        }
        copy(mp_res.processing_time_.begin(), mp_res.processing_time_.end(), std::ostream_iterator<float>(metricsFile , ","));
      }
      process_time -= mp_res.processing_time_[j];
    }
    if (process_time <= 0.0)
      process_time = 0.0;
    metrics["process_time REAL"] = boost::lexical_cast<std::string>(process_time);
    metricsFile<<std::endl;
  }
}

void BenchmarkExecutor::writeOutput(const BenchmarkRequest& brequest, const std::string& start_time,
                                    double benchmark_duration)
{
  const std::map<std::string, std::vector<std::string>>& planners = options_.getPlannerConfigurations();

  size_t num_planners = 0;
  for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end();
       ++it)
    num_planners += it->second.size();

  std::string hostname = getHostname();
  if (hostname.empty())
    hostname = "UNKNOWN";

  std::string filename = options_.getOutputDirectory();
  if (filename.size() && filename[filename.size() - 1] != '/')
    filename.append("/");

  // Ensure directories exist
  boost::filesystem::create_directories(filename);

  filename += (options_.getBenchmarkName().empty() ? "" : options_.getBenchmarkName() + "_") + brequest.name + "_" +
              getHostname() + "_" + start_time + ".log";
  std::ofstream out(filename.c_str());
  if (!out)
  {
    ROS_ERROR("Failed to open '%s' for benchmark output", filename.c_str());
    return;
  }

  out << "MoveIt! version " << MOVEIT_VERSION << std::endl;
  out << "Experiment " << brequest.name << std::endl;
  out << "Running on " << hostname << std::endl;
  out << "Starting at " << start_time << std::endl;

  // Experiment setup
  moveit_msgs::PlanningScene scene_msg;
  planning_scene_->getPlanningSceneMsg(scene_msg);
  out << "<<<|" << std::endl;
  out << "Motion plan request:" << std::endl
      << brequest.request << std::endl;
  out << "Planning scene: " << std::endl
      << scene_msg << std::endl
      << "|>>>" << std::endl;

  // Not writing optional cpu information

  // The real random seed is unknown.  Writing a fake value
  out << "0 is the random seed" << std::endl;
  out << brequest.request.allowed_planning_time << " seconds per run" << std::endl;
  // There is no memory cap
  out << "-1 MB per run" << std::endl;
  out << options_.getNumRuns() << " runs per planner" << std::endl;
  out << benchmark_duration << " seconds spent to collect the data" << std::endl;

  // No enum types
  out << "0 enum types" << std::endl;

  out << num_planners << " planners" << std::endl;

  size_t run_id = 0;
  for (std::map<std::string, std::vector<std::string>>::const_iterator it = planners.begin(); it != planners.end();
       ++it)
  {
    for (std::size_t i = 0; i < it->second.size(); ++i, ++run_id)
    {
      // Write the name of the planner.
      out << it->second[i] << std::endl;

      // in general, we could have properties specific for a planner;
      // right now, we do not include such properties
      out << "0 common properties" << std::endl;

      // Create a list of the benchmark properties for this planner
      std::set<std::string> properties_set;
      for (std::size_t j = 0; j < benchmark_data_[run_id].size(); ++j)  // each run of this planner
        for (PlannerRunData::const_iterator pit = benchmark_data_[run_id][j].begin();
             pit != benchmark_data_[run_id][j].end(); ++pit)  // each benchmark property of the given run
          properties_set.insert(pit->first);

      // Writing property list
      out << properties_set.size() << " properties for each run" << std::endl;
      for (std::set<std::string>::const_iterator pit = properties_set.begin(); pit != properties_set.end(); ++pit)
        out << *pit << std::endl;

      // Number of runs
      out << benchmark_data_[run_id].size() << " runs" << std::endl;

      // And the benchmark properties
      for (std::size_t j = 0; j < benchmark_data_[run_id].size(); ++j)  // each run of this planner
      {
        // Write out properties in the order we listed them above
        for (std::set<std::string>::const_iterator pit = properties_set.begin(); pit != properties_set.end(); ++pit)
        {
          // Make sure this run has this property
          PlannerRunData::const_iterator runit = benchmark_data_[run_id][j].find(*pit);
          if (runit != benchmark_data_[run_id][j].end())
            out << runit->second;
          out << "; ";
        }
        out << std::endl;  // end of the run
      }
      out << "." << std::endl;  // end the planner
    }
  }

  out.close();
  ROS_INFO("Benchmark results saved to '%s'", filename.c_str());
}

void BenchmarkExecutor::planningSceneCallback(const moveit_msgs::PlanningSceneConstPtr& planningSceneUpdate)
{
  ROS_ERROR_STREAM("in planning scene callback");
  planning_scene_->setPlanningSceneDiffMsg(*planningSceneUpdate);
}


bool BenchmarkExecutor::addConnector(geometry_msgs::Pose objectPose, bool first)
{
  moveit_msgs::PlanningScene modifiedSceneMsg;
  planning_scene_->getPlanningSceneMsg(modifiedSceneMsg);


  moveit_msgs::CollisionObject connector;
  connector.id = "connector";
  connector.header.frame_id = "world";
  connector.header.stamp = ros::Time::now();
  /*if(first)
  {*/
    connector.operation = connector.ADD;

    Eigen::Vector3d scaling(0.1, 0.4, 0.1);
    shapes::Mesh* m = shapes::createMeshFromResource("package://g500_csip_benchmarks/objects/connector_10.stl",scaling);
    shapes::ShapeMsg mesh_msg;  
      shapes::constructMsgFromShape(m, mesh_msg);

      shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

      connector.meshes.push_back(mesh);
  /*}
  else
    connector.operation = connector.MOVE;*/

    connector.mesh_poses.push_back(objectPose);
  modifiedSceneMsg.world.collision_objects.push_back(connector);

  ROS_ERROR_STREAM("object pushed");
  modifiedSceneMsg.is_diff = true;
  planningScenePub_.publish(modifiedSceneMsg);
  planning_scene_->setPlanningSceneDiffMsg(modifiedSceneMsg);
  
    updateCollisionMatrixForGraspingObject("connector");
}