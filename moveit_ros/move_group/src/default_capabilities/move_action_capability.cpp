/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include "move_action_capability.h"

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/GetPositionFK.h>
#include <eigen_conversions/eigen_msg.h>

#include <smpl/debug/marker_utils.h>
#include <smpl/debug/colors.h>
#include <smpl/debug/marker.h>
#include <smpl/debug/visualize.h>

move_group::MoveGroupMoveAction::MoveGroupMoveAction() : MoveGroupCapability("MoveAction"), move_state_(IDLE)
{
}

void move_group::MoveGroupMoveAction::initialize()
{

  // start the move action server
  move_action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction>(
      root_node_handle_, MOVE_ACTION, boost::bind(&MoveGroupMoveAction::executeMoveCallback, this, _1), false));
  move_action_server_->registerPreemptCallback(boost::bind(&MoveGroupMoveAction::preemptMoveCallback, this));
  move_action_server_->start();
}

void move_group::MoveGroupMoveAction::executeMoveCallback(const moveit_msgs::MoveGroupGoalConstPtr &goal)
{
  setMoveState(PLANNING);
  context_->planning_scene_monitor_->updateFrameTransforms();

 

  moveit_msgs::MoveGroupResult action_res;
  if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
  {
    if (!goal->planning_options.plan_only)
      ROS_WARN("This instance of MoveGroup is not allowed to execute trajectories but the goal request has plan_only "
               "set to false. Only a motion plan will be computed anyway.");
    executeMoveCallback_PlanOnly(goal, action_res);
  }
  else
    executeMoveCallback_PlanAndExecute(goal, action_res);

  bool planned_trajectory_empty = trajectory_processing::isTrajectoryEmpty(action_res.planned_trajectory);
  std::string response =
      getActionResultString(action_res.error_code, planned_trajectory_empty, goal->planning_options.plan_only);
  if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    move_action_server_->setSucceeded(action_res, response);
  else
  {
    if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
      move_action_server_->setPreempted(action_res, response);
    else
      move_action_server_->setAborted(action_res, response);
  }
  setMoveState(IDLE);
}

void move_group::MoveGroupMoveAction::executeMoveCallback_PlanAndExecute(const moveit_msgs::MoveGroupGoalConstPtr &goal,
                                                                         moveit_msgs::MoveGroupResult &action_res)
{
  ROS_INFO("Combined planning and execution request received for MoveGroup action. Forwarding to planning and "
           "execution pipeline.");

  if (planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff))
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);

    
    const robot_state::RobotState &current_state = lscene->getCurrentState();

    // check to see if the desired constraints are already met
    for (std::size_t i = 0; i < goal->request.goal_constraints.size(); ++i)
      if (lscene->isStateConstrained(current_state,
                                     kinematic_constraints::mergeConstraints(goal->request.goal_constraints[i],
                                                                             goal->request.path_constraints)))
      {
        ROS_INFO("Goal constraints are already satisfied. No need to plan or execute any motions");
        action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        return;
      }
  }


  //Check if thr grid has changed resolution to replace the planning scene for this request
 

  /*if (grid_world->grid()->resolution()!=goal->request.grid_res)
  {
    moveit_msgs::PlanningScene scene_msg; 
    scene_nonconst->getPlanningSceneMsg(scene_msg);
    scene_nonconst->changeGridParams(goal->request.workspace_parameters,goal->request.grid_res,0.8,scene_msg.world);
    grid_world = dynamic_cast<  collision_detection::GridWorld*>(scene_nonconst->getWorldNonConst().get());
  }*/



  plan_execution::PlanExecution::Options opt;

  const moveit_msgs::MotionPlanRequest &motion_plan_request =
      planning_scene::PlanningScene::isEmpty(goal->request.start_state) ? goal->request :
                                          
                                                                          clearRequestStartState(goal->request);
  const moveit_msgs::PlanningScene &planning_scene_diff =
      planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff.robot_state) ?
          goal->planning_options.planning_scene_diff :
          clearSceneRobotState(goal->planning_options.planning_scene_diff);

  opt.replan_ = goal->planning_options.replan;
  opt.replan_attempts_ = goal->request.num_planning_attempts;//goal->planning_options.replan_attempts;
  opt.replan_delay_ = goal->planning_options.replan_delay;
  opt.before_execution_callback_ = boost::bind(&MoveGroupMoveAction::startMoveExecutionCallback, this);

  opt.plan_callback_ = boost::bind(&MoveGroupMoveAction::planUsingPlanningPipeline, this, boost::cref(motion_plan_request), _1);
  
  /*if(goal->planning_options.replan)
  {
    opt.repair_plan_callback_ = boost::bind(&MoveGroupMoveAction::repairPlan, this, boost::cref(motion_plan_request), _1, _2);
  }*/

  if (goal->planning_options.look_around && context_->plan_with_sensing_)
  {
    opt.plan_callback_ = boost::bind(&plan_execution::PlanWithSensing::computePlan, context_->plan_with_sensing_.get(),
                                     _1, opt.plan_callback_, goal->planning_options.look_around_attempts,
                                     goal->planning_options.max_safe_execution_cost);
    context_->plan_with_sensing_->setBeforeLookCallback(boost::bind(&MoveGroupMoveAction::startMoveLookCallback, this));
  }


  plan_execution::ExecutableMotionPlan plan;
  context_->plan_execution_->planAndExecute(plan, planning_scene_diff, opt);
  convertToMsg(plan.plan_components_, action_res.trajectory_start, action_res.planned_trajectory);
  if (plan.executed_trajectory_)
    plan.executed_trajectory_->getRobotTrajectoryMsg(action_res.executed_trajectory);
  action_res.error_code = plan.error_code_;
  int trials = 0;
  while (action_res.error_code.val== moveit_msgs::MoveItErrorCodes::CONTROL_FAILED && trials<10)
  {
    trials++;
    ROS_ERROR("Controller failed, goal aborted. Compute an Escape Point!");
    ROS_ERROR_STREAM("At the e#beginning with the result  "<<action_res.error_code.val);

    std::unique_ptr<sbpl::motion::BFS_3D> base_bfs;
    planning_scene::PlanningScene* scene_nonconst = const_cast <planning_scene::PlanningScene*> ((context_->planning_scene_monitor_.get())->getPlanningScene().get());
      
    collision_detection::GridWorld* grid_world = dynamic_cast<  collision_detection::GridWorld*>(scene_nonconst->getWorldNonConst().get());
  

    robot_state::RobotState& robot_state = scene_nonconst->getCurrentStateNonConst();


    std::vector<double> goal_config;
    robot_state::RobotState goal_state = robot_state;//plan.plan_components_.back().trajectory_->getLastWayPoint ();
    for(int i=0;i<goal_state.getVariableCount();i++)
    {
      goal_config.push_back(goal_state.getVariablePosition(goal_state.getVariableNames()[i]));
      ROS_ERROR_STREAM("Goal config ["<<i<<"] is "<<goal_config[i]);
    }

    
    moveit_msgs::GetPositionFK::Request fk_req;
    moveit_msgs::GetPositionFK::Response fk_res;
    ros::ServiceClient service_client = root_node_handle_.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
    fk_req.header.frame_id = "world";
    fk_req.header.stamp = ros::Time::now(); 
    fk_req.fk_link_names.push_back("jaw");
    
    moveit::core::robotStateToRobotStateMsg(robot_state,fk_req.robot_state);
    
    service_client.call(fk_req,fk_res);
    bool result = fk_res.error_code.val==fk_res.error_code.SUCCESS?1:0;
    
    double ee_x,ee_y,ee_z;

    ee_x = fk_res.pose_stamped[0].pose.position.x;
    ee_y = fk_res.pose_stamped[0].pose.position.y;
    ee_z = fk_res.pose_stamped[0].pose.position.z;


    const int xc = grid_world->grid()->numCellsX();
    const int yc = grid_world->grid()->numCellsY();
    const int zc = grid_world->grid()->numCellsZ();
    base_bfs.reset(new sbpl::motion::BFS_3D(xc, yc, zc));
    const int cell_count = xc * yc * zc;
    for (int x = 0; x < xc; ++x) {
      for (int y = 0; y < yc; ++y) {
        for (int z = 0; z < zc; ++z) {
            const double radius = 0.4;
            if (grid_world->grid()->getDistance(x, y, z) <= radius) {
                base_bfs->setWall(x, y, z);
            }
        }
      }
    }

    int gx, gy, gz;
    grid_world->grid()->worldToGrid(ee_x,ee_y,ee_z,
           //goal_config[0], goal_config[1], goal_config[2],
            gx, gy, gz);

   if (!base_bfs->inBounds(gx, gy, gz)) {
        ROS_ERROR_STREAM("Computing Escape Point: Heuristic goal is out of BFS bounds");
    }
    
    base_bfs->run(gx, gy, gz);
    
    std::vector<Eigen::Vector3d> centers;
    for (int x = 0; x < xc; x++) {
    for (int y = 0; y < yc; y++) {
    for (int z = 0; z < zc; z++) {
        if (base_bfs->isWall(x, y, z)) {
            Eigen::Vector3d p;
            grid_world->grid()->gridToWorld(x, y, z, p.x(), p.y(), p.z());
            centers.push_back(p);
        }
    }
    }
    }

   
    sbpl::visual::Color color;
    color.r = 100.0f / 255.0f;
    color.g = 149.0f / 255.0f;
    color.b = 238.0f / 255.0f;
    color.a = 1.0f;

    
    SV_SHOW_INFO_NAMED("bfs_walls_escape", sbpl::visual::MakeCubesMarker(
            centers,
            grid_world->grid()->resolution(),
            color,
            "/world",
            "bfs_walls_escape"));

    ROS_ERROR_STREAM("Here after showing heursitics!");


    double x,y,z;
    std::vector<double> costs;
    int min_idx  = -1;
    double min_cost = 99999;
    robot_state::RobotState best_escape = robot_state;
     ROS_ERROR_STREAM("Here after setting state1111d!");
    robot_state::RobotState current = robot_state;
     ROS_ERROR_STREAM("Here after setting state2222!");
    std::vector<robot_state::RobotState> computed_states;

     ROS_ERROR_STREAM("Here after setting state333!");
    double step = 0.2;
    int correction = 4;
    for(int i=-4;i<=4;i++)
    {
      if(i==0)
      {
        correction = 3;
        continue;
      }

      ROS_ERROR_STREAM("Here before generating random state! "<<step*i<<","<<(step/2)*i);
      //robot_state.setToRandomPositions();
     /* std::random_device rd;
      std::default_random_engine generator(rd());
      std::uniform_real_distribution<double> distribution(-1.0,1.0);*/
      float number = (float)rand() / (float)RAND_MAX; //distribution(generator);
      number = -1 + (2*number);
      robot_state.setVariablePosition(current.getVariableNames()[0], current.getVariablePosition(current.getVariableNames()[0]) + (step*i)); //(2*number));
      robot_state.setVariablePosition(current.getVariableNames()[1], current.getVariablePosition(current.getVariableNames()[1]) + (step*i));//(2*number));
      robot_state.setVariablePosition(current.getVariableNames()[2], current.getVariablePosition(current.getVariableNames()[2]) + ((step/2)*i));//(0.5*number));
      robot_state.setVariablePosition(current.getVariableNames()[3], current.getVariablePosition(current.getVariableNames()[3]) + ((step/2)*i));//(2*number));
      

      //robot_state.setVariablePosition(robot_state.getVariableNames()[3], goal_config[3]);
      robot_state.setVariablePosition(current.getVariableNames()[4], goal_config[4]);
      robot_state.setVariablePosition(current.getVariableNames()[5], goal_config[5]);
      robot_state.setVariablePosition(current.getVariableNames()[6], goal_config[6]);
      robot_state.setVariablePosition(current.getVariableNames()[7], goal_config[7]);

      x = robot_state.getVariablePosition(current.getVariableNames()[0]);
      y = robot_state.getVariablePosition(current.getVariableNames()[1]);
      z = robot_state.getVariablePosition(current.getVariableNames()[2]);

      
      if(scene_nonconst->isStateColliding(robot_state, goal->request.group_name, true))
      {
        ROS_ERROR_STREAM("Generated random config is in collision, try again!");
        //i--;
        continue;
      }
      
      ROS_ERROR_STREAM("before converting robot state to joint state");

      moveit::core::robotStateToRobotStateMsg(robot_state,fk_req.robot_state);
    
      service_client.call(fk_req,fk_res);
      bool result = fk_res.error_code.val==fk_res.error_code.SUCCESS?1:0;
    
      ROS_ERROR_STREAM("after calling FK!");

      Eigen::Vector3i dp;
      grid_world->grid()->worldToGrid(fk_res.pose_stamped[0].pose.position.x,fk_res.pose_stamped[0].pose.position.y,fk_res.pose_stamped[0].pose.position.z,
                                      dp.x(), dp.y(), dp.z());

      ROS_ERROR_STREAM("Random config["<<i+correction<<"] is "<<x<<","<<y<<","<<z<<","<<robot_state.getVariablePosition(robot_state.getVariableNames()[3]));
      ROS_ERROR_STREAM("The equivalent EE pose "<<fk_res.pose_stamped[0].pose.position.x<<","<<fk_res.pose_stamped[0].pose.position.y<<","<<fk_res.pose_stamped[0].pose.position.z);
      ROS_ERROR_STREAM("the grid values "<<dp.x()<<","<<dp.y()<<","<<dp.z());

      if (!base_bfs->inBounds(dp.x(), dp.y(), dp.z())) {
          //i--;
          ROS_ERROR_STREAM("Out of bound state !");
          continue;
      }
      else if (base_bfs->getDistance(x, y, z) == sbpl::motion::BFS_3D::WALL || base_bfs->getDistance(dp.x(), dp.y(), dp.z()) ==0) {
        //i--;
        ROS_ERROR_STREAM("Wall state !");
        continue;
      }
      else {
        computed_states.push_back(robot_state);
        
        ROS_ERROR_STREAM("Here before computing dist!");
        double dist = ((double)(base_bfs->getDistance(dp.x(), dp.y(), dp.z())))*((double)grid_world->grid()->resolution());
        costs.push_back(dist);
        ROS_ERROR_STREAM("Dist is "<<dist<<" and before "<<base_bfs->getDistance(dp.x(), dp.y(), dp.z())<<" res "<<grid_world->grid()->resolution());
      }

      ROS_ERROR_STREAM(" The cost of the state is "<<costs[i+correction]);
    }

    /*std::nth_element(costs.begin(), costs.begin() + costs.size() / 2, costs.end());

    min_cost = costs[costs.size() / 2];
    best_escape = computed_states[costs.size() / 2];
    ROS_ERROR_STREAM("The chosen config index & cost "<<costs.size() / 2<<","<<min_cost);*/

    std::sort(costs.begin(),costs.end());
    for(int i=0;i<costs.size();i++)
    {
      if(costs[i]<8*0.2 && costs[i]>=5*0.2 && costs[i]<min_cost)
      {
        min_cost = costs[i];
        min_idx = i;
        best_escape = computed_states[i];
        ROS_ERROR_STREAM("The chosen config index is "<<min_idx);
        break;
      }
    }

    
    if(min_idx==-1)
    {  
      min_idx = 0;
      min_cost = costs[0];
      best_escape = computed_states[0];
      ROS_ERROR_STREAM("None fullfill the condition, pick the min. The chosen config cost is "<<min_cost);
    }
    plan.plan_components_.back().trajectory_->clear();
    plan.plan_components_.back().trajectory_->addSuffixWayPoint(best_escape,1);

    plan_execution::PlanExecution::Options escape_opt = opt;
    plan_execution::ExecutableMotionPlan escapePlan;
    moveit_msgs::MoveItErrorCodes moveToEscapeResult, moveFromEscapeResult;
    

    moveit_msgs::MotionPlanRequest* escape_plan_request = const_cast <moveit_msgs::MotionPlanRequest*> (&motion_plan_request);
    escape_plan_request->request_id = motion_plan_request.request_id+1;
    moveit::core::robotStateToRobotStateMsg(best_escape,escape_plan_request->start_state);
    escape_opt.plan_callback_ = boost::bind(&MoveGroupMoveAction::planUsingPlanningPipeline, this, boost::cref(*escape_plan_request), _1);
    
    std::thread moveingToEscapePoint (&plan_execution::PlanExecution::moveToEscapePoint, context_->plan_execution_, std::ref(plan), std::ref(opt), std::ref(moveToEscapeResult));
    ROS_ERROR_STREAM("Moving to Escape Point started!");
    escapePlan.planning_scene_ = (context_->planning_scene_monitor_.get())->getPlanningScene();
    escapePlan.planning_scene_monitor_ = context_->planning_scene_monitor_;
    std::thread escapePointPlanning (escape_opt.plan_callback_, std::ref(escapePlan));
    ROS_ERROR_STREAM("Planning thread started!");
    
    moveingToEscapePoint.join();
    ROS_ERROR_STREAM("Moving to Escape Point done!");

    escapePointPlanning.join();
    ROS_ERROR_STREAM("Planning thread done!");


    ROS_ERROR_STREAM("BACK FROM BOTH THREAD!!!!!!!"<<escapePlan.error_code_.val<<","<<moveToEscapeResult.val);
    if(moveToEscapeResult.val == moveit_msgs::MoveItErrorCodes::SUCCESS && escapePlan.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_ERROR_STREAM("moving to escape point and planning to goal done, move to goal!");
      context_->plan_execution_->moveToEscapePoint(escapePlan,escape_opt,moveFromEscapeResult);
    }
    else
    {
      ROS_ERROR_STREAM("Something went wrong trying to move to the compute Escape Point!");
    }

    action_res.error_code = moveFromEscapeResult;
    ROS_ERROR_STREAM("After executing the plan the result is "<<moveFromEscapeResult.val);
  }  

}

void move_group::MoveGroupMoveAction::executeMoveCallback_PlanOnly(const moveit_msgs::MoveGroupGoalConstPtr &goal,
                                                                   moveit_msgs::MoveGroupResult &action_res)
{
  ROS_INFO("Planning request received for MoveGroup action. Forwarding to planning pipeline.");

  planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);  // lock the scene so that it
                                                                                            // does not modify the world
                                                                                            // representation while
                  
                                                                                   // diff() is called
  const planning_scene::PlanningSceneConstPtr &the_scene =
      (planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff)) ?
          static_cast<const planning_scene::PlanningSceneConstPtr &>(lscene) :
          lscene->diff(goal->planning_options.planning_scene_diff);

 planning_interface::MotionPlanResponse res;
 try
  {
    context_->planning_pipeline_->generatePlan(the_scene, goal->request, res);
  }
  catch (std::runtime_error &ex)
  {
    ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  catch (...)
  {
    ROS_ERROR("Planning pipeline threw an exception");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  convertToMsg(res.trajectory_, action_res.trajectory_start, action_res.planned_trajectory);
  action_res.error_code = res.error_code_;
  action_res.planning_time = res.planning_time_;
  if(res.error_code_.val== moveit_msgs::MoveItErrorCodes::SUCCESS)
    action_res.planned_trajectory.ee_pose = res.trajectory_->getEEPath();
}

bool move_group::MoveGroupMoveAction::planUsingPlanningPipeline(const planning_interface::MotionPlanRequest &req,
                                                                plan_execution::ExecutableMotionPlan &plan)
{
  setMoveState(PLANNING);
  planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);
  bool solved = false;
  planning_interface::MotionPlanResponse res;
  try
  {
    solved = context_->planning_pipeline_->generatePlan(plan.planning_scene_, req, res);
  }
  catch (std::runtime_error &ex)
  {
    ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  catch (...)
  {
    ROS_ERROR("Planning pipeline threw an exception");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  if (res.trajectory_)
  {
    plan.plan_components_.resize(1);
    plan.plan_components_[0].trajectory_ = res.trajectory_;
    plan.plan_components_[0].description_ = "plan";
  }
  plan.error_code_ = res.error_code_;
  return solved;
}

bool move_group::MoveGroupMoveAction::repairPlan(const planning_interface::MotionPlanRequest &req, plan_execution::ExecutableMotionPlan &plan, const std::pair<int, int>& invalidPointIdx)
{
  ROS_INFO_STREAM("INSIDE REPAIR!!!!!"<<std::get<0>(invalidPointIdx)<<","<<std::get<1>(invalidPointIdx));
      
  //added a new field in the motionplan request message
  //TODO maybe need to add more information and it is better to create another moveit_msgs
  planning_interface::MotionPlanRequest repairReq;
  repairReq = req;
  setMoveState(PLANNING);
  planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);

  int num = plan.planning_scene_monitor_->getPlanningScene()->getWorldNonConst().get()->getObjectIds().size();
  bool solved = false;
  planning_interface::MotionPlanResponse res;
  try
  {
    /*if(std::get<1>(invalidPointIdx)!=-1)
    {
      moveit::core::robotStateToRobotStateMsg(plan.planning_scene_monitor_->getPlanningScene()->getCurrentStateNonConst(),repairReq.start_state);
      const double* positions = plan.plan_components_.back().trajectory_->getWayPoint(std::get<1>(invalidPointIdx)).getVariablePositions();
      moveit_msgs::RobotState robot_state = repairReq.start_state;
      robot_state.joint_state.position[0]=positions[0];
      robot_state.joint_state.position[1]=positions[1];
      robot_state.joint_state.position[2]=positions[2];
      robot_state.joint_state.position[3]=positions[3];
      robot_state.joint_state.position[4]=positions[4];
      robot_state.joint_state.position[5]=positions[5];
      robot_state.joint_state.position[6]=positions[6];
      robot_state.joint_state.position[7]=positions[7];
      
      robot_state::RobotState rs = plan.planning_scene_monitor_->getPlanningScene()->getCurrentStateNonConst();
      robot_state::robotStateMsgToRobotState(robot_state, rs);
      geometry_msgs::PoseStamped fk_pose;
      tf::poseEigenToMsg(rs.getGlobalLinkTransform(repairReq.goal_constraints[0].position_constraints[0].link_name), fk_pose.pose);
      fk_pose.header.frame_id = repairReq.goal_constraints[0].position_constraints[0].header.frame_id;
      fk_pose.header.stamp = ros::Time::now();
      
      repairReq.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.x = fk_pose.pose.position.x;
      repairReq.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.y = fk_pose.pose.position.y;
      repairReq.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.z = fk_pose.pose.position.z;
      repairReq.repair_index = std::get<0>(invalidPointIdx);
    }*/
    moveit::core::robotStateToRobotStateMsg(plan.planning_scene_monitor_->getPlanningScene()->getCurrentStateNonConst(), repairReq.start_state);
    solved = context_->planning_pipeline_->repairPlan(plan.planning_scene_, repairReq, res);
  }
  catch (std::runtime_error &ex)
  {
    ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  catch (...)
  {
    ROS_ERROR("Planning pipeline threw an exception");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  if (res.trajectory_)
  {
    plan_execution::ExecutableTrajectory& lastPlanComponent = plan.plan_components_.back();
    robot_trajectory::RobotTrajectoryPtr meged_traj =  res.trajectory_;
    
    /*for(int i = std::get<1>(invalidPointIdx)+1; i<lastPlanComponent.trajectory_->getWayPointCount();i++)
      meged_traj->addSuffixWayPoint(lastPlanComponent.trajectory_->getWayPoint(i),lastPlanComponent.trajectory_->getWayPointDurationFromPrevious(i));
    */

    lastPlanComponent.trajectory_ = meged_traj;
  }
  plan.error_code_ = res.error_code_;
  return solved;
  
 /* else
  {
    repairReq.repair_index = -1;
    return planUsingPlanningPipeline(repairReq,plan);
  }*/
}

void move_group::MoveGroupMoveAction::startMoveExecutionCallback()
{
  setMoveState(MONITOR);
}

void move_group::MoveGroupMoveAction::startMoveLookCallback()
{
  setMoveState(LOOK);
}

void move_group::MoveGroupMoveAction::preemptMoveCallback()
{
  context_->plan_execution_->stop();
}

void move_group::MoveGroupMoveAction::setMoveState(MoveGroupState state)
{
  move_state_ = state;
  move_feedback_.state = stateToStr(state);
  move_action_server_->publishFeedback(move_feedback_);
}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupMoveAction, move_group::MoveGroupCapability)
