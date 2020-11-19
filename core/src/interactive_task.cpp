/*********************************************************************
 * Copyright (c) 2019 Bielefeld University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Artur Istvan Karoly
   Desc:   Class for creating pick-place/pick-hold tasks and their building blocks (pick object, lift object, place object, release object) in a modular fashion and 
	provide these planning capabilities as ros actions.
*/

#include <moveit/task_constructor/task.h>
#include <moveit/robot_model/robot_model.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/container.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <stages/dummy.h>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/PoseStamped.h>
#include <task_planning_msgs/AddPickAction.h>
#include <task_planning_msgs/AddPlaceAction.h>
#include <task_planning_msgs/AddReleaseAction.h>
#include <task_planning_msgs/ControlTaskAction.h>

#include <actionlib/server/simple_action_server.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <cmath>
#include <tf/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>

namespace interactive_task {
constexpr char LOGNAME[] = "interactive_task";
using namespace moveit::task_constructor;

// Class for the action servers
class Interactive_Task{
	public:

		// Start action servers when calling the constructor:
		Interactive_Task(): add_pick_server(Interactive_Task::nh, "add_pick", boost::bind(&Interactive_Task::add_pick_server_cb, this, _1), false),
		add_place_server(Interactive_Task::nh, "add_place", boost::bind(&Interactive_Task::add_place_server_cb, this, _1), false),
		add_release_server(Interactive_Task::nh, "add_release", boost::bind(&Interactive_Task::add_release_server_cb, this, _1), false),
		control_task_server(Interactive_Task::nh, "control_task", boost::bind(&Interactive_Task::control_task_server_cb, this, _1), false)
    	{
			add_pick_server.start();
			add_place_server.start();
			add_release_server.start();
			control_task_server.start();
			ROS_INFO_NAMED(LOGNAME, "Starting Interactive Task action servers");
    	}

		// Initialize internal parameters of the class
		void init();

		// Functions that are called to build the tasks
		std::unique_ptr<SerialContainer> Pick_Container(const std::string& object);
		std::unique_ptr<SerialContainer> Lift_Container(const std::string& object);
		std::unique_ptr<SerialContainer> Pick_and_Lift_Module(const std::string& object, bool this_is_start=true);
		std::unique_ptr<SerialContainer> Place_Container(const std::string& object, const geometry_msgs::PoseStamped& target_pose);
		std::unique_ptr<SerialContainer> Release_and_Retreat_Container(const std::string& object);
		std::unique_ptr<SerialContainer> Release_and_Retreat_Module(const std::string& object, bool this_is_start=true);
		std::unique_ptr<SerialContainer> Place_Module(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool this_is_start=true);
		/****************************************************
		 *                                                  *
		 *    Callback functions for the action servers     *
		 *                                                  *
		 ***************************************************/

		void add_pick_server_cb(const task_planning_msgs::AddPickGoalConstPtr& goal){
			bool this_is_start = false;

			if(!task_){
				task_.reset();
				task_.reset(new moveit::task_constructor::Task());
				this_is_start = true;
			}
			task_->reset();
			if (task_->stages()->numChildren() == 0){
				this_is_start = true;
			}
			if (this_is_start){
				task_->loadRobotModel();

				robot_model_ = task_->getRobotModel();
			}
			
			task_->add(Pick_and_Lift_Module(goal->object_name, this_is_start));

			std::stringstream ss;
			task_->printState(ss);
			std::string task_state = parse_task_state_string_stream(ss);

			add_pick_result.success = true;
			add_pick_result.task_state = task_state;

			add_pick_server.setSucceeded(add_pick_result);
		}

		void add_place_server_cb(const task_planning_msgs::AddPlaceGoalConstPtr& goal){
			bool this_is_start = false;

			if(!task_){
				task_.reset();
				task_.reset(new moveit::task_constructor::Task());
				this_is_start = true;
			}
			task_->reset();
			if (task_->stages()->numChildren() == 0){
				this_is_start = true;
			}
			if (this_is_start){
				task_->loadRobotModel();

				robot_model_ = task_->getRobotModel();
			}
			
			task_->add(Place_Module(goal->object_name, goal->object_target_pose, this_is_start));

			std::stringstream ss;
			task_->printState(ss);
			std::string task_state = parse_task_state_string_stream(ss);

			add_place_result.success = true;
			add_place_result.task_state = task_state;

			add_place_server.setSucceeded(add_place_result);
		}

		void add_release_server_cb(const task_planning_msgs::AddReleaseGoalConstPtr& goal){
			bool this_is_start = false;

			if(!task_){
				task_.reset();
				task_.reset(new moveit::task_constructor::Task());
				this_is_start = true;
			}
			task_->reset();
			if (task_->stages()->numChildren() == 0){
				this_is_start = true;
			}
			if (this_is_start){
				task_->loadRobotModel();

				robot_model_ = task_->getRobotModel();
			}
			
			task_->add(Release_and_Retreat_Module(goal->object_name, this_is_start));

			std::stringstream ss;
			task_->printState(ss);
			std::string task_state = parse_task_state_string_stream(ss);

			add_release_result.success = true;
			add_release_result.task_state = task_state;

			add_release_server.setSucceeded(add_release_result);
		}

		void control_task_server_cb(const task_planning_msgs::ControlTaskGoalConstPtr& goal){

			bool success = false;
			moveit_task_constructor_msgs::Solution sol;
			control_task_result.success = success;
			control_task_result.solution = sol;
			control_task_result.task_state = "";

			if (!task_){
				control_task_result.success = true;
				control_task_result.task_state = "Empty task";
				if (goal->operation == goal->SET_NAME || goal->operation == goal->PLAN){
					control_task_result.success = false;
				}
				control_task_server.setSucceeded(control_task_result);
			} else {
				task_->reset();
				if (goal->operation == goal->CLEAR){
					task_->clear();
					sampling_planner = std::make_shared<solvers::PipelinePlanner>();
					sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
					control_task_result.success = true;
				} else if (goal->operation == goal->SET_NAME){
					task_->stages()->setName(goal->data);
					control_task_result.success = true;
				} else if (goal->operation == goal->GET_STATUS){
					control_task_result.success = true;
				} else if (goal->operation == goal->PLAN){
					try{
						success = task_->plan(std::stoi(goal->data));
						control_task_result.success = success;
						if (success && task_->numSolutions() != 0){
							ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
							task_->solutions().front()->fillMessage(sol);
							control_task_result.solution = sol;
						} else {
							ROS_INFO_NAMED(LOGNAME, "Planning failed");
							control_task_result.success = false;
						}
					} catch (const moveit::task_constructor::InitStageException& ex) {
						std::cerr << "Exception during initializing the stages" << std::endl << ex;
						control_task_server.setAborted(control_task_result, "Exception during initializing the stages");
					} catch (const std::invalid_argument& ia) {
						std::cerr << "Invalid argument in action's data field. Expected an integer convertible string: " << ia.what() << std::endl;
						control_task_server.setAborted(control_task_result, "Invalid argument in action's data field. Expected an integer convertible string");
					} catch (const std::runtime_error& e) {
						std::cerr << "Runtime error during the planning of the task: " << e.what() << std::endl;
						control_task_server.setAborted(control_task_result, "Runtime error during the planning of the task");
					} catch (...) {
						std::cerr <<  "Unexpected error happened during the planning of the task";
						control_task_server.setAborted(control_task_result, "Unexpected error happened during the planning of the task");
					}
				}

				std::stringstream ss;
				task_->printState(ss);
				std::string task_state = parse_task_state_string_stream(ss);
				control_task_result.task_state = task_state;

				control_task_server.setSucceeded(control_task_result);
			}
		}

	private:
		ros::NodeHandle nh;

		// Action servers and result messages
		actionlib::SimpleActionServer<task_planning_msgs::AddPickAction> add_pick_server;
		actionlib::SimpleActionServer<task_planning_msgs::AddPlaceAction> add_place_server;
		actionlib::SimpleActionServer<task_planning_msgs::AddReleaseAction> add_release_server;
		actionlib::SimpleActionServer<task_planning_msgs::ControlTaskAction> control_task_server;
		task_planning_msgs::AddPickResult add_pick_result;
		task_planning_msgs::AddPlaceResult add_place_result;
		task_planning_msgs::AddReleaseResult add_release_result;
		task_planning_msgs::ControlTaskResult control_task_result;

		// The task
		moveit::task_constructor::TaskPtr task_;

		// Planners
		moveit::task_constructor::solvers::CartesianPathPtr cartesian_planner;
		moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner;

		// Planning Groups
		std::string group;
		std::string hand_group_name;

		// Frames
		std::string hand_frame;
		std::string eef_name;

		// Robot Model
		moveit::core::RobotModelConstPtr robot_model_;

		// Internal stage pointer for hooks
		Stage* current_state_stage;
		Stage* attach_object_stage;
		Stage* lift_object_stage;

		// Support surfaces
		std::vector<std::string> support_surfaces;

		// Transformation between grasp frame and robot hand frame
		Eigen::Isometry3d grasp_frame_transform;

		/****************************************************
		 *                                                  *
		 *             Internal functions                   *
		 *                                                  *
		 ***************************************************/

		std::string parse_task_state_string_stream(std::stringstream& stream){
			std::string line;
			std::string task_state = "";
			std::string stage_name;
			std::string whitespaces;
			std::string dashes = "";

			int line_count = 0;
			int whitespace_num = 0;
			while(getline(stream, line)){
				whitespaces = line.substr(0, line.find("-"));
				if (line_count == 0){
					whitespace_num = whitespaces.length();
				}
				for (int i=0; i < (whitespaces.length()-whitespace_num)/2; i++){
					dashes+="-";
				}
				stage_name = dashes + line.substr(line.find("/") + 2, line.length()-(line.find("/") + 2));
				task_state += stage_name + '\n';
				dashes = "";
				line_count++;
			}
			return task_state;
		}
};

void Interactive_Task::init(){
	// Initializing internal parameters of the class

	// Load required params from the param server
	ROS_INFO_NAMED(LOGNAME, "Initializing Interactive Task");
	ros::NodeHandle pnh("~");

	size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_group_name", group);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "eef_name", eef_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_group_name", hand_group_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_frame", hand_frame);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "support_surfaces", support_surfaces);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "grasp_frame_transform", grasp_frame_transform);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

	// Init Cartesian planner
	cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScaling(1.0);
	cartesian_planner->setMaxAccelerationScaling(1.0);
	cartesian_planner->setStepSize(.01);

	// Init sampling planner
	sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

	// Internal stage pointer for hooks
	current_state_stage = nullptr;
	attach_object_stage = nullptr;
	lift_object_stage = nullptr;

	ROS_INFO_NAMED(LOGNAME, "Initialization finished!\nReady to start planning ...");
}

std::unique_ptr<SerialContainer> Interactive_Task::Pick_Container(const std::string& object){
	auto c = std::make_unique<SerialContainer>("Grasp '" + object + "' with " + group);

	/****************************************************
	 *                                                  *
	 *               Open Hand                          *
	 *                                                  *
	 ***************************************************/
	{  // Open Hand
		auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
		stage->setGroup(hand_group_name);
		stage->setGoal("open");
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Move to Pick                       *
	 *                                                  *
	 ***************************************************/
	{  // Move-to pre-grasp
		auto stage = std::make_unique<stages::Connect>(
		    "move to pick", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Approach object                    *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
		stage->properties().set("marker_ns", "approach_object");
		stage->properties().set("link", hand_frame);
		stage->properties().set("group", group);
		stage->setMinMaxDistance(0.1, 0.15);

		// Set hand forward direction
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = hand_frame;
		vec.vector.z = 1.0;
		stage->setDirection(vec);
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Generate Grasp Pose                *
	 *                                                  *
	 ***************************************************/
	{

		auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
		stage->properties().set("marker_ns", "grasp_pose");
		stage->setPreGraspPose("open");
		stage->setObject(object);
		stage->setAngleDelta(M_PI / 12);
		stage->setMonitoredStage(current_state_stage);  // Hook into current state
		stage->setEndEffector(eef_name);

		// Compute IK
		auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
		wrapper->setMaxIKSolutions(8);
		wrapper->setMinSolutionDistance(1.0);
		wrapper->setIKFrame(grasp_frame_transform, hand_frame);
		wrapper->properties().set("group", group);
		wrapper->setEndEffector(eef_name);
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		c->insert(std::move(wrapper));
	}

	/****************************************************
	 *                                                  *
	 *          Allow Collision (hand object)           *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
		stage->allowCollisions(
			object, robot_model_->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
			true);
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *                  Close Hand                      *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
        stage->properties().set("group", hand_group_name);
		stage->setGoal("close");
		c->insert(std::move(stage));
	}

	return c;
}

std::unique_ptr<SerialContainer> Interactive_Task::Lift_Container(const std::string& object){
	auto c = std::make_unique<SerialContainer>("Lift '" + object + "' with " + group);

	/****************************************************
	 *                                                  *
	 *                 Attach Object                    *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
		stage->attachObject(object, hand_frame);
		attach_object_stage = stage.get();
		c->insert(std::move(stage));
	}


	/****************************************************
	 *                                                  *
	 *       Allow collision (object support)           *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
		stage->allowCollisions({ object }, support_surfaces, true);
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *                  Lift Object                     *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
		stage->properties().set("group", group);
		stage->setMinMaxDistance(0.1, 0.15);
		stage->setIKFrame(hand_frame);
		stage->properties().set("marker_ns", "lift_object");

		// Set upward direction
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = "world";
		vec.vector.z = 1;
		stage->setDirection(vec);
		c->insert(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *        Forbid collision (object support)         *
	 *                                                  *
	 ***************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
		stage->allowCollisions({ object }, support_surfaces, false);
		lift_object_stage = stage.get();
		c->insert(std::move(stage));
	}

	return c;
}

std::unique_ptr<SerialContainer> Interactive_Task::Pick_and_Lift_Module(const std::string& object, bool this_is_start){
	
	auto c = std::make_unique<SerialContainer>("Pick '" + object + "' with " + group);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	{
		std::unique_ptr<moveit::task_constructor::stages::PredicateFilter> applicability_filter;
		auto _current_state = std::make_unique<stages::CurrentState>("'before pick' state");
		auto _dummy_state = std::make_unique<stages::Dummy>("'before pick' state");
		_dummy_state->restrictDirection(PropagatingEitherWay::FORWARD);
	
		// Verify that object is not attached
		if (this_is_start){
			applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
		} else {
			applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(_dummy_state));
		}
		applicability_filter->setPredicate([object](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				comment = "object with id '" + object + "' is already attached and cannot be picked";
				return false;
			}
			return true;
		});

		current_state_stage = applicability_filter.get();
		c->insert(std::move(applicability_filter));
	}

	// Pick
	c->insert(std::move(Interactive_Task::Pick_Container(object)));


	// Lift
	c->insert(std::move(Interactive_Task::Lift_Container(object)));

	return c;
}

std::unique_ptr<SerialContainer> Interactive_Task::Place_Container(const std::string& object, const geometry_msgs::PoseStamped& target_pose){
	auto c = std::make_unique<SerialContainer>("Approach target pose for '" + object + "' with " + group);

	/******************************************************
	 *                                                    *
	 *          Lower Object                              *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
		stage->properties().set("marker_ns", "lower_object");
		stage->properties().set("link", hand_frame);
		stage->properties().set("group", group);
		stage->setMinMaxDistance(.03, .13);

		// Set downward direction
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = "world";
		vec.vector.z = -1;
		stage->setDirection(vec);
		current_state_stage = stage.get();
		c->insert(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Generate Place Pose                       *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
		stage->properties().set("marker_ns", "place_pose");
		stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
		stage->setObject(object);

		stage->setPose(target_pose);
		stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

		// Compute IK
		auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
		wrapper->setMaxIKSolutions(2);
		wrapper->setIKFrame(grasp_frame_transform, hand_frame);
		wrapper->properties().set("group", group);
		wrapper->setEndEffector(eef_name);
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		c->insert(std::move(wrapper));
	}
	return c;
}

std::unique_ptr<SerialContainer> Interactive_Task::Release_and_Retreat_Container(const std::string& object){
	auto c = std::make_unique<SerialContainer>("Release '" + object + "' and retreat, " + group);

	// /******************************************************
	//  *                                                    *
	//  *                  Open Hand                         *
	//  *                                                    *
	//  *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
        stage->properties().set("group", hand_group_name);
		stage->setGoal("open");
		c->insert(std::move(stage));
	}

	// /******************************************************
	//  *                                                    *
	//  *         Forbid collision (hand, object)            *
	//  *                                                    *
	//  *****************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
		stage->allowCollisions(
			object,
			robot_model_->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(), false);
		c->insert(std::move(stage));
	}

	// /******************************************************
	//  *                                                    *
	//  *                 Detach Object                      *
	//  *                                                    *
	//  *****************************************************/
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
		stage->detachObject(object, hand_frame);
		c->insert(std::move(stage));
	}

	// /******************************************************
	//  *                                                    *
	//  *                Retreat Motion                      *
	//  *                                                    *
	//  *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
		stage->setMinMaxDistance(.12, .25);
		stage->setIKFrame(hand_frame);
		stage->properties().set("marker_ns", "retreat");
		stage->properties().set("group", group);
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = hand_frame;
		vec.vector.z = -1;
		stage->setDirection(vec);
		c->insert(std::move(stage));
	}

	return c;
}

std::unique_ptr<SerialContainer> Interactive_Task::Release_and_Retreat_Module(const std::string& object, bool this_is_start){
	std::string tem = hand_frame;
	
	auto c = std::make_unique<SerialContainer>("Release '" + object + "' and retreat, " + group);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	{
		std::unique_ptr<moveit::task_constructor::stages::PredicateFilter> applicability_filter;
		auto _current_state = std::make_unique<stages::CurrentState>("current state");
		auto _dummy_state = std::make_unique<stages::Dummy>("current state");
		_dummy_state->restrictDirection(PropagatingEitherWay::FORWARD);

		// Verify that object is attached to the given planning group
		if (this_is_start){
			applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
		} else {
			applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(_dummy_state));
		}
		applicability_filter->setPredicate([object, tem](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				if (s.start()->scene()->getCurrentState().getAttachedBody(object)->getAttachedLinkName() == tem){
					return true;
				} else {
					comment = "object with id '" + object + "' is attached to a link other than the hand frame of this group";
					return false;
				}
			} else {
				comment = "object with id '" + object + "' is not attached so cannot be released";
				return false;
			}
		});

		c->insert(std::move(applicability_filter));
	}

	// Release and retreat
	c->insert(std::move(Interactive_Task::Release_and_Retreat_Container(object)));

	return c;

}

std::unique_ptr<SerialContainer> Interactive_Task::Place_Module(const std::string& object, const geometry_msgs::PoseStamped& target_pose, bool this_is_start){

	std::string tem = hand_frame;
	
	auto c = std::make_unique<SerialContainer>("Place '" + object + "' with " + group);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	{
		std::unique_ptr<moveit::task_constructor::stages::PredicateFilter> applicability_filter;
		auto _current_state = std::make_unique<stages::CurrentState>("current state");
		auto _dummy_state = std::make_unique<stages::Dummy>("current state");
		_dummy_state->restrictDirection(PropagatingEitherWay::FORWARD);

		// Verify that object is attached
		if (this_is_start){
			applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
		} else {
			applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(_dummy_state));
		}
		applicability_filter->setPredicate([object, tem](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				if (s.start()->scene()->getCurrentState().getAttachedBody(object)->getAttachedLinkName() == tem){
					return true;
				} else {
					comment = "object with id '" + object + "' is attached to a link other than the hand frame of this group";
					return false;
				}
			} else {
				comment = "object with id '" + object + "' is not attached and cannot be placed";
				return false;
			}
		});

		attach_object_stage = applicability_filter.get();
		c->insert(std::move(applicability_filter));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Place                             *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { group, sampling_planner } });
		stage->setTimeout(5.0);
		c->insert(std::move(stage));
	}

	// Place
	c->insert(std::move(Interactive_Task::Place_Container(object, target_pose)));

	return c;
}

}

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object) {
	if (!psi.applyCollisionObject(object))
		throw std::runtime_error("Failed to spawn object: " + object.id);
}

moveit_msgs::CollisionObject createObject() {
	ros::NodeHandle pnh("~");
	std::vector<double> object_dimensions = {0.25, 0.02};
	geometry_msgs::Pose pose;

	moveit_msgs::CollisionObject object;
	object.id = "object";
	object.header.frame_id = "world";
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	object.primitives[0].dimensions = object_dimensions;
	pose.position.x = 0.5;
	pose.position.y = -0.25;
	pose.position.z = 0.5 * object_dimensions[0];
	pose.orientation.w = 1.0;
	object.primitive_poses.push_back(pose);
	return object;
}

moveit_msgs::CollisionObject createTable() {
	ros::NodeHandle pnh("~");
	std::vector<double> table_dimensions = {0.4, 0.5, 0.1};
	geometry_msgs::Pose pose;

	moveit_msgs::CollisionObject object;
	object.id = "table";
	object.header.frame_id = "world";
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	object.primitives[0].dimensions = table_dimensions;
	pose.position.x = 0.5;
	pose.position.y = -0.25;
	pose.position.z -= 0.5 * table_dimensions[2];  // align surface with world
	pose.orientation.w = 1.0;
	object.primitive_poses.push_back(pose);
	return object;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "interactive_task");

	interactive_task::Interactive_Task IT;
	IT.init();

	// run an asynchronous spinner to communicate with the move_group node and rviz
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Duration(1.0).sleep();
	moveit::planning_interface::PlanningSceneInterface psi;
	ros::NodeHandle pnh("~");
	spawnObject(psi, createTable());
	spawnObject(psi, createObject());

	ros::waitForShutdown();  // keep alive for interactive inspection in rviz
	return 0;
}
