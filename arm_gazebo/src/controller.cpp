#include <functional>
#include "ros/ros.h"
#include "arm_lib/FK.h"
#include "arm_lib/IK.h"
#include "std_msgs/Float64.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <sstream>
#include "arm_lib/arm_joint_angles.h"
#include "arm_lib/JointPose.h"
#include "arm_lib/actuator_pos.h"
#include "arm_lib/control_cmd.h"

namespace gazebo
{
	class ModelPush : public ModelPlugin
	{
	

	public:
		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			this->model = _parent;
			this->jointController = this->model->GetJointController();
			this->jointList = model->GetJoints();

			this->pid = common::PID(32.0, 7.0, 5.0);

			for (int i = 0; i < jointList.size() - 4; i++)
			{
				jointController->SetPositionTarget(jointList[i]->GetScopedName(), 0);
				jointController->SetPositionPID(jointList[i]->GetScopedName(), this->pid);
			}

			this->pid = common::PID(19.0, 10.0, 10.0);


			jointController->SetPositionPID(model->GetJoint("palm_left_finger")->GetScopedName(), this->pid);
			jointController->SetPositionPID(model->GetJoint("palm_right_finger")->GetScopedName(), this->pid);

			this->jointController->SetPositionTarget(model->GetJoint("arm1_arm2_joint")->GetScopedName(), -0.4);
			this->jointController->SetPositionTarget(model->GetJoint("palm_left_finger")->GetScopedName(), -0.7);
			this->jointController->SetPositionTarget(model->GetJoint("palm_right_finger")->GetScopedName(), 0.7);

			
			this->init_node();

			/* -- Assignment 1
			this->init_publisher();
			this->init_subscriber();
			*/

			// this->init_fk_service(); -- to initialize FK service

			this->init_ik_service();
			this->init_ik_subscriber();
			this->init_cmd_subscriber();


			
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind(&ModelPush::OnUpdate, this));
		}

	public:
		void OnUpdate()
		{
			run_subscriber();
			
			//this->publishCurrentAngles(); -- Assignment 1

			// this->run_fk_service(); -- to run FK service
			
			
		}

		private:

		void init_node(){
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "robot_control");
		}
		void init_fk_service(){
  			ros::NodeHandle n;
  			this->srvClient = n.serviceClient<arm_lib::FK>("fk");
		}
 
		void init_ik_service(){
  			ros::NodeHandle n;
  			this->srvClient = n.serviceClient<arm_lib::IK>("ik");
		}


		void init_publisher(){
			ros::NodeHandle n;
			this->pub = n.advertise<arm_lib::arm_joint_angles>("joint_angles", 1000);
		}

		void init_subscriber(){
			ros::NodeHandle n_;
			this->sub = n_.subscribe("change_angles", 1000, &ModelPush::updateRobot, this);
		}

		void init_ik_subscriber(){
			ros::NodeHandle n_;
			this->sub = n_.subscribe("ik_angles", 1000, &ModelPush::run_ik_service, this);
		}

		void init_cmd_subscriber(){
			ros::NodeHandle n_;
			this->cmd_sub = n_.subscribe("control", 1000, &ModelPush::drag_drop, this);
		}
		

		void drag_drop(const arm_lib::control_cmd &msg){

			std::string command = msg.cmd;

			std::string lt = this->model->GetJoint("palm_left_finger")->GetScopedName();
			std::string rt = this->model->GetJoint("palm_right_finger")->GetScopedName();

			if (command == "catch")
			{
				ROS_INFO("Catching");

				this->jointController->SetPositionTarget(lt, 0.6);
				this->jointController->SetPositionTarget(rt, -0.6);
			}
			else if (command == "release")
			{
				ROS_INFO("Releasing");
				this->jointController->SetPositionTarget(lt, -0.7);
				this->jointController->SetPositionTarget(rt, 0.7);
			}
			this->jointController->Update();
		}

		void run_ik_service(const arm_lib::actuator_pos &msg){

			
			arm_lib::IK srv;
			srv.request.joint_positions = {0, 0, 0, 0, 0, 0};
			srv.request.actuator_pose = {msg.x, msg.y, msg.z};

			for (int i = 0; i < jointList.size() - 4; i++)
			{
				srv.request.joint_positions[i] = physics::JointState(jointList[i]).Position(0);
			}

			if ((this->srvClient).call(srv))
			{			
	  			ROS_INFO("Calling FK Service");
				ROS_INFO("Angles: [%f, %f, %f, %f, %f, %f]", srv.response.new_angles[0], srv.response.new_angles[1], srv.response.new_angles[2], srv.response.new_angles[3], srv.response.new_angles[4], srv.response.new_angles[5]);

				for (int i = 0; i < jointList.size() - 4; i++)
				{
					jointController->SetPositionTarget(jointList[i]->GetScopedName(), srv.response.new_angles[i]);
				}
			}
			else
			{
				ROS_ERROR("Failed to call IK service");
			}

		}
		

		void run_fk_service(){

			double z0 = physics::JointState(this->model->GetJoint("base_arm1_joint")).Position(0);

			double x1 = physics::JointState(this->model->GetJoint("arm1_arm2_joint")).Position(0);
			
			double x2 = physics::JointState(this->model->GetJoint("arm2_arm3_joint")).Position(0);

			double x3 = physics::JointState(this->model->GetJoint("arm3_arm4_joint")).Position(0);

			double x4 = physics::JointState(this->model->GetJoint("arm4_palm_joint")).Position(0);
			double y4 = physics::JointState(this->model->GetJoint("arm4_palm_joint")).Position(1);



			std::vector<double> angles = {(double)z0, (double)x1, (double)x2, (double)x3, (double)x4, (double)y4};
            // std::vector<double> angles = {0.2, 1.0, 0.6, 0.5, 1.0, 1.0, 0};
			arm_lib::FK srv;
			srv.request.joint_angles = angles;
			
			if ((this->srvClient).call(srv)){
				ROS_INFO("Calling FK Service");
				ROS_INFO("Pose: %f %f %f", srv.response.actuator_pose[0], srv.response.actuator_pose[1], srv.response.actuator_pose[2]);
   			
   			}
			else
			{
				ROS_ERROR("Failed to call FK service");
			}
		}

	

		void run_subscriber(){
			ros::spinOnce();
		}
		
		void updateRobot(const arm_lib::arm_joint_angles &msg){
			this->updateJointAngles(msg.z0, msg.x1, msg.x2, msg.x3);
		}
		
		void updateJoints(const arm_lib::JointPose &msg){
			this->updateJointAngles(msg.joint1, msg.joint2, msg.joint3, msg.joint4);
		}

		void updateJointAngles(double z0, double x1, double x2, double x3){

			ROS_INFO("%f %f %f %f", z0,x1,x2,x3);

			std::string base_arm1 = this->model->GetJoint("base_arm1_joint")->GetScopedName();
			std::string arm1_arm2 = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();
			std::string arm2_arm3 = this->model->GetJoint("arm2_arm3_joint")->GetScopedName();
			std::string arm3_arm4 = this->model->GetJoint("arm3_arm4_joint")->GetScopedName();

			// change to radian
			z0 = z0 * M_PI/ 180.0;
			x1 = x1 * M_PI/ 180.0;
			x2 = x2 * M_PI/ 180.0;
			x3 = x3 * M_PI/ 180.0;
			ROS_INFO("%f %f %f %f\n\n", z0,x1,x2,x3);

			this->jointController->SetPositionTarget(base_arm1, z0);

			this->jointController->SetPositionTarget(arm1_arm2, x1);

			this->jointController->SetPositionTarget(arm2_arm3, x2);

			this->jointController->SetPositionTarget(arm3_arm4, x3);


		}

		void publishCurrentAngles(){

			double z0 = physics::JointState(this->model->GetJoint("base_arm1_joint")).Position(0);

			double x1 = physics::JointState(this->model->GetJoint("arm1_arm2_joint")).Position(0);
			
			double x2 = physics::JointState(this->model->GetJoint("arm2_arm3_joint")).Position(0);

			double x3 = physics::JointState(this->model->GetJoint("arm3_arm4_joint")).Position(0);

			// change radian to degree
			z0 = z0 * 180.0 / M_PI;

			x1 = x1 * 180.0 / M_PI;

			x2 = x2 * 180.0 / M_PI;

			x3 = x3 * 180.0 / M_PI;

			arm_lib::arm_joint_angles current_angles;
			current_angles.z0  = z0;
			current_angles.x1  = x1;
			current_angles.x2  = x2;
			current_angles.x3  = x3;

			(this->pub).publish(current_angles);

			ros::spinOnce();

		}
		

	private:
		physics::ModelPtr model;

	private:
		physics::JointControllerPtr jointController;

	private:
		event::ConnectionPtr updateConnection;

	private:
		common::PID pid;

	private:
		ros::Publisher pub;
		ros::Subscriber sub;
		ros::Subscriber cmd_sub;
		physics::Joint_V jointList;
		


		ros::ServiceClient srvClient;

	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
