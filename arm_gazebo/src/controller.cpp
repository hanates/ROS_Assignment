#include <functional>
#include "ros/ros.h"
// #include "arm_lib/arm_joint_angles.h"
#include "std_msgs/Float64.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <sstream>
#include "arm_lib/arm_joint_angles.h"

namespace gazebo
{
	class ModelPush : public ModelPlugin
	{
	

	public:
		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			// Store the pointer to the model
			this->model = _parent;

			// // instiantiate the joint controller
			this->jointController = this->model->GetJointController();

			// // set your PID values
			this->pid = common::PID(30.1, 10.01, 10.03);

			std::string name = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();

			this->jointController->SetPositionPID(name, pid);

			std::string name1 = this->model->GetJoint("arm2_arm3_joint")->GetScopedName();

			this->jointController->SetPositionPID(name1, pid);
			
			this->init_publisher();
			// this->init_subscriber();
		
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind(&ModelPush::OnUpdate, this));
		}

		// Called by the world update start event
	public:
		void OnUpdate()
		{
			// 	float angleDegree = -90;
			// 	float rad = M_PI * angleDegree / 180;

			// this->jointController->SetPositionPID(name, pid);
			// this->jointController->SetPositionTarget(name, rad);
			// this->jointController->Update();

			this->publishCurrentAngles();
			// this->run_subscriber();
			
		}

		

		// Update joint angles
		private:

		void init_publisher(){
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "angle_publisher");
			ros::NodeHandle n;
			this->pub = n.advertise<arm_lib::arm_joint_angles>("joint_angles", 1000);
		}

		void init_subscriber(){
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "angle_changer");
			ros::NodeHandle n_;
			this->sub = n_.subscribe("change_angles", 1000, &ModelPush::updateRobot, this);
		}

		void run_subscriber(){
			ros::spinOnce();
		}
		
		void updateRobot(const arm_lib::arm_joint_angles &msg){
			this->updateJointAngles(msg.z0, msg.x1, msg.x2, msg.x3);
		}

		void updateJointAngles(double z0, double x1, double x2, double x3){

			std::string base_arm1 = this->model->GetJoint("base_arm1_joint")->GetScopedName();
			std::string arm1_arm2 = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();
			std::string arm2_arm3 = this->model->GetJoint("arm2_arm3_joint")->GetScopedName();
			std::string arm3_arm4 = this->model->GetJoint("arm3_arm4_joint")->GetScopedName();

			// change to radian
			z0 = z0 * M_PI/ 180.0;
			x1 = x1 * M_PI/ 180.0;
			x2 = x2 * M_PI/ 180.0;
			x3 = x3 * M_PI/ 180.0;

			this->jointController->SetJointPosition(base_arm1, z0, 0);

			this->jointController->SetJointPosition(arm1_arm2, x1, 0);

			this->jointController->SetJointPosition(arm2_arm3, x2, 0);

			this->jointController->SetJointPosition(arm3_arm4, x3, 0);


		}

		void publishCurrentAngles(){

			// Get joint position by index. 
			// 0 returns rotation accross X axis
			// 1 returns rotation accross Y axis
			// 2 returns rotation accross Z axis
			// If the Joint has only Z axis for rotation, 0 returns that value and 1 and 2 return nan
			double z0 = physics::JointState(this->model->GetJoint("base_arm1_joint")).Position(0);

			double x1 = physics::JointState(this->model->GetJoint("arm1_arm2_joint")).Position(0);
			
			double x2 = physics::JointState(this->model->GetJoint("arm2_arm3_joint")).Position(0);

			double x3 = physics::JointState(this->model->GetJoint("arm3_arm4_joint")).Position(0);

			// change to radian to degree
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
		

		// a pointer that points to a model object
	private:
		physics::ModelPtr model;

		// 	// A joint controller object
		// 	// Takes PID value and apply angular velocity
		// 	//  or sets position of the angles
	private:
		physics::JointControllerPtr jointController;

	private:
		event::ConnectionPtr updateConnection;

		// // 	// PID object
	private:
		common::PID pid;

	private:
		ros::Publisher pub;
		ros::Subscriber sub;
		

	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}