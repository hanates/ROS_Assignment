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

			// // intiantiate the joint controller
			this->jointController = this->model->GetJointController();

			// // set your PID values
			this->pid = common::PID(30.1, 10.01, 10.03);

			auto joint_name = "arm0_arm1_joint";

			std::string name = this->model->GetJoint("arm0_arm1_joint")->GetScopedName();

			this->jointController->SetPositionPID(name, pid);

			std::string name1 = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();

			this->jointController->SetPositionPID(name1, pid);

			

			
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "angle_publisher");
			ros::NodeHandle n;
			this->pub = n.advertise<arm_lib::arm_joint_angles>("joint_angles", 1000);
		
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

			// Get joint position by index. 
			// 0 returns rotation accross X axis
			// 1 returns rotation accross Y axis
			// 2 returns rotation accross Z axis
			// If the Joint has only Z axis for rotation, 0 returns that value and 1 and 2 return nan

			double x0 = physics::JointState(this->model->GetJoint("base_arm0_joint")).Position(0);
			double y0 = physics::JointState(this->model->GetJoint("base_arm0_joint")).Position(1);
			double z0 = physics::JointState(this->model->GetJoint("base_arm0_joint")).Position(2);
			// double z1 = this->model->GetJoint("base_arm0_joint")->GetAngle(2).Radian();

			double x1 = physics::JointState(this->model->GetJoint("arm0_arm1_joint")).Position(0);
			double y1 = physics::JointState(this->model->GetJoint("arm0_arm1_joint")).Position(1);
			double z1 = physics::JointState(this->model->GetJoint("arm0_arm1_joint")).Position(2);



			double x2 = physics::JointState(this->model->GetJoint("arm1_arm2_joint")).Position(0);

			double x3 = physics::JointState(this->model->GetJoint("arm2_arm3_joint")).Position(0);

			// change radian to degree
			x0 = x0 * 180.0 / M_PI;
			z0 = z0 * 180.0 / M_PI;

			x1 = x1 * 180.0 / M_PI;

			x2 = x2 * 180.0 / M_PI;

			x3 = x3 * 180.0 / M_PI;

			arm_lib::arm_joint_angles current_angles;
			current_angles.x0  = x0;
			current_angles.z0  = z0;
			current_angles.x1  = x1;
			current_angles.x2  = x2;
			current_angles.x3  = x3;

			// prints x0 nan nan ???
			std::cout << "Current arm0_arm1_joint values: " << x0 << " "<< y0 << " " << z0 << " "<< std::endl;
			(this->pub).publish(current_angles);

			ros::spinOnce();
			
		}

		// Update joint angles
		private:
		void updateJointAngles(double x0, double z0, double x1, double x2, double x3){

			std::string base_arm0 = this->model->GetJoint("base_arm0_joint")->GetScopedName();
			std::string arm0_arm1 = this->model->GetJoint("arm0_arm1_joint")->GetScopedName();
			std::string arm1_arm2 = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();
			std::string arm2_arm3 = this->model->GetJoint("arm2_arm3_joint")->GetScopedName();

			this->jointController->SetJointPosition(base_arm0, x0, 0);
			this->jointController->SetJointPosition(base_arm0, z0, 2);

			this->jointController->SetJointPosition(arm0_arm1, x1, 0);

			this->jointController->SetJointPosition(arm1_arm2, x2, 0);

			this->jointController->SetJointPosition(arm2_arm3, x3, 0);


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
		
		
		// std::unique_ptr<ros::NodeHandle> rosNode;
		ros::Publisher pub;
		

	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}