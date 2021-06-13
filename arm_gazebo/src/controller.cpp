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
			this->pid = common::PID(30, 10, 10);

			std::string name1 = this->model->GetJoint("base_arm1_joint")->GetScopedName();

			this->jointController->SetPositionPID(name1, pid);

			std::string name2 = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();

			this->jointController->SetPositionPID(name2, pid);

			std::string name3 = this->model->GetJoint("arm2_arm3_joint")->GetScopedName();

			this->jointController->SetPositionPID(name3, pid);

			std::string name4 = this->model->GetJoint("arm3_arm4_joint")->GetScopedName();

			this->jointController->SetPositionPID(name4, pid);

			std::string name5 = this->model->GetJoint("arm4_palm_joint")->GetScopedName();

			this->jointController->SetPositionPID(name5, pid);

			std::string name6 = this->model->GetJoint("palm_jnt_joint")->GetScopedName();

			this->jointController->SetPositionPID(name6, pid);

			std::string name7 = this->model->GetJoint("palm_left_finger")->GetScopedName();
			
			this->jointController->SetPositionPID(name7, pid);

			std::string name8 = this->model->GetJoint("left_finger_tip_joint")->GetScopedName();
			
			this->jointController->SetPositionPID(name8, pid);

			std::string name9 = this->model->GetJoint("palm_right_finger")->GetScopedName();
			
			this->jointController->SetPositionPID(name9, pid);
			
			std::string name10 = this->model->GetJoint("right_finger_tip_joint")->GetScopedName();
				
			this->jointController->SetPositionPID(name10, pid);


			
			this->init_node();
			//this->init_publisher();
			//this->init_subscriber();
			// this->sub_chatter();

			// this->init_fk_service();
			this->init_ik_service();
			
		
			
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind(&ModelPush::OnUpdate, this));
		}

		// Called by the world update start event
	public:
		void OnUpdate()
		{
			
			//this->publishCurrentAngles();
			// this->run_subscriber();

			// this->run_fk_service();
			
			this->run_ik_service();
			
			
			// this->updateJointAngles(1.0 + this->x,10.0,10.0,30.0);
			// (this->x)+=0.5;
			

			
		}

		// Update joint angles
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

		void catch_box(){

			std::string palm_lt_fng = this->model->GetJoint("palm_left_finger")->GetScopedName();
			std::string palm_lt_tip = this->model->GetJoint("left_finger_tip_joint")->GetScopedName();
			std::string palm_rt_fng = this->model->GetJoint("palm_right_finger")->GetScopedName();
			std::string palm_rt_tip = this->model->GetJoint("right_finger_tip_joint")->GetScopedName();
			std::string arm1_arm2 = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();



			int rad = 0.5;

			this->jointController->SetPositionTarget(palm_lt_fng, -rad);
			this->jointController->SetPositionTarget(palm_lt_tip, -rad);
			this->jointController->SetPositionTarget(palm_rt_fng, rad);
			this->jointController->SetPositionTarget(palm_rt_tip, rad);
			// this->jointController->SetPositionTarget(arm1_arm2, rad);

			this->jointController->Update();


		}

		void run_ik_service(){

			if(this->ik_srv != 1){
				arm_lib::IK srv;

				std::string base_arm1 = this->model->GetJoint("base_arm1_joint")->GetScopedName();
				std::string arm1_arm2 = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();
				std::string arm2_arm3 = this->model->GetJoint("arm2_arm3_joint")->GetScopedName();
				std::string arm3_arm4 = this->model->GetJoint("arm3_arm4_joint")->GetScopedName();
				std::string arm4_palm = this->model->GetJoint("arm4_palm_joint")->GetScopedName();
				std::string palm_jnt = this->model->GetJoint("palm_jnt_joint")->GetScopedName();
				std::string palm_lt_fng = this->model->GetJoint("palm_left_finger")->GetScopedName();
				std::string palm_lt_tip = this->model->GetJoint("left_finger_tip_joint")->GetScopedName();
				std::string palm_rt_fng = this->model->GetJoint("palm_right_finger")->GetScopedName();
				std::string palm_rt_tip = this->model->GetJoint("right_finger_tip_joint")->GetScopedName();
				
				// std::string jnt_tip1 = this->model->GetJoint("arm4_palm_joint")->GetScopedName();
				
				srv.request.actuator_pose = {1.6, 1.6, 0};
				
				if ((this->srvClient).call(srv)){
					ROS_INFO("Calling IK Service");
					ROS_INFO(" %f", srv.response.new_angles[0]);
					ROS_INFO(" %f", srv.response.new_angles[1]);
					ROS_INFO(" %f", srv.response.new_angles[2]);

					// this part is not setting the joint angles??
					this->jointController->SetPositionTarget(base_arm1, srv.response.new_angles[0]);
					this->jointController->SetPositionTarget(arm1_arm2, srv.response.new_angles[1]);
					this->jointController->SetPositionTarget(arm2_arm3, srv.response.new_angles[2]);
					this->jointController->SetPositionTarget(arm3_arm4, srv.response.new_angles[3]);
					this->jointController->SetPositionTarget(arm4_palm, srv.response.new_angles[4]);
					this->jointController->SetPositionTarget(palm_jnt, srv.response.new_angles[5]);
					this->jointController->SetPositionTarget(palm_lt_fng, srv.response.new_angles[6]);
					this->jointController->SetPositionTarget(palm_lt_tip, srv.response.new_angles[7]);
					this->jointController->SetPositionTarget(palm_rt_fng, srv.response.new_angles[6]);
					this->jointController->SetPositionTarget(palm_rt_tip, srv.response.new_angles[7]);
					this->jointController->Update();

					this->ik_srv = 1;

					// this->catch_box();
				}

			}
	
		}
		

		void run_fk_service()
   		{
			
			
			double z0 = physics::JointState(this->model->GetJoint("base_arm1_joint")).Position(0);

			double x1 = physics::JointState(this->model->GetJoint("arm1_arm2_joint")).Position(0);
			
			double x2 = physics::JointState(this->model->GetJoint("arm2_arm3_joint")).Position(0);

			double x3 = physics::JointState(this->model->GetJoint("arm3_arm4_joint")).Position(0);

			double x4 = physics::JointState(this->model->GetJoint("arm4_palm_joint")).Position(0);
			double y4 = physics::JointState(this->model->GetJoint("arm4_palm_joint")).Position(1);



			std::vector<float> angles = {(float)z0, (float)x1, (float)x2, (float)x3, (float)x4, (float)y4};

			arm_lib::FK srv;
			srv.request.joint_angles = angles;
			
			if ((this->srvClient).call(srv)){
				ROS_INFO("Calling FK Service");
				ROS_INFO("Pose: %f %f %f", srv.response.actuator_pose[0], srv.response.actuator_pose[1], srv.response.actuator_pose[2]);
   			
   			}
		}

	

		void init_publisher(){
			ros::NodeHandle n;
			this->pub = n.advertise<arm_lib::arm_joint_angles>("joint_angles", 1000);
		}

		void init_subscriber(){
			ros::NodeHandle n_;
			this->sub = n_.subscribe("change_angles", 1000, &ModelPush::updateRobot, this);
		}
		
		void run_subscriber(){
			ros::spinOnce();
		}
		
		void updateRobot(const arm_lib::arm_joint_angles &msg){
			this->updateJointAngles(msg.z0, msg.x1, msg.x2, msg.x3);
		}
		
		void updateJoints(const arm_lib::JointPose &msg){
			printf("here");
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
			printf("here1");

			this->jointController->SetPositionTarget(base_arm1, z0);

			this->jointController->SetPositionTarget(arm1_arm2, x1);

			this->jointController->SetPositionTarget(arm2_arm3, x2);

			this->jointController->SetPositionTarget(arm3_arm4, x3);


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
		

		// a pointer that points to a model object
	private:
		physics::ModelPtr model;

		// A joint controller object
		// Takes PID value and apply angular velocity
		//  or sets position of the angles
	private:
		physics::JointControllerPtr jointController;

	private:
		event::ConnectionPtr updateConnection;

	private:
		common::PID pid;

	private:
		ros::Publisher pub;
		ros::Subscriber sub;

		ros::ServiceClient srvClient;
		float x;
		int ik_srv = 0;
		

	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
