#ifndef REACTIVE_CONTROLLER
#define REACTIVE_CONTROLLER

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/String.h"
#include <angles/angles.h>

#include <string>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>

#include "arm_controllers/ControllerJointState.h"

#define PI 3.141592
#define D2R PI/180.0
#define R2D 180.0/PI



namespace arm_controllers{

	class ReactiveController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
      private:
         int loop_count_;
         string cmd;

         // joint handles
         unsigned int n_joints_;
         std::vector<std::string> joint_names_;
         std::vector<hardware_interface::JointHandle> joints_;
         std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

         // kdl
         KDL::Tree 	kdl_tree_;
         KDL::Chain	kdl_chain_;
         boost::scoped_ptr<KDL::ChainDynParam> id_solver_;	// inverse dynamics solver
         KDL::JntArray G_;									// gravity torque vector
         KDL::Vector gravity_;

         // cmd, state
         realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;
         KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_;
         KDL::JntArray q_, qdot_;

         Eigen::VectorXd tau_cmd_, tau_fric_;
         Eigen::VectorXd q_error_, q_error_dot_;

         // gain
         std::vector<control_toolbox::Pid> pids_;       /**< Internal PID controllers. */

         // topic
         ros::Subscriber command_sub_;
         ros::Subscriber command_behavior;
         ros::NodeHandle n;
         boost::scoped_ptr<
            realtime_tools::RealtimePublisher<
               arm_controllers::ControllerJointState> > controller_state_pub_;

		public:
		~ReactiveController()
		{

		}

		bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  		{

  		}

		void starting(const ros::Time& time)
		{

		}

		void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
		{

		}

		void commandBH(const std_msgs::String::ConstPtr& msg)
		{

		}

  		void update(const ros::Time& time, const ros::Duration& period)
  		{



  		}

		void balancing(const ros::Time& time, const ros::Duration& period)
		{

		}

		void slowing(const ros::Time& time, const ros::Duration& period)
		{

		}

  		void stopping(const ros::Time& time) { }

		void enforceJointLimits(double &command, unsigned int index)
		{

		}

	};

}

PLUGINLIB_EXPORT_CLASS(arm_controllers::ReactiveController, controller_interface::ControllerBase)

#endif
