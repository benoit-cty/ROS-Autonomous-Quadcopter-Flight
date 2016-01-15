#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <pthread.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <action_controller/MultiDofFollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"
#include <math.h>

class Controller{
private:
	typedef actionlib::ActionServer<action_controller::MultiDofFollowJointTrajectoryAction> ActionServer;
	typedef ActionServer::GoalHandle GoalHandle;

	
// void depthCallback(const sensor_msgs::ImageConstPtr& original_image){
// }
public:
 	nav_msgs::Odometry last_position;
// 	void positionCallback(const nav_msgs::Odometry::ConstPtr& position){
	Controller(ros::NodeHandle &n) :
		node_(n),
		action_server_(node_, "multi_dof_joint_trajectory_action",
				boost::bind(&Controller::goalCB, this, _1),
				boost::bind(&Controller::cancelCB, this, _1),
				false),
				has_active_goal_(false)
{
		creato=0;
		empty.linear.x=0;
		empty.linear.y=0;
		empty.linear.z=0;
		empty.angular.z=0;
		empty.angular.y=0;
		empty.angular.x=0;
		pub_topic = node_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		
		action_server_.start();
		ROS_INFO_STREAM("Node ready!");
}
private:
	ros::NodeHandle node_;
	ActionServer action_server_;
	ros::Publisher pub_topic;
	geometry_msgs::Twist empty;
	geometry_msgs::Transform_<std::allocator<void> > lastPosition;
	geometry_msgs::Twist cmd;
	pthread_t trajectoryExecutor;
	int creato;
	
	bool has_active_goal_;
	GoalHandle active_goal_;
	trajectory_msgs::MultiDOFJointTrajectory_<std::allocator<void> > toExecute;

	void cancelCB(GoalHandle gh){
		if (active_goal_ == gh)
		{
			// Stops the controller.
			if(creato){
				ROS_INFO_STREAM("Stop thread");
				pthread_cancel(trajectoryExecutor);
				creato=0;
			}
			pub_topic.publish(empty);

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}
		ros::spinOnce();
	}

	void goalCB(GoalHandle gh){
		ROS_INFO_STREAM("controller received goal");
		
		if (has_active_goal_)
		{
			// Stops the controller.
			if(creato){
				pthread_cancel(trajectoryExecutor);
				creato=0;
			}
			pub_topic.publish(empty);

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}

		gh.setAccepted();
		active_goal_ = gh;
		has_active_goal_ = true;
		toExecute = gh.getGoal()->trajectory;

		//controllore solo per il giunto virtuale Base
		if(pthread_create(&trajectoryExecutor, NULL, threadWrapper, this)==0){
			creato=1;
			ROS_INFO_STREAM("Thread for trajectory execution created");
		} else {
			ROS_INFO_STREAM("Thread creation failed!");
		}
		ros::spinOnce();

	}

	static void* threadWrapper(void* arg) {
		Controller * mySelf=(Controller*)arg;
		mySelf->executeTrajectory();
		return NULL;
	}

	void executeTrajectory(){
		ROS_INFO_STREAM("Executing trajectory!");

		if(toExecute.joint_names[0]=="Base" && toExecute.points.size()>0){
			for(int k=0; k<toExecute.points.size(); k++){
				//ricavo cmd da effettuare
				geometry_msgs::Transform_<std::allocator<void> > punto=toExecute.points[k].transforms[0];
				bool eseguito=true;
				if(k!=0){
					eseguito=publishTranslationComand(punto,false);
					if(k==(toExecute.points.size()-1)){
						if(!eseguito) publishTranslationComand(punto,true);
						publishRotationComand(punto,false);
					}
				} else {
					publishRotationComand(punto,true);
				}
				pub_topic.publish(empty);
				//aggiorno start position
				if(eseguito){
					lastPosition.translation=punto.translation;
					lastPosition.rotation=punto.rotation;
				}
			}
		}
		active_goal_.setSucceeded();
		has_active_goal_=false;
		creato=0;
		
		ros::spinOnce();
	}
	inline double limitAngleRange(double angle){
		while(angle>M_PI)
			angle -= 2*M_PI;
		while(angle<=-M_PI)
			angle += 2*M_PI;
		return angle;
	}

	bool publishTranslationComand(geometry_msgs::Transform_<std::allocator<void> > punto, bool anyway){
		float Kc_linear = 1.0;
		float Kc_bearing = 1.0;
		
// 		ROS_INFO_STREAM("Translation");
		// Get the orienation of the quadrotor
		tf::Quaternion q;
		tf::quaternionMsgToTF(last_position.pose.pose.orientation,q);
		tf::Transform transf(q);
		
		// Get the relative position of the goal
		tf::Vector3 pos;
		pos.setX(punto.translation.x-last_position.pose.pose.position.x);
		pos.setY(punto.translation.y-last_position.pose.pose.position.y);
		pos.setZ(punto.translation.z-last_position.pose.pose.position.z);
		
		// Transform the relative position to the quadrotor frame
		tf::Vector3 relativePosition = transf.inverse()*(pos);

// 		ROS_INFO_STREAM("Goal Position: [" << punto.translation.x <<
// 				", "<< punto.translation.y <<
// 				", "<< punto.translation.z <<"] ");
// 		ROS_INFO_STREAM("Current position: [" << last_position.pose.pose.position.x << " " << last_position.pose.pose.position.y << " " << last_position.pose.pose.position.z << "]");
// 		ROS_INFO_STREAM("Relative Position in world frame: [" << pos.getX() <<
// 				", "<< pos.getY() <<
// 				", "<< pos.getZ() <<"] ");
// 		ROS_INFO_STREAM("Relative Position in quadrotor frame: [" << relativePosition.getX() <<
// 				", "<< relativePosition.getY() <<
// 				", "<< relativePosition.getZ() <<"] ");
		
		

		// Get the relative 
		cmd.linear.x=relativePosition.getX()*Kc_linear;
		cmd.linear.y=relativePosition.getY()*Kc_linear;
		cmd.linear.z=relativePosition.getZ()*Kc_linear;
		cmd.angular.x=cmd.angular.y=0;
	
		double yaw,pitch,roll;
		transf.getBasis().getEulerYPR(yaw,pitch,roll);
		double angle = atan2(pos.getY(),pos.getX());
		double bearing = limitAngleRange(angle - yaw);
		cmd.angular.z=bearing*Kc_bearing;

		if(anyway || cmd.linear.x>=0.5 || cmd.linear.y>=0.5 || cmd.linear.z>=0.5){
			printPositionInfo();
			printCmdInfo();
			pub_topic.publish(cmd);
			//tempo d'esecuzione
		ros::spinOnce();
			ros::Duration(1.0).sleep();
			return true;
		}
		return false;
	}

	void publishRotationComand(geometry_msgs::Transform_<std::allocator<void> > punto, bool start){
// 		ROS_INFO_STREAM("Rotation");
		//comando di allineamento, permesse solo rotazioni sull'asse z
		cmd.linear.x=cmd.linear.y=cmd.linear.z=cmd.angular.x=cmd.angular.y=0;
		//start = true --> devo tornare nell'orientazione 0
		//start = false --> devo arrivare al'orientazione punto.rotation.z
		cmd.angular.z=(start?0-punto.rotation.z:punto.rotation.z);

		printCmdInfo();

		double sleep=cmd.angular.z*3.0; //tempo necessario a tornare nella giusta orientazione
		if(sleep<0) sleep=-sleep;
		pub_topic.publish(cmd);
		ros::Duration(sleep).sleep();
		cmd.angular.z=0;
		ros::spinOnce();
	}

	void printPositionInfo(){
		ROS_INFO_STREAM("Start Position: ["<<last_position.pose.pose.position.x<<
				", "<<last_position.pose.pose.position.y<<
				", "<<last_position.pose.pose.position.z<<"] "<<
				"[ "<<lastPosition.rotation.x<<
				", "<<lastPosition.rotation.y<<
				", "<<lastPosition.rotation.z<<" ]");
		ros::spinOnce();
	}

	void printCmdInfo(){
		ROS_INFO_STREAM("cmd to execute: "<<"x:"<<cmd.linear.x
				<<" y: " << cmd.linear.y
				<<" z: " << cmd.linear.z
				<<" rX: " << cmd.angular.x
				<<" rY: " << cmd.angular.y
				<<" rZ: " << cmd.angular.z);
		ros::spinOnce();
	}
	

};

ros::Subscriber position_sub;
Controller *control;

	void positionCallback(const nav_msgs::Odometry::ConstPtr& position){
// 		ROS_INFO_STREAM("Got position!");
		control->last_position.pose.pose.position.x = position->pose.pose.position.x;
		control->last_position.pose.pose.position.y = position->pose.pose.position.y;
		control->last_position.pose.pose.position.z = position->pose.pose.position.z;
		control->last_position.pose.pose.orientation.x = position->pose.pose.orientation.x;
		control->last_position.pose.pose.orientation.y = position->pose.pose.orientation.y;
		control->last_position.pose.pose.orientation.z = position->pose.pose.orientation.z;
		control->last_position.pose.pose.orientation.w = position->pose.pose.orientation.w;
	}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "my_controller_node");
	ros::NodeHandle node;//("~");
	control = new Controller(node);
	ros::Subscriber position_sub = node.subscribe("/ground_truth/state", 1, positionCallback);

	ros::spin();

	return 0;
}
