#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <bstars_navigation/GoToWaypointsAction.h>
#include <fstream>
#include <ros/package.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <costmap_2d/costmap_2d.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionServer<bstars_navigation::GoToWaypointsAction> WaypointsServer;

class WaypointsAction
{
public:
  MoveBaseClient *ac;
  ros::NodeHandle n_;
  WaypointsServer as_;

  WaypointsAction(std::string name) :
    as_(n_, name, boost::bind(&WaypointsAction::executeCB, this, _1), false),
    action_name_(name)
  {
    ac = new MoveBaseClient("move_base", true);
    initialized_ = false;
    current_goal_index_ = 0; // set as 0 by default, but loadGoalIndex overwrites
    as_.start();
  }

  ~WaypointsAction()
  {
    delete ac;
  }

  void init()
  {
    while(!ac->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    initialized_ = true;
  }

  // load the waypoints file
  // waypoints in file will be in form of x y yaw (separated by spaces)
  int loadWaypoints(std::string wp_file_name)
  {
    std::string line;
    std::string path = ros::package::getPath("bstars_navigation");
    path = path + "/param/" + wp_file_name;
    std::ifstream wp_file(path.c_str());
    if(wp_file.is_open())
    {
      while(std::getline(wp_file, line))
      {
        //std::cout << line << "\n";
        std::vector<float> row;
        std::string buf;
        std::stringstream ss(line);

        // get each value from a line, separated by spaces
        // TODO: this should check if there are actually 3 entries
        while(ss >> buf)
          row.push_back(std::stof(buf));

        goal_vec_.push_back(row);
      }
      wp_file.close();
    }
    else
    {
      std::cout << "Could not find param/waypoints.txt in bstars_navigation \n";
      return 0;
    }
    return 1;
  }

  // go to all waypoints, starting at current_goal_index_
  int goToWaypoints()
  {
    if (!initialized_)
    {
      ROS_INFO("waypoints_node not initialized. stopping.");
      return 0;
    }

    while(current_goal_index_ < goal_vec_.size() && ros::ok())
    {
      // check to see if the waypoints should be reset
      bool reset_param;
      if (ros::param::get("~reset_index", reset_param))
      {
        if(reset_param)
        {
          resetGoalIndex();
          ros::param::set("~reset_index", false);
        }
      }
      else
      {
        ros::param::set("~reset_index", false);
      }

      // check if preempt has been requested
      if (as_.isPreemptRequested())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        return 0;
      }

      // set the waypoint goal
      move_base_msgs::MoveBaseGoal goal = vecToMoveBaseGoal(goal_vec_[current_goal_index_]);
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      ROS_INFO("Sending x, y, yaw goal of %f, %f, %f",
               goal.target_pose.pose.position.x,
               goal.target_pose.pose.position.y,
               goal_vec_[current_goal_index_][2]);
      ac->sendGoal(goal);
      ROS_INFO("Goal index of %d successfully sent", current_goal_index_); //without this message, it doesn't always run

      // TODO: add in a delay or similar here to ensure that this doesn't mess up anymore

      ac->waitForResult();
      if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("woop woop");
        feedback_.consecutive_failed_pts = 0;
      }
      else
      {
        ROS_WARN("aw raspberries, didn't get to the point");
        feedback_.consecutive_failed_pts++;
      }
      feedback_.current_waypoint = current_goal_index_;
      as_.publishFeedback(feedback_);

      if(ros::ok()) saveCurrentGoalIndex(); //crazy bug without this if statement

      current_goal_index_++;
    }

    if (!ros::ok()) return 0;
    else return 1; // only gets here if all waypoints are completed
  }

  // convert the x y yaw vector to a MoveBaseGoal (poseStamped)
  move_base_msgs::MoveBaseGoal vecToMoveBaseGoal(std::vector<float> vec)
  {
    move_base_msgs::MoveBaseGoal out;
    out.target_pose.pose.position.x = vec[0];
    out.target_pose.pose.position.y = vec[1];

    tf::Quaternion q;
    q.setRPY(0., 0., vec[2]);
    tf::quaternionTFToMsg(q, out.target_pose.pose.orientation);

    return out;
  }

  // save the current goal index to a text file
  int saveCurrentGoalIndex()
  {
    std::string path = ros::package::getPath("bstars_navigation");
    path = path + "/param/saved_goal_index.txt";
    std::ofstream wp_index_file(path.c_str(), std::fstream::out | std::fstream::trunc);
    if(wp_index_file.is_open() && ros::ok())
    {
      wp_index_file << current_goal_index_;
      wp_index_file.close();
    }
    else
    {
      ROS_WARN("Could not open param/saved_goal_index.txt in bstars_navigation");
      return 0;
    }
    return 1;
  }

  int loadGoalIndex()
  {
    std::string line;
    std::string path = ros::package::getPath("bstars_navigation");
    path = path + "/param/saved_goal_index.txt";
    std::ifstream wp_index_file(path.c_str());
    if(wp_index_file.is_open())
    {
      while(std::getline(wp_index_file, line))
      {
        current_goal_index_ = std::stoi(line);
      }
      wp_index_file.close();
    }
    else
    {
      ROS_WARN("Could not find param/saved_goal_index.txt in bstars_navigation");
      return 0;
    }
    ROS_INFO("Initial goal index set to %d.", current_goal_index_);
    return 1;
  }

  void executeCB(const bstars_navigation::GoToWaypointsGoalConstPtr &goal)
  {
    init();

    std::string wp_file = goal->waypoints_file;
    bool wp_load_success = loadWaypoints(wp_file);
    bool wp_ind_load_success = loadGoalIndex();

    if(wp_load_success && wp_ind_load_success)
    {
      int success = goToWaypoints();
      if (success == 1)
      {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded();
      }
      else
      {
        ROS_WARN("%s: Failed", action_name_.c_str());
        as_.setAborted();
      }
    }
    else
    {
      ROS_ERROR("Waypoints files not correctly loaded.");
    }
  }

  // reset the goal index
  int resetGoalIndex()
  {
    current_goal_index_ = 0;
    saveCurrentGoalIndex();
    ROS_INFO("Pose goal index reset.");
  }

private:
  bool initialized_;
  move_base_msgs::MoveBaseGoal current_goal_;
  std::vector< std::vector<float> > goal_vec_;
  int current_goal_index_;

  // action server stuff
  bstars_navigation::GoToWaypointsFeedback feedback_;
  std::string action_name_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoints");
  WaypointsAction wp("waypoints");
  ros::spin();

  ros::shutdown();
  return 0;
  /*
  wp.init();

  bool wp_load_success = wp.loadWaypoints("waypoints.txt");
  bool wp_ind_load_success = wp.loadGoalIndex();

  if(wp_load_success && wp_ind_load_success)
  {
    wp.goToWaypoints();
  }
  */

  /* TEST CODE, THIS STUFF WORKS IF EVERYTHING ELSE BREAKS
  MoveBaseClient ac("move_base", true);
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 11.472;
  goal.target_pose.pose.position.y = 13.032;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  */
}
