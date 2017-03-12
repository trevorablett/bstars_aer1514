#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <ros/package.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <costmap_2d/costmap_2d.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class WaypointsNode
{
public:
  MoveBaseClient *ac;
  ros::NodeHandle n_;

  WaypointsNode()
  {
    ac = new MoveBaseClient("move_base", true);
    initialized_ = false;
    current_goal_index_ = 1; //TODO: THIS SHOULD BE LOADED (TO "CONTINUE" FROM STOPPED POSITION), NOT SET AS 0
  }

  ~WaypointsNode()
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
  int loadWaypoints()
  {
    std::string line;
    std::string path = ros::package::getPath("bstars_navigation");
    path = path + "/param/waypoints.txt";
    std::ifstream wp_file(path.c_str());
    if(wp_file.is_open())
    {
      while(std::getline(wp_file, line))
      {
        std::cout << line << "\n";
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
    while(current_goal_index_ < goal_vec_.size())
    {
      move_base_msgs::MoveBaseGoal goal = vecToMoveBaseGoal(goal_vec_[current_goal_index_]);
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      ROS_INFO("Sending x, y, yaw goal of %f, %f, %f",
               goal.target_pose.pose.position.x,
               goal.target_pose.pose.position.y,
               goal_vec_[current_goal_index_][2]);
      ac->sendGoal(goal);
      ROS_INFO("Goal sucessfully sent"); //without this message, it doesn't always run

      ac->waitForResult();
      if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("woop woop");
      else
        ROS_INFO("aw raspberries");

      current_goal_index_++;
    }
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

  // append a visited waypoint to a "visited waypoints" file
  // store the index of the last visited waypoint in another file
  int storeVisitedWaypoint()
  {

  }

  // reset the stored waypoints files
  int resetStoredWaypoints()
  {

  }

private:
  bool initialized_;
  move_base_msgs::MoveBaseGoal current_goal_;
  std::vector< std::vector<float> > goal_vec_;
  int current_goal_index_;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "waypoints_node");
  WaypointsNode wp;
  wp.init();

  bool load_success = wp.loadWaypoints();

  if(load_success)
  {
    wp.goToWaypoints();
  }

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

  return 0;
}
