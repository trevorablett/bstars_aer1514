#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sstream>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

void make_marker(visualization_msgs::Marker &marker, int i){
       // Set our initial shape type to be a cube
       uint32_t shape = visualization_msgs::Marker::CUBE;
       std::string frame_text = "qr_location_";
       frame_text.append(patch::to_string(i));

       ROS_INFO_STREAM("frame: " << frame_text);

       marker.header.frame_id = frame_text;
       marker.header.stamp = ros::Time::now();

       marker.ns = "basic_shapes";
       marker.id = i;

       marker.type = shape;

       marker.action = visualization_msgs::Marker::ADD;
       marker.pose.position.x = 0;
       marker.pose.position.y = 0;
       marker.pose.position.z = 0;
       marker.pose.orientation.x = 0.0;
       marker.pose.orientation.y = 0.0;
       marker.pose.orientation.z = 0.0;
       marker.pose.orientation.w = 1.0;

       // Set the scale of the marker -- 1x1x1 here means 1m on a side
       marker.scale.x = 0.25;
       marker.scale.y = 0.25;
       marker.scale.z = 0.25;


       // Set the color -- be sure to set alpha to something non-zero!
       marker.color.r = 0.0f;
       marker.color.g = 1.0f;
       marker.color.b = 0.0f;
       marker.color.a = 1.0;

       marker.lifetime = ros::Duration();
     }

int main( int argc, char** argv )
{
   ros::init(argc, argv, "basic_shapes");
   ros::NodeHandle n;
   ros::Rate r(1);
   ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

   while (ros::ok())
   {
     visualization_msgs::MarkerArray ma;
     ma.markers.resize(10);

     int i;
     n.getParam("qr_count", i);
     make_marker(ma.markers[i], i);


     // Publish the marker
     while (marker_pub.getNumSubscribers() < 1)
     {
       if (!ros::ok())
       {
         return 0;
       }
       ROS_WARN_ONCE("Please create a subscriber to the markers");
       sleep(1);
     }

     marker_pub.publish(ma);

    r.sleep();
  }
}
