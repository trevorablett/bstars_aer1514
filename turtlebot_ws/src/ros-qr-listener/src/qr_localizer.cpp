#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>

typedef std::vector<double> Vectors2d;

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

class QrBroadcaster{

  public:
    int i;
    std::vector<std::string> store;
    std::map<char,Vectors2d> qr_locations;

    QrBroadcaster(){

      n.setParam("qr_count", 0);
      sub = n.subscribe("qrcode", 1,
      &QrBroadcaster::qrCallback, this);
      tf_listener = new tf2_ros::TransformListener(tfBuffer);
    }

  private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    tf2_ros::TransformListener* tf_listener;
    tf2_ros::Buffer tfBuffer;

    void qrCallback(const std_msgs::String::ConstPtr& msg){

        n.getParam("qr_count", i);
        bool far_enough = false;

        if(std::find(store.begin(), store.end(), msg->data.c_str()) != store.end()) {

          geometry_msgs::TransformStamped transform;

          for (int j = 0; j<= store.size()-1; j+=1){
            std::string frame_text = "qr_location_";
            frame_text.append(patch::to_string(j));

            try {
                transform = tfBuffer.lookupTransform("base_link", "odom", ros::Time(0));
            } catch (tf2::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }

            ROS_INFO_STREAM("distance: " << transform.transform.translation.x);

            if(false){
              far_enough = true;
              //qr_locations[frame_text]=
            }
          }

          if(far_enough){
            std::string frame_text = "qr_location_";
            frame_text.append(patch::to_string(i));

            ROS_INFO_STREAM("frame: " << frame_text);
            ROS_INFO("qr data received: [%s]", msg->data.c_str());

            store.push_back(msg->data.c_str());

            //store location here

            i+=1;
            n.setParam("qr_count", i);
          }

        } else {
          std::string frame_text = "qr_location_";
          frame_text.append(patch::to_string(i));

          ROS_INFO_STREAM("frame: " << frame_text);
          ROS_INFO("qr data received: [%s]", msg->data.c_str());

          store.push_back(msg->data.c_str());

          //store location here

          i+=1;
          n.setParam("qr_count", i);
        }

    }

    std::vector<std::string> get_stored_words(){
      return store;
    }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "qr_localizer");

  QrBroadcaster pb;

  ros::spin();

  return 0;
}
