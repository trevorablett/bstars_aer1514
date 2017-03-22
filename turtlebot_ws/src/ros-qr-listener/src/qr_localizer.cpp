#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include <iostream>
#include <string>
#include <algorithm>

//TODO: Why do the qr frame spawn without parents if you don't launch the simulation first
// ... then qr scripts, will this happen on the real robot?
//TODO: Why does the TF Listener always give 0 as the transform

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

    QrBroadcaster(){
      n.setParam("qr_count", 0);

      sub = n.subscribe("qrcode", 1,
      &QrBroadcaster::qrCallback, this);
    }

  private:
    tf::TransformBroadcaster qr_broadcaster;
    tf::TransformListener listener;
    ros::NodeHandle n;
    ros::Subscriber sub;


    void qrCallback(const std_msgs::String::ConstPtr& msg){

        n.getParam("qr_count", i);
        bool far_enough = false;

        if(std::find(store.begin(), store.end(), msg->data.c_str()) != store.end()) {

          tf::StampedTransform transform;

          for (int j = 0; j<= store.size()-1; j+=1){
            std::string frame_text = "qr_location_";
            frame_text.append(patch::to_string(j));

            try {
                listener.lookupTransform("base_link", frame_text, ros::Time(0), transform);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }

            ROS_INFO_STREAM("distance: " << transform.getOrigin().x());

            if(false){
              far_enough = true;
            }
          }

          if(far_enough){
            std::string frame_text = "qr_location_";
            frame_text.append(patch::to_string(i));

            ROS_INFO_STREAM("frame: " << frame_text);
            ROS_INFO("qr data received: [%s]", msg->data.c_str());

            store.push_back(msg->data.c_str());

            qr_broadcaster.sendTransform(tf::StampedTransform(
              tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(0.0, 0.0, 0.0)),
              ros::Time::now(),"base_link", frame_text));

            i+=1;
            n.setParam("qr_count", i);
          }

        } else {
          std::string frame_text = "qr_location_";
          frame_text.append(patch::to_string(i));

          ROS_INFO_STREAM("frame: " << frame_text);
          ROS_INFO("qr data received: [%s]", msg->data.c_str());

          store.push_back(msg->data.c_str());

          qr_broadcaster.sendTransform(tf::StampedTransform(
            tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(0.0, 0.0, 0.0)),
            ros::Time::now(), "base_link", frame_text));

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
