#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

typedef std::vector<float> Vectors2d;

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

class QrLocalizer{

  public:
    int i;
    float threshold;
    std::vector<std::string> store;
    std::map<std::string, Vectors2d> qr_locations;

    QrLocalizer(){

      n.setParam("qr_count", 0);
      n.setParam("distance_threshold", 2.0);
      n.setParam("stored_keywords", store);
      sub = n.subscribe("/web_cam_qr/qrcode", 1,
      &QrLocalizer::qrCallback, this);
      tf_listener = new tf2_ros::TransformListener(tfBuffer);
    }

  private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    tf2_ros::TransformListener* tf_listener;
    tf2_ros::Buffer tfBuffer;
    geometry_msgs::TransformStamped robot_location;


    void qrCallback(const std_msgs::String::ConstPtr& msg){
        n.getParam("qr_count", i);
        n.getParam("distance_threshold", threshold);

        if(std::find(store.begin(), store.end(), msg->data.c_str()) != store.end()) {

          float distance;
          bool far_enough = true;
          Vectors2d v;
          std::vector<std::string>::iterator iter = store.begin();

          while((iter = std::find(iter, store.end(), msg->data.c_str())) != store.end()) {
            std::vector<std::string>::iterator it = std::find(store.begin(), store.end(), msg->data.c_str());
            ptrdiff_t index = find(iter, store.end(), msg->data.c_str()) - store.begin();

            std::string frame_text = "qr_location_";
            frame_text.append(patch::to_string(index));

            try {
                robot_location = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
            } catch (tf2::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }

            distance = sqrt(pow((robot_location.transform.translation.x - qr_locations[frame_text][0]), 2) +
            pow((robot_location.transform.translation.y - qr_locations[frame_text][1]), 2));

            if(distance<threshold){
              far_enough = false;
              ROS_INFO_STREAM("Repeated word [" << msg->data.c_str()
              << "] ignored since distance " << distance << " to previous frame " << frame_text << " is too small");
            }

            iter++;
          }

          if(far_enough){
            std::string frame_text = "qr_location_";
            frame_text.append(patch::to_string(i));

            ROS_INFO("qr data stored: [%s]", msg->data.c_str());

            store.push_back(msg->data.c_str());

            float myfloats[] = {robot_location.transform.translation.x, robot_location.transform.translation.y};
            v.assign (myfloats, myfloats+2);
            qr_locations[frame_text] = v;
            ROS_INFO_STREAM("New (repeat word) frame " << frame_text << " x coordinate: " << qr_locations[frame_text][0]);
            ROS_INFO_STREAM("New (repeat word) frame " << frame_text << " y coordinate: " << qr_locations[frame_text][1]);

            i+=1;
            n.setParam("qr_count", i);
            n.setParam("stored_keywords", store);
          }

        }
        else {
          std::string frame_text = "qr_location_";
          frame_text.append(patch::to_string(i));
          Vectors2d v;

          ROS_INFO("qr data stored: [%s]", msg->data.c_str());

          try {
              robot_location = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
          } catch (tf2::TransformException ex) {
              ROS_ERROR("%s",ex.what());
          }

          store.push_back(msg->data.c_str());

          float myfloats[] = {robot_location.transform.translation.x, robot_location.transform.translation.y};
          v.assign (myfloats, myfloats+2);
          qr_locations[frame_text] = v;
          ROS_INFO_STREAM("New frame " << frame_text << " x coordinate: " << qr_locations[frame_text][0]);
          ROS_INFO_STREAM("New frame " << frame_text << " y coordinate: " << qr_locations[frame_text][1]);

          i+=1;
          n.setParam("qr_count", i);
          n.setParam("stored_keywords", store);
        }

    }

    std::vector<std::string> get_stored_words(){
      return store;
    }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "qr_localizer");

  QrLocalizer pb;

  ros::spin();

  return 0;
}
