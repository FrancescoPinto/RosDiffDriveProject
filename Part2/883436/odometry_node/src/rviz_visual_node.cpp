
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_broadcaster.h>

#include <sstream>

uint32_t shape = visualization_msgs::Marker::SPHERE;

class rviz_visual
{
    public:
    rviz_visual(){
    sub = n.subscribe("/robot_markerset/pose", 1000,  &rviz_visual::callback,this);
    marker_pub = n.advertise<visualization_msgs::Marker>("/rviz_visualize/marker", 1000);

    }

    void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map"; //todo, cambiarlo ...
        marker.header.stamp = msg->header.stamp;

        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = msg->pose.position.x;
        marker.pose.position.y = msg->pose.position.y;
        marker.pose.position.z = msg->pose.position.z;

        marker.pose.orientation.x = msg->pose.orientation.x;
        marker.pose.orientation.y =msg->pose.orientation.y;
        marker.pose.orientation.z =msg->pose.orientation.z;
        marker.pose.orientation.w =msg->pose.orientation.w;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker_pub.publish(marker);

        tf::Transform map_markerset;
        map_markerset.setOrigin(tf::Vector3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z));
        tf::Quaternion q1(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
        map_markerset.setRotation(q1);

        transform_broadcaster.sendTransform (tf::StampedTransform(map_markerset, ros::Time::now(), "map", "robot_markerset"));
        
    }

    private:
    ros::Subscriber sub;
    ros::Publisher marker_pub;
    ros::NodeHandle n;
    tf::TransformBroadcaster transform_broadcaster;

};



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "marker_node");
  rviz_visual visualizer; 
  ros::MultiThreadedSpinner spinner(2); //use 2 threads
  spinner.spin();

  return 0;
}