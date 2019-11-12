//Importing libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <iostream>
#include <cmath>

#include <video_node/UVStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//Std namepsace to clean up code
using namespace std;
using namespace message_filters;

class BallLocationKeeper {
    private:
    bool ball_found;
    
    ros::NodeHandle nodeh;
    
    ros::Publisher ball_relative_coords_pub;
    ros::Publisher ball_odom_coords_pub;
    message_filters::Subscriber<video_node::UVStamped> uv_subscriber;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_subscriber;
    typedef sync_policies::ApproximateTime<video_node::UVStamped, sensor_msgs::PointCloud2> UV_PCL2_Policy;
    typedef Synchronizer<UV_PCL2_Policy> Sync;
    boost::shared_ptr<Sync> sync;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    geometry_msgs::PointStamped ball_odom_coords;

    public:
    BallLocationKeeper(ros::NodeHandle &nh);
    void find_world_point(const video_node::UVStampedConstPtr& uvs,
                          const sensor_msgs::PointCloud2ConstPtr& pcl2);
    void broadcast_ball_coords(const ros::TimerEvent& event);
};

BallLocationKeeper::BallLocationKeeper(ros::NodeHandle &nh)
    : tfListener(tfBuffer),
      uv_subscriber(nh, "/marco/ball_uv", 100),
      pcl_subscriber(nh, "/camera/depth/points", 1)
    {
    // Initialize node handle for the whole node/class
    nodeh = nh;

    ball_found = false;
    ball_odom_coords.header.frame_id = "odom";

    // Advertise that geopoint will be sent
    ball_relative_coords_pub = nh.advertise<geometry_msgs::PointStamped>("/geopoint", 100);
    ball_odom_coords_pub = nh.advertise<geometry_msgs::PointStamped>("/marco/ball_odom_pt", 100);

    // Define the approximate time sync policy so that we get nearly synched pcls and uvs
    sync.reset(new Sync(UV_PCL2_Policy(10), uv_subscriber, pcl_subscriber));      
    sync->registerCallback(boost::bind(&BallLocationKeeper::find_world_point, this, _1, _2));
    return;
}

void BallLocationKeeper::broadcast_ball_coords(const ros::TimerEvent& event) {

    if(ball_found == false) {
        return;
    }

    // First get tf for current base_footprint (i.e. local) frame ... wait up to 250ms for the next tf
    try {
        geometry_msgs::TransformStamped odom_to_basefp;
        geometry_msgs::PointStamped ball_basefp_coords;
        odom_to_basefp = tfBuffer.lookupTransform("base_footprint", "odom", ros::Time::now(), ros::Duration(0.25));
        tf2::doTransform(ball_odom_coords, ball_basefp_coords, odom_to_basefp);

        ball_basefp_coords.header.stamp = ros::Time::now();
        ball_basefp_coords.header.frame_id = "base_footprint";
        
        ball_relative_coords_pub.publish(ball_basefp_coords);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT transform odom to base_footprint: %s", ex.what());
    }

    // Second broadcast point for odom (i.e. world) frame
    return;
}

void BallLocationKeeper::find_world_point(const video_node::UVStampedConstPtr& uvs,
                                          const sensor_msgs::PointCloud2ConstPtr& pcl2) {
    ROS_DEBUG_STREAM("Found approximately synched messages. "
                         << " UVS came at time " << uvs->header.stamp
                         << " and "
                         << " PCL2 came at time " << pcl2->header.stamp);

    if(uvs->ball_visible == true) {
        ROS_DEBUG_STREAM("Ball is visible!");
        // Define a local point which can be used temporarily to gather point cloud
        geometry_msgs::PointStamped basefp_pt;

        // Define a TF which will be used to take the point to the world/odometry frame
        geometry_msgs::TransformStamped camerapcl_to_odom;
        int width = pcl2->width;
        int height = pcl2->height;

        // Convert from u (column / width), v (row/height) to position in array
        int point_idx = (uvs->pixel_row * pcl2->row_step) + (uvs->pixel_column * pcl2->point_step);

        // compute position in array where x,y,z data start
        int arrayPosX = point_idx + pcl2->fields[0].offset; // X has an offset of 0
        int arrayPosY = point_idx + pcl2->fields[1].offset; // Y has an offset of 4
        int arrayPosZ = point_idx + pcl2->fields[2].offset; // Z has an offset of 8
        basefp_pt.point.x = pcl2->data[arrayPosX];
        basefp_pt.point.y = pcl2->data[arrayPosY];
        basefp_pt.point.z = pcl2->data[arrayPosZ];

        camerapcl_to_odom = tfBuffer.lookupTransform("odom",
                                                  pcl2->header.frame_id,
                                                  pcl2->header.stamp,
                                                  ros::Duration(0.25));
        
        tf2::doTransform(basefp_pt, ball_odom_coords, camerapcl_to_odom);

        // Set this flag so that the local frame coords will send cyclically
        ball_found = true;

    } else {
        // Dont do anything with this point because there's no found ball
        ROS_DEBUG_STREAM("Ball isnt visible ...");
    }
    return;
}

/**
 * This main function instantiates the ROS node, and triggers the publishers and subscribers running..
 */
int main(int argc, char **argv)
{
    //Node intitialization  
    ros::init(argc, argv, "depthcloud");
    ros::NodeHandle n;
    BallLocationKeeper blk(n);

    ros::Timer timer = n.createTimer(ros::Duration(0.05), &BallLocationKeeper::broadcast_ball_coords, &blk);

    ros::spin();

  return 0;
}
