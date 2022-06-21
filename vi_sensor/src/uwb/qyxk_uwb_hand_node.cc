/*************************************************************************
  > File Name: uwb_node.cpp
	> Author:Deakin
	> Mail: deakinhwang@163.com
  > Created Time:
 ************************************************************************/
#include <iostream>
#include <sstream>

#include <ros/ros.h> 
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include "../../common/utility-math/kalmanfilter.hpp"

static ros::Publisher uls_tag_position_pub_g;

static ros::Publisher uls_visualization_pub_g;
static ros::Publisher uwb_visualization_pub_g;


static kf::SingleValueLowPass kf_x, kf_y, kf_z;
static const double Q = 0.3;


// visualiz
void uwb_visulization_tracking( const ros::Publisher& pub, const geometry_msgs::Point p ){
    static visualization_msgs::Marker marker_points_g;
    //ROS_INFO("==visualization callback");
    marker_points_g.header.frame_id = "/map";
    marker_points_g.header.stamp = ros::Time::now();
    marker_points_g.type = visualization_msgs::Marker::POINTS;
    marker_points_g.action = visualization_msgs::Marker::ADD;
    marker_points_g.id = 1;

    marker_points_g.lifetime = ros::Duration();

    marker_points_g.scale.x = 0.02;
    marker_points_g.scale.y = 0.02;
    //points.scale.z = 0.02;
    marker_points_g.color.g = 1;
    marker_points_g.color.a = 1.0;

    marker_points_g.points.push_back( p );
    pub.publish( marker_points_g );
}

void uls_visulization_tracking( const ros::Publisher& pub, const geometry_msgs::Point p ){
    static visualization_msgs::Marker marker_points_g;
    //ROS_INFO("==visualization callback");
    marker_points_g.header.frame_id = "/map";
    marker_points_g.header.stamp = ros::Time::now();
    marker_points_g.type = visualization_msgs::Marker::POINTS;
    marker_points_g.action = visualization_msgs::Marker::ADD;
    marker_points_g.id = 2;

    marker_points_g.lifetime = ros::Duration();

    marker_points_g.scale.x = 0.02;
    marker_points_g.scale.y = 0.02;
    //points.scale.z = 0.02;
    marker_points_g.color.b = 0.7f;
    marker_points_g.color.a = 1.0;

    marker_points_g.points.push_back( p );
    pub.publish( marker_points_g );
}


void uwb_tag_position_callback( const geometry_msgs::Point::ConstPtr& msg ){
    geometry_msgs::Point point = *msg;
    ROS_INFO("tag position: %f %f %f", point.x, point.y, point.z );

    uwb_visulization_tracking( uwb_visualization_pub_g, point );

    point.x = kf_x.Update( point.x, Q, 10 * Q );
    point.y = kf_y.Update( point.y, Q, 10 * Q );
    //point.z = kf_z.SingleValueFilter( point.z, Q, 10 * Q );
    point.z = 0;
    uls_tag_position_pub_g.publish( point );
    uls_visulization_tracking( uls_visualization_pub_g, point );
    //pub.publish( point );
}


int main( int argc, char **argv ) {
    ros::init( argc, argv,"qyxk_uwb_hand");
    ROS_INFO("qyxk uwb hand start.");
    ros::NodeHandle nh;

    ros::Subscriber uwb_tag_position_sub;
    uwb_tag_position_sub = nh.subscribe("/qyxk_uwb/tag_position", 10, uwb_tag_position_callback );

    uls_tag_position_pub_g = nh.advertise<geometry_msgs::Point>("uls/tag_position", 10 );
    uwb_visualization_pub_g = nh.advertise<visualization_msgs::Marker>("uwb/tag_position_visual", 10 );
    uls_visualization_pub_g = nh.advertise<visualization_msgs::Marker>("uls/tag_position_visual", 10 );

    ros::spin();
    ROS_INFO("qyxk uwb hand terminal.");
    return 0;
}






