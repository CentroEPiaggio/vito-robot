
#ifndef ROS_DESPERATE_HOUSEWIFE_UTILITIES_H
#define ROS_DESPERATE_HOUSEWIFE_UTILITIES_H

// #include <ros/node_handle.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <kdl/kdl.hpp>
#include <Eigen/Core>
#include "distance_between_lines.h"



visualization_msgs::Marker inline Line2markerLine(LineCollisions::Line Line_local, float r =1.0, float g = 0.0, float b = 0.0, int id  = 0, std::string frame_name = std::string("world"),   int type = visualization_msgs::Marker::LINE_LIST)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = frame_name;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.type = type;
    line_list.id = id;
    line_list.scale.x = 0.01;
    line_list.color.r = r;
    line_list.color.g = g;
    line_list.color.b = b;
    line_list.color.a = 1.0;


    geometry_msgs::Point p1;
    p1.x = Line_local.P1[0];
    p1.y = Line_local.P1[1];
    p1.z = Line_local.P1[2];

    geometry_msgs::Point p2;
    p2.x = Line_local.P2[0];
    p2.y = Line_local.P2[1];
    p2.z = Line_local.P2[2];
    line_list.lifetime = ros::Duration(1);

    line_list.points.push_back(p1);
    line_list.points.push_back(p2);
    return line_list;
}

Eigen::Quaterniond inline RotationMarker(KDL::Vector &ris_Force, KDL::Vector &point)
{

    Eigen::Vector3d  x(1, 0, 0);
    Eigen::Vector3d Force_eigen(ris_Force.x(), ris_Force.y(), ris_Force.z());
    double angle = std::acos( (x.dot(Force_eigen)) / (x.norm() * Force_eigen.norm()));
    Eigen::Matrix3d transformation_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d axis(0, 0, 0);
    axis = (x.cross(Force_eigen)) / (x.cross(Force_eigen)).norm();
    transformation_ = Eigen::AngleAxisd(angle, axis);

    Eigen::Quaterniond quat_eigen_hand(transformation_);

    return quat_eigen_hand.normalized();
}


visualization_msgs::Marker inline Force2MarkerArrow( KDL::Vector force, KDL::Vector position, int id = 0, std::string frame_name = std::string("world") )
{
    int32_t shape = visualization_msgs::Marker::ARROW;
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = frame_name;
    marker.header.stamp = ros::Time::now();
    // marker.ns = "basic_shapes";
    marker.id = id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();

    Eigen::Quaterniond quat;
    quat =  RotationMarker(force, position);

    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();

    // marker.scale.x = 1.0 * force;
    marker.scale.x = 0.10;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1;
    marker.lifetime = ros::Duration(1);

    return marker;
}


Eigen::MatrixXd inline getGainMatrix(std::string parameter, ros::NodeHandle n, int dimension)
{
    XmlRpc::XmlRpcValue my_list;
    n.getParam(parameter.c_str(), my_list);
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    Eigen::MatrixXd K(dimension, dimension);
    K = Eigen::MatrixXd::Zero(dimension, dimension);
    for (int i = 0; i < std::max(my_list.size(), dimension); ++i)
    {
        K(i, i) = static_cast<double>(my_list[i]);
    }
    return K;
}


Eigen::Matrix<double, 4, 4>  inline FromKdlToEigen(KDL::Frame &T_v_o)
{
    Eigen::Matrix<double, 4, 4>  Tvo_eigen;
    Tvo_eigen.row(0) << T_v_o.M.data[0], T_v_o.M.data[1], T_v_o.M.data[2], T_v_o.p.x();
    Tvo_eigen.row(1) << T_v_o.M.data[3], T_v_o.M.data[4], T_v_o.M.data[5], T_v_o.p.y();
    Tvo_eigen.row(2) << T_v_o.M.data[6], T_v_o.M.data[7], T_v_o.M.data[8], T_v_o.p.z();
    Tvo_eigen.row(3) << 0, 0, 0, 1;
    return Tvo_eigen;
}



#endif
