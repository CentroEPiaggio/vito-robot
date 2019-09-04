
#ifndef PF_UTILITIES_H
#define PF_UTILITIES_H

#include <kdl/kdl.hpp>
#include <Eigen/Core>
#include "geometries_utilities.h"



inline Eigen::Matrix<double, 6, 1> GetFIRAS(double min_distance, Eigen::Vector3d &distance_der_partial, double influence = 0.1, double gain = 1.0, int func_type = 1)
{

  Eigen::Matrix<double, 6, 1> Force = Eigen::Matrix<double, 6, 1>::Zero();
  double V = 0.0;
  switch (func_type)
  {
  case 1: // 
    V = gain * ( (1.0 / min_distance) -
                 (1.0 / influence) )  * (1.0 / (min_distance * min_distance));
    break;

  }

  Force(0) = V * distance_der_partial[0];
  Force(1) = V * distance_der_partial[1];
  Force(2) = V * distance_der_partial[2];
  Force(3) = 0;
  Force(4) = 0;
  Force(5) = 0;


  return Force;
}

inline Eigen::Matrix<double, 6, 1> getRepulsiveForceTable(KDL::Frame &T_in, double influence = 0.1, double gain = 1.0)
{
  Eigen::Matrix<double, 6, 1> force_local_object = Eigen::Matrix<double, 6, 1>::Zero();

  KDL::Frame T_table_world;

  T_table_world.p = T_in.p;
  T_table_world.p.data[2] = 0;

  KDL::Vector Table_position(0, 0, 0.0);

  double distance_local = std::abs( -Table_position.z() + T_in.p.z());

  Eigen::Vector3d distance_der_partial(0, 0, 1);

  if (distance_local <= influence )
  {
    force_local_object = GetFIRAS(distance_local, distance_der_partial, influence, gain);
  }

  Eigen::Matrix<double, 6, 1> force_local_link = Eigen::Matrix<double, 6, 1>::Zero();
  force_local_link = getAdjointT( T_in.Inverse() * T_table_world) * force_local_object;

  return force_local_link;
}


inline Eigen::Matrix<double, 6, 1> getRepulsiveForceObjects(KDL::Frame &T_in, double influence, KDL::Frame &Object_pos, double radius, double height, double gain = 1.0)
{
  Eigen::Matrix<double, 6, 1> ForceAndIndex;
  ForceAndIndex =  Eigen::Matrix<double, 6, 1>::Zero();

  KDL::Frame T_link_object = ( T_in.Inverse() * Object_pos ).Inverse();

  KDL::Frame T_CollisionPoint;

  getClosestPointstoCylinder( T_link_object, T_CollisionPoint, radius, height);

  T_CollisionPoint = Object_pos * T_CollisionPoint;


  double distance = (T_in.p - T_CollisionPoint.p).Norm();

  Eigen::Vector3d Nolmal_to_Cylinder;
  KDL::Vector Nolmal_to_Cylinder_kdl;
  Nolmal_to_Cylinder << T_CollisionPoint.M.UnitZ().data[0], T_CollisionPoint.M.UnitZ().data[1], T_CollisionPoint.M.UnitZ().data[2];
  Nolmal_to_Cylinder_kdl = T_CollisionPoint.M.UnitZ();
  
  // arrows_total.markers.push_back(Force2MarkerArrow(  Nolmal_to_Cylinder_kdl, T_CollisionPoint.p, id_global));

  LineCollisions::Point Point1(T_in.p.x(), T_in.p.y(), T_in.p.z());
  LineCollisions::Point Point2(T_CollisionPoint.p.x(), T_CollisionPoint.p.y(), T_CollisionPoint.p.z());
  LineCollisions::Line ClosestPoints(Point1, Point2);

  if ( distance <= influence)
  {
    // lines_total.markers.push_back(Line2markerLine(ClosestPoints, 1.0, 0.0, 0.0, id_global));
    ForceAndIndex = GetFIRAS(distance, Nolmal_to_Cylinder, influence, gain);
  }
  else
  {
    // lines_total.markers.push_back( Line2markerLine(ClosestPoints, 0.0, 1.0, 0.0, id_global) );
  }


  Eigen::Matrix<double, 6

  , 1> force_local_link = Eigen::Matrix<double, 6, 1>::Zero();
  force_local_link = getAdjointT( T_in.Inverse() * Object_pos) * ForceAndIndex;

  // tf::Transform CollisionTransform;
  // tf::transformKDLToTF( T_CollisionPoint, CollisionTransform);
  // tf_desired_hand_pose.sendTransform( tf::StampedTransform( CollisionTransform, ros::Time::now(), "world", "collision_point") );

  return force_local_link;

}

#endif