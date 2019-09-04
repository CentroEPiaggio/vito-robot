
#ifndef GEOMETRIES_UTILITIES_H
#define GEOMETRIES_UTILITIES_H

#include <kdl/kdl.hpp>
#include <Eigen/Core>
#include "distance_between_lines.h"

void inline getClosestPointstoCylinder( KDL::Frame T_link_object, KDL::Frame &T_CollisionPoint, double radius, double height)
{
    LineCollisions LineCollisionsLocal;

    LineCollisions::Point Point1(T_link_object.p.x(), T_link_object.p.y(), T_link_object.p.z());
    LineCollisions::Point Point2(T_link_object.p.x() + .001, T_link_object.p.y() + .001, T_link_object.p.z() + .001);

    LineCollisions::Point Point3(0.0, 0.0, height / 2.0);
    LineCollisions::Point Point4(0.0, 0.0, -height / 2.0);

    LineCollisions::Line Line1(Point1, Point2);
    LineCollisions::Line Line2(Point3, Point4);
    LineCollisions::Line ClosestPoints;

    ClosestPoints = LineCollisionsLocal.getClosestPoints(Line1, Line2);

    KDL::Vector V1(ClosestPoints.P1[0], ClosestPoints.P1[1], ClosestPoints.P1[2]);
    KDL::Vector V2(ClosestPoints.P2[0], ClosestPoints.P2[1], ClosestPoints.P2[2]);
    KDL::Vector V3(V1 - V2);
    // V3  = (V1 - V2);
    KDL::Frame pos_final;
    KDL::Vector collision_point_on_line(ClosestPoints.P2[0], ClosestPoints.P2[1], ClosestPoints.P2[2] );
    if ( (ClosestPoints.P2 == Point4) || (ClosestPoints.P2 == Point3) )
    {   // The point is on one of the planar face

        KDL::Vector V4( V3.x(), V3.y(), 0.0  );
        double pos_on_plane = V4.Norm();
        if (pos_on_plane >= radius )
        {   // The point is on the border
            KDL::Vector Uz(V4 / V4.Norm() );
            KDL::Vector Ux(0.0, 0.0, 1.0);
            pos_final = KDL::Frame( KDL::Rotation(Ux, Uz * Ux, Uz), collision_point_on_line) *
                        KDL::Frame( KDL::Rotation::RotX(0.0), KDL::Vector( 0.0 , 0.0, radius) );
            KDL::Vector V5;
            V5 = (T_link_object.Inverse() * pos_final).Inverse().p;
            V5 = V5 /  V5.Norm();
            double angle = std::atan2(V5.x(), V5.z());
            pos_final.M =  pos_final.M * KDL::Rotation::RotY( angle );
        }
        else
        {   //The cylinder is on the one of the planar faces
            KDL::Vector Ux;
            KDL::Vector Uz;
            Ux = ( V4 / V4.Norm() );
            Uz = KDL::Vector(0.0, 0.0, 1.0);
            if ( ClosestPoints.P2 == Point4 )
            {
                Uz = KDL::Vector(0.0, 0.0, -1.0);
            }
            pos_final = KDL::Frame( KDL::Rotation(Ux, Uz * Ux, Uz), collision_point_on_line) *
                        KDL::Frame( KDL::Rotation::RotX(0.0), KDL::Vector(pos_on_plane, 0.0, 0.0) );

        }
    }
    else
    {   // The point is on the cylindrical face
        KDL::Vector Uz(V3 / V3.Norm() );
        KDL::Vector Ux(0.0, 0.0, 1.0);
        pos_final = KDL::Frame( KDL::Rotation(Ux, Uz * Ux, Uz), collision_point_on_line) *
                    KDL::Frame( KDL::Rotation::RotX(0.0), KDL::Vector(0.0, 0.0, radius) );
    }

    T_CollisionPoint.M = pos_final.M;
    T_CollisionPoint.p = pos_final.p;

}

/*
Eigen::Vector3d GetPartialDerivate(KDL::Frame &T_v_o, KDL::Vector &Point_v, double radius, double height)
{
  Eigen::Matrix<double, 4, 4>  Tvo_eigen;
  Tvo_eigen = FromKdlToEigen(T_v_o);
  Eigen::Vector4d Point_v_eigen(Point_v.x(), Point_v.y(), Point_v.z(), 1);

  Eigen::Vector4d Point_o;
  Point_o = Tvo_eigen.inverse() * Point_v_eigen;
  double n = 2; // n as in the paper should be in 4 but considering the shortest distance to obstacles. Here this is not being considered :( TODO

  Eigen::Vector4d distance_der_partial(0, 0, 0, 0);
// distance_der_partial = x^2/radius + y^2 / radius + 2*(z^2n) /l
  distance_der_partial[0] = (Point_o[0] * 2) / radius ;
  distance_der_partial[1] = (Point_o[1] * 2) / radius ;
  distance_der_partial[2] = (std::pow((Point_o[2] * 2 / height), (2 * n - 1)) * (2 * n) ); //n=4
//n=1
// distance_der_partial[2] = (Point_o[2]*4) / height ;
  distance_der_partial[3] = 0;

  Eigen::Vector3d Der_v;
  Eigen::Vector4d partial_temp;
  partial_temp = Tvo_eigen * distance_der_partial;
  partial_temp = partial_temp.normalized();
  Der_v[0] = partial_temp[0];
  Der_v[1] = partial_temp[1];
  Der_v[2] = partial_temp[2];

  return Der_v;
}*/


#endif