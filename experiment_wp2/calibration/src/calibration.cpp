#include <ros/ros.h>

//#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Eigen>
#include <fstream>
using namespace Eigen;


class calibration
{
  private:
   
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;
   
    std::string parent_frame_name_;
    std::string child_frame_name_;
   
    std::string phase_frame_name_;

    std::string camera_frame_name_;
    std::string type_frame_name_;
    std::string tf_launch_file_dir;
    std::string calibration_type_;

    //tf transform listener
    tf::TransformListener tf_listener_;

    //Maual pose estimation
    Matrix4f matrix_manual_pose_;

    //Matrix of the calculated transform from parent_frame_name to child_frame_name
    Matrix4f matrix_calculated_transform;


  public:
    
    void calculateTranformation();
    void writeLaunchTranformation(const std::string nomefile, Eigen::Quaternionf mat_rot,Eigen::Vector3f trasl );

    Matrix4f calculateMatrix(const tf::StampedTransform& );

    
    calibration(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
      priv_nh_.param<std::string>("tf_launch_file_dir", tf_launch_file_dir, "");
     
      priv_nh_.param<std::string>("child_frame_name_", child_frame_name_, "");
      priv_nh_.param<std::string>("parent_frame_name_", parent_frame_name_, "");
     
      priv_nh_.param<std::string>("phase_frame_name_",  phase_frame_name_, "");
 
      //to be used for object calibration
      priv_nh_.param<std::string>("camera_frame_name_",  camera_frame_name_, "");

      //to be used for hand calibration
      priv_nh_.param<std::string>("type_frame_name_",  type_frame_name_, "");
      priv_nh_.param<std::string>("calibration_type_", calibration_type_, "");

      std::string txyz_string, quaternionwxyz_string;
      priv_nh_.param<std::string>("manualTxyz", txyz_string, "");
      priv_nh_.param<std::string>("manualQwxyz", quaternionwxyz_string, "");

 
      std::vector<float> elems;
      std::stringstream ss(txyz_string);
      if (txyz_string!="")
      {
        std::string item;
        while (std::getline(ss, item, ',')) 
          elems.push_back(std::atof(item.c_str()));
        tf::Vector3 manualTxyz(elems[0],elems[1],elems[2]);
       
        elems.clear();
        std::stringstream ss1(quaternionwxyz_string);
        while (std::getline(ss1, item, ',')) {
          elems.push_back(std::atof(item.c_str()));
        }

        tf::Quaternion manualQwxyz(elems[1],elems[2],elems[3],elems[0]);
     
        tf::StampedTransform manual_tf;
        manual_tf.setOrigin(manualTxyz);
        manual_tf.setRotation(manualQwxyz);
        matrix_manual_pose_=calculateMatrix(manual_tf);
      }
    }

    //! Empty stub
    ~calibration() {}

};

/*
Matrix4f calculate_object_pose()
{

 string filename = "/home/kuko/Code/poseEstimation/parametersFiles/config.txt";    
 ParametersPoseEstimation params(filename);
 
 string recognizedObjects_dir = "/home/kuko/Code/poseEstimation/data/recognizedObjects"; 
 I_SegmentedObjects objects(recognizedObjects_dir);
 pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points = params.kinectGrabFrame();

 params.recognizePose(objects,xyz_points);   
 // definitions::Object detected_objects;
  boost::shared_ptr<vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms = objects.getTransforms ();
  Eigen::Matrix4f object_pose = transforms->at(0);
  return object_pose;
}*/

//Write the launch file for executing the node of the static transformation bewteen parent_frame_name_ to child_frame_name_
void calibration::writeLaunchTranformation(const std::string nomefile, Eigen::Quaternionf mat_rot,Eigen::Vector3f trasl )
{
    std::string file_str=tf_launch_file_dir+"/"+nomefile+".launch";
    std::ofstream f_out(file_str.c_str());
    if (!f_out)
      std::cout<<"Problems opening file"<<std::endl;
    f_out<<"<launch>\n<node pkg=\"tf\" type=\"static_transform_publisher\" name=";
    f_out<<"\""<<nomefile<<"\" args=\""<<trasl(0)<<' '<<trasl(1)<<' '<<trasl(2)<<' '<<mat_rot.x() <<' '<<mat_rot.y()<<' '<<mat_rot.z()<<' '<<mat_rot.w();
    f_out<<' '<< parent_frame_name_<<' '<<child_frame_name_<<" 100\" />\n </launch>";

}


//calculate the 4x4 matrix from a stamped transform
Matrix4f calibration::calculateMatrix(const tf::StampedTransform& listened_tf)
{   tf::Matrix3x3 rotation= listened_tf.getBasis();
    tf::Vector3 trasl=listened_tf.getOrigin();

    Matrix4f matrix_transf;
    for (int i=0;i<3;i++)
    {  for(int j=0;j<3;j++)
          matrix_transf(i,j)=rotation[i][j];
          matrix_transf(i,3)=trasl[i];
    }   
    matrix_transf(3,0)=matrix_transf(3,1)=matrix_transf(3,2)=0;
    matrix_transf(3,3)=1;
  

    return matrix_transf;
}


void calibration::calculateTranformation()
{
  tf::StampedTransform tf_world_type;
  std::string nomefile;
   
  try{  tf_listener_.waitForTransform(phase_frame_name_, type_frame_name_, ros::Time(0), ros::Duration(10.0) );
        tf_listener_.lookupTransform(phase_frame_name_, type_frame_name_, ros::Time(0),tf_world_type);
  }
  catch (tf::TransformException ex){ ROS_ERROR("%s",ex.what()); }

  Matrix4f matrix_world_type=calculateMatrix(tf_world_type);
 
  if (!calibration_type_.compare("camera"))
  {   nomefile="tf_camera_world";
      matrix_calculated_transform=matrix_manual_pose_* matrix_world_type.inverse();
  }
 
   
  if (!calibration_type_.compare("object"))
  {   tf::StampedTransform tf_world_camera;
      nomefile="tf_star_object";
      Matrix4f matrix_world_camera;
      try{  tf_listener_.waitForTransform(phase_frame_name_, camera_frame_name_, ros::Time(0), ros::Duration(10.0) );
            tf_listener_.lookupTransform(phase_frame_name_, camera_frame_name_, ros::Time(0),tf_world_camera);
      }
      catch (tf::TransformException ex){ ROS_ERROR("%s",ex.what()); }
      matrix_world_camera=calculateMatrix(tf_world_camera);
      matrix_calculated_transform=matrix_world_type.inverse() * matrix_world_camera * matrix_manual_pose_;

   }

   if (!calibration_type_.compare("hand"))
  {   tf::StampedTransform tf_world_wrist;
      nomefile="tf_wrist_hand";

      std::cout<<"dddddd"<<std::endl;
      try{  tf_listener_.waitForTransform(phase_frame_name_, parent_frame_name_, ros::Time(0), ros::Duration(10.0) );
            tf_listener_.lookupTransform(phase_frame_name_, parent_frame_name_, ros::Time(0),tf_world_wrist);
      }
      catch (tf::TransformException ex){ ROS_ERROR("%s",ex.what()); }   
    
      Matrix4f matrix_world_wrist=calculateMatrix(tf_world_wrist);

      matrix_calculated_transform = matrix_world_wrist.inverse()*matrix_world_type;

  }

  
  Eigen::Quaternionf mat_rot(matrix_calculated_transform.block<3,3>(0,0)); 
  Eigen::Vector3f trasl=matrix_calculated_transform.block<3,1>(0,3).transpose();
  writeLaunchTranformation(nomefile, mat_rot,trasl);
  std::cout << "Transformation from " << parent_frame_name_<<" to "<<child_frame_name_ <<"  "<<trasl(0)<<' '<<trasl(1)<<' '<<trasl(2)<<' '<<mat_rot.x() <<' '<<mat_rot.y()<<' '<<mat_rot.z()<<' '<<mat_rot.w()<< std::endl;


}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "calibration_node");
  ros::NodeHandle nh;

  calibration node(nh);
  node.calculateTranformation();
 
  //ros::spin();  
  
  return 0;
}