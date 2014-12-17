#include <ros/ros.h>
#include <ros/serialization.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "phase_space/PhaseSpaceMarkerArray.h"


#include "collect/AllData.h"


namespace collect {

class dataPublisher
{
  private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;
      
     //! Subscriber for markers readings
    typedef phase_space::PhaseSpaceMarkerArray::ConstPtr MarkerPtr;
    typedef geometry_msgs::TransformStamped::ConstPtr transPtr;
    typedef message_filters::sync_policies::ApproximateTime<phase_space::PhaseSpaceMarkerArray,geometry_msgs::TransformStamped,geometry_msgs::TransformStamped> MySyncPolicy;
    
    message_filters::Subscriber<phase_space::PhaseSpaceMarkerArray> sub_markers_readings;
    message_filters::Subscriber<geometry_msgs::TransformStamped> tf_readings_wrist;
    message_filters::Subscriber<geometry_msgs::TransformStamped> tf_readings_star;
      
    message_filters::Synchronizer< MySyncPolicy > sync;

    //! Publisher for the data 
    ros::Publisher data_info;

    // There always should be a listener and a broadcaster!
    //! A tf transform listener
    tf::TransformListener tf_listener_;

    //! A tf transform broadcaster
    tf::TransformBroadcaster tf_broadcaster_;

  public:
   
  void callback(const MarkerPtr& , const transPtr&, const transPtr&);
  dataPublisher(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"),sub_markers_readings(nh,nh.resolveName("/phase_space_markers"),1),tf_readings_wrist(nh,"/phase_space_world_to_/bracelet",1),tf_readings_star(nh,"/phase_space_world_to_/star",1),sync(MySyncPolicy(100),sub_markers_readings,tf_readings_wrist,tf_readings_star)
  { 
      sync.registerCallback( boost::bind( &dataPublisher::callback, this, _1, _2 ,_3) );
      data_info = nh_.advertise<AllData>(nh_.resolveName("/data_info"), 10);
  }

    


    //! Empty stub
    ~dataPublisher() {}

};

//Publishe for the markers, the point cloud , the tranformation published on /tf
void dataPublisher::callback(const MarkerPtr& markers, const transPtr& t_world_wrist, const transPtr& t_world_star )
{
  // read from the topics
  AllData message;
 
//  std::cout<<"Message arrived"<<std::endl;

  message.header = markers->header;
  
  message.LEDs.markers.resize((*markers).markers.size());
  message.LEDs.markers=(*markers).markers;
  
  message.world_wrist=*t_world_wrist;
  message.world_star=*t_world_wrist;
 
  data_info.publish(message);
}


} // namespace collect

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "dataPublisher_node");
  ros::NodeHandle nh;

  collect::dataPublisher node(nh);

  ros::spin();  
  
  return 0;
}