
std_msgs/Header header

## contact point and wrench on each thimble
intrinsic_tactile_toolbox/TactileInfo[] thimbles

#phase space markers
phase_space/PhaseSpaceMarkerArray LEDs

#tranformations
geometry_msgs/TransformStamped  world_wrist
geometry_msgs/TransformStamped world_star
geometry_msgs/TransformStamped world_thimble

#point cloud of the scene
sensor_msgs/PointCloud2 scene



