#geotiff保存地图
if (map_service_client_.call(srv_map))
 // when we use 'rostopic pub syscommand std_msgs/String "savegeotiff" .. the codes after here will run'


map_service_client_ = n_.serviceClient<nav_msgs::GetMap>("map");  
##一个服务的客户端 通过call("map") 向server发送命令请求得到地图的命令


path_service_client_ = n_.serviceClient<hector_nav_msgs::GetRobotTrajectory>("trajectory");
##一个call “trajectory”话题的客户端 这个客户端的service在hector_trajectory_server
