# costmap_2d/costmap_2d_ros.h


在ROS的导航中，costmap_2d这个包主要负责根据传感器的信息建立和更新二维或三维的地图。ROS的地图（costmap）采用网格（grid）的形式，每个网格的值从0~255，分为三种状态：占用（有障碍物）、无用（空闲的）、未知。具体状态和值的对应关系如下


上图共分为五个部分：（下面的红色框图是机器人的轮廓，旁边的黑框是上图的映射位置）
      （1） Lethal（致命的）:机器人的中心与该网格的中心重合，此时机器人必然与障碍物冲突。
      （2） Inscribed（内切）：网格的外切圆与机器人的轮廓内切，此时机器人也必然与障碍物冲突。
      （3） Possibly circumscribed（外切）：网格的外切圆与机器人的轮廓外切，此时机器人相当于靠在障碍物附近，所以不一定冲突。
      （4） Freespace（自由空间）：没有障碍物的空间。
      （5） Unknown（未知）：未知的空间。




http://docs.ros.org/jade/api/costmap_2d/html/costmap__2d__ros_8cpp_source.html#l00062

00062 Costmap2DROS::Costmap2DROS(std::string name, tf::TransformListener& tf) :
00063     layered_costmap_(NULL), name_(name), tf_(tf), stop_updates_(false), initialized_(true), stopped_(false),
00064     robot_stopped_(false), map_update_thread_(NULL), last_publish_(0),
00065     plugin_loader_("costmap_2d", "costmap_2d::Layer"), publisher_(NULL)
00066 {
00067   ros::NodeHandle private_nh("~/" + name);
00068   ros::NodeHandle g_nh;
00069 
00070   // get our tf prefix
00071   ros::NodeHandle prefix_nh;
00072   std::string tf_prefix = tf::getPrefixParam(prefix_nh);
00073 
00074   // get two frames
00075   private_nh.param("global_frame", global_frame_, std::string("/map"));
00076   private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
00077 
00078   // make sure that we set the frames appropriately based on the tf_prefix
00079   global_frame_ = tf::resolve(tf_prefix, global_frame_);
00080   robot_base_frame_ = tf::resolve(tf_prefix, robot_base_frame_);
00081 
00082   ros::Time last_error = ros::Time::now();
00083   std::string tf_error;
00084   // we need to make sure that the transform between the robot base frame and the global frame is available
00085   while (ros::ok()
00086       && !tf_.waitForTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1), ros::Duration(0.01),
00087                                &tf_error))
00088   {
00089     ros::spinOnce();
00090     if (last_error + ros::Duration(5.0) < ros::Time::now())
00091     {
00092       ROS_WARN("Timed out waiting for transform from %s to %s to become available before running costmap, tf error: %s",
00093                robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
00094       last_error = ros::Time::now();
00095     }
00096     // The error string will accumulate and errors will typically be the same, so the last
00097     // will do for the warning above. Reset the string here to avoid accumulation.
00098     tf_error.clear();
00099   }
00100 
00101   // check if we want a rolling window version of the costmap
00102   bool rolling_window, track_unknown_space, always_send_full_costmap;
00103   private_nh.param("rolling_window", rolling_window, false);
00104   private_nh.param("track_unknown_space", track_unknown_space, false);
00105   private_nh.param("always_send_full_costmap", always_send_full_costmap, false);
00106 
00107   layered_costmap_ = new LayeredCostmap(global_frame_, rolling_window, track_unknown_space);
00108 
00109   if (!private_nh.hasParam("plugins"))
00110   {
00111     resetOldParameters(private_nh);
00112   }
00113 
00114   if (private_nh.hasParam("plugins"))
00115   {
00116     XmlRpc::XmlRpcValue my_list;
00117     private_nh.getParam("plugins", my_list);
00118     for (int32_t i = 0; i < my_list.size(); ++i)
00119     {
00120       std::string pname = static_cast<std::string>(my_list[i]["name"]);
00121       std::string type = static_cast<std::string>(my_list[i]["type"]);
00122       ROS_INFO("Using plugin \"%s\"", pname.c_str());
00123 
00124       boost::shared_ptr<Layer> plugin = plugin_loader_.createInstance(type);
00125       layered_costmap_->addPlugin(plugin);
00126       plugin->initialize(layered_costmap_, name + "/" + pname, &tf_);
00127     }
00128   }
00129 
00130   // subscribe to the footprint topic
00131   std::string topic_param, topic;
00132   if (!private_nh.searchParam("footprint_topic", topic_param))
00133   {
00134     topic_param = "footprint_topic";
00135   }
00136 
00137   private_nh.param(topic_param, topic, std::string("footprint"));
00138   footprint_sub_ = private_nh.subscribe(topic, 1, &Costmap2DROS::setUnpaddedRobotFootprintPolygon, this);
00139 
00140   if (!private_nh.searchParam("published_footprint_topic", topic_param))
00141   {
00142     topic_param = "published_footprint";
00143   }
00144 
00145   private_nh.param(topic_param, topic, std::string("oriented_footprint"));
00146   footprint_pub_ = private_nh.advertise<geometry_msgs::PolygonStamped>("footprint", 1);
00147 
00148   setUnpaddedRobotFootprint(makeFootprintFromParams(private_nh));
00149 
00150   publisher_ = new Costmap2DPublisher(&private_nh, layered_costmap_->getCostmap(), global_frame_, "costmap",
00151                                       always_send_full_costmap);
00152 
00153   // create a thread to handle updating the map
00154   stop_updates_ = false;
00155   initialized_ = true;
00156   stopped_ = false;
00157 
00158   // Create a timer to check if the robot is moving
00159   robot_stopped_ = false;
00160   double pose_update_frequency;
00161   private_nh.param("pose_update_frequency", pose_update_frequency, 10.0);
00162   timer_ = private_nh.createTimer(ros::Duration(1 / pose_update_frequency),
00163                                   &Costmap2DROS::movementCB, this);
00164 
00165   dsrv_ = new dynamic_reconfigure::Server<Costmap2DConfig>(ros::NodeHandle("~/" + name));
00166   dynamic_reconfigure::Server<Costmap2DConfig>::CallbackType cb = boost::bind(&Costmap2DROS::reconfigureCB, this, _1,
00167                                                                               _2);
00168   dsrv_->setCallback(cb);
00169 }
