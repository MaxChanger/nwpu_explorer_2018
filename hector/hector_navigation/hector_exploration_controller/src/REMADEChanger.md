# 调整速度大小

    twist.linear.x *= 0.18;
    twist.linear.y *= 0.18*1.2;
    twist.angular.z *= 0.18*1.6;//调整大小
    ROS_ERROR("[hector_exploration_controller] vel pub %lf %lf ==>%lf", twist.linear.x, twist.linear.y ,twist.angular.z);
    vel_pub_.publish(twist);//发送给底盘

##controller的几个作用：

1.调用timerPlanExploration 请求(call) exploration_node给他一个探索路径的回复（并且广播了出去）  
  然后node调用planner里边的doExploration
  返回的答案存在了srv_exploration_plan里面  
  然后又调用了follower里边的setplan函数
  setplan函数用到了刚才返回的结果作为参数  然后setPlan又调用follower里边的transformGlobalPlan函数 结果存在了global_plan_中
  最后global_plan_经过下面computeVelocityCommands计算速度的时候调用了
2.调用timerCmdVelGeneration 然后调用timerCmdVelGeneration调用follower 里边的 computeVelocityCommands（计算下一步的速度）
  computeVelocityCommands用了global_plan_的数据 最后生成要发布的速度存在了twist里边 发给底盘