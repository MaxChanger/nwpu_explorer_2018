#使用 tf 的 now() 和 Time(0)的区别 
http://blog.csdn.net/github_35160620/article/details/52473291

为什么要讲这个，这是因为 ROS 的 tf 在进行坐标之间的转换变换不是实时的转换，它有一个缓冲器来存放一段时间的坐标转换数据，所以有时，可能没有当前时间的坐标转换数据，使用 lookupTransform() 函数就可以得到你想的某个时间的坐标数据，前提是缓冲区里要有这个时间的坐标数据，tf 的坐标转换是自动计算的，所以有时，你想得到当前的时间的坐标转换数据，你调用 lookupTransform() 函数去获取，但是缓冲器里还没有来得及去计算现在的坐标转换数据，就是说：现在还没有。如果你非要获取，就会出错，所以你要使用 waitForTransform() 函数来等待 tf 的坐标转换线程得到你想要的时间点的坐标转换数据。 
简单的说：waitForTransform() 就是一个安全程序。


##trajectory_pub_ = nh.advertise<nav_msgs::Path>("trajectory", 1, true);

trajectory_provider_service_ = nh.advertiseService("trajectory", &PathContainer::trajectoryProviderCallBack, this);
    ///回复exploration_planner的client

recovery_info_provider_service_ = nh.advertiseService("trajectory_recovery_info", &PathContainer::recoveryInfoProviderCallBack, this);
