##ROS消息头
像其他类型消息一样， sensor_msgs/LaserScan和sensor_msgs/PointCloud消息包含TF坐标系和时间关联的消息。为了标准化此信息的发送方式， Header消息类型用作所有消息中的字段
Header类型中有三个字段如下所示。seq字段对应于标识符的消息由发布者发送时候自动增加。stamp字段存储与数据相关联的时间信息。以激光为例，stamp可能对应于扫描发生时的时间。frame_id字段存储与数据相关联的TF坐标系的消息。以激光为例，应该是设置为进行扫描的坐标系。


##LaserScan消息
对于具有激光扫描仪的机器人，ROS在sensor_msgs包提供特殊的消息类型，叫LaserScan，用于保存有关扫描的信息。主要从扫描仪返回的数据可以格式化成这种消息类型，就可以使代码更容易处理。在谈论如何生成和发布这些消息之前，让我们来看看消息规范：

Header header            # Header也是一个结构体,包含了seq,stamp,frame_id,其中seq指的是扫描顺序增加的id,stamp包含了开始扫描的时间和与开始扫描的时间差,frame_id是扫描的参考系名称.注意扫描是逆时针从正前方开始扫描的.

float32 angle_min        # 开始扫描的角度(角度)[rad]
float32 angle_max        # 结束扫描的角度(角度)
float32 angle_increment  # 每一次扫描增加的角度(角度)

float32 time_increment   # 测量的时间间隔(s)
float32 scan_time        # 扫描的时间间隔(s)

float32 range_min        # 距离最小值(m)
float32 range_max        # 距离最大值(m)

float32[] ranges         # 距离数组(长度360)
float32[] intensities    # 与设备有关,强度数组(长度360)


##tf
要为我们的简单示例创建一个变换树，我们将创建两个节点，一个用于“base_link”坐标系，一个用于“base_laser”坐标系。
为了创建它们之间的方向，我们首先需要决定哪个节点是父节点，哪些节点是子节点。
记住，这种区别很重要，因为tf假定所有转换都从父对象移动到子对象。
让我们选择“base_link”坐标系作为父节点，因为随着其他部分/传感器被添加到机器人，通过遍历“base_link”框架，它们将最有意义地与“base_laser”坐标系相关。
这意味着连接“base_link”和“base_laser”的变换应为（x：0.1m，y：0.0m，z：0.2m）。使用这个变换树设置，将在“base_laser”坐标系中接收到的激光扫描转换为“base_link”坐标系就像调用tf库一样简单。
我们的机器人可以使用这些信息来理解“base_link”框架中的激光扫描，并安全地计划避开环境中的障碍物。