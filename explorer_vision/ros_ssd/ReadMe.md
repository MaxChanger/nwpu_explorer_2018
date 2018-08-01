# ROS_SSD 
基于[SSD-Tensorflow](https://github.com/balancap/SSD-Tensorflow)创建的ROS接口，用于explorer救援机器人视觉识别。

## 依赖
需要[SSD-Tensorflow](https://github.com/balancap/SSD-Tensorflow)源码。

将SSD-Tensorflow源码添加软链接至lib文件夹内，修改visualization.py文件夹bboxes_draw_on_img函数，添加返回值`return img`

## 使用
```bash
roslaunch ros_ssd ssd_detect.launch
```
```bash
launch参数:
'--model 指定模型路径'
'--cameraSubscriber 指定摄像头发布消息话题路径，需要指定为compressed话题，默认/camera/image_raw/compressed'
'--numclass 类型数目，默认为2'
'--pubtopic 节点发布话题 默认为/camera/detect_frame'
'--caminfo 摄像头信息，默认为/camera/camera_info'
```