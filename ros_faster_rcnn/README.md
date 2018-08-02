# ROS_faster_rcnn
A [ROS] (http://www.ros.org/) wrapper for the [python implementation] (https://github.com/rbgirshick/py-faster-rcnn) 
of [faster-RCNN] (https://github.com/ShaoqingRen/faster_rcnn). This wrapper is based on `demo.py`, 
that is included in the python implementation. It publishes messages containing the class, position, 
size and probabiity of the detected objects in the received images.
> **Faster** R-CNN is an object detection framework based on deep convolutional networks, 
which includes a Region Proposal Network (RPN) and an Object Detection Network. Both networks are trained for sharing convolutional layers for fast testing. 
> 
> Faster R-CNN was initially described in an [arXiv tech report](http://arxiv.org/abs/1506.01497).

## Installation
- Clone the repository
 - `git clone https://github.com/ChielBruin/ros_faster_rcnn.git --recursive`
 - run `git submodule update --init --recursive`, when the modules are not correctly cloned
- Install py-faster-rcnn located in the libraries folder
 - Follow the guide provided [here](https://github.com/rbgirshick/py-faster-rcnn#installation-sufficient-for-the-demo)
 - If you are running Ubuntu 15 or 16, check [this](https://gist.github.com/wangruohui/679b05fcd1466bb0937f) or 
 [this](https://github.com/BVLC/caffe/wiki/Ubuntu-16.04-or-15.10-Installation-Guide) guide (respectively) for the installation of the caffe dependency
- Install all the needed ROS dependencies
 - run `rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO`, where $ROS_DISTRO is your desired version of ROS
 
## Development notes
This ROS node is being developed as a part of 
[this](https://github.com/ChielBruin/tomatenplukkers) repository. The wrapper functions correctly, but some features are still missing:
- A ROS service to send an image and receive the detections
 - This function will execute on the CPU due to 
 [this] (http://answers.ros.org/question/240998/py-faster-rcnn-network-detection-runs-on-cpu-through-ros-callback-function/) issue, therefore it is tricky to implement correctly
