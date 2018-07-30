import rospy
from std_msgs.msg import String
def callback(str):
	pass


if __name__=="__main__":
	rospy.init_node("start")
	rospy.logerr("script start")
	rospy.Subscrier("lll",String,callback)
	rospy.spin()