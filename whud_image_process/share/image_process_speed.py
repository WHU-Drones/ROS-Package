#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

class ImageSpeedControl:
    def __init__(self):
        self.subscriber_x = rospy.Subscriber("/x_speed/control_effort",Float64,callback=self.xspeed)
        self.subscriber_y = rospy.Subscriber("/y_speed/control_effort",Float64,callback=self.yspeed)
        self.publisher = rospy.Publisher("/image_speed_control",Odometry)
        self.xflag_publish = False
        self.yflag_publish = False
        self.ratio = 5000
        self.limit = 0.1
        self.speed = Odometry()
        rospy.spin()     

    def xspeed(self,data):
        self.speed.twist.twist.linear.x = data.data/self.ratio
        if self.speed.twist.twist.linear.x > self.limit:
            self.speed.twist.twist.linear.x = self.limit
        elif self.speed.twist.twist.linear.x < -self.limit:
            self.speed.twist.twist.linear.x = -self.limit
        self.xflag_publish = True
        if self.xflag_publish==True and self.yflag_publish==True:
            self.publisher.publish(self.speed)
            self.xflag_publish = self.yflag_publish = False

    def yspeed(self,data):
        self.speed.twist.twist.linear.y = data.data/self.ratio
        if self.speed.twist.twist.linear.y > self.limit:
            self.speed.twist.twist.linear.y = self.limit
        elif self.speed.twist.twist.linear.y < -self.limit:
            self.speed.twist.twist.linear.y = -self.limit
        self.yflag_publish = True

if __name__=='__main__':
    #Create a node 
    rospy.init_node("image_speed_control_node",anonymous=True)
    ImageSpeed = ImageSpeedControl()