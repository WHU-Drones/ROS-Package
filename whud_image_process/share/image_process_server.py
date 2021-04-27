#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String,Float64,Bool
from darknet_ros_msgs.msg import BoundingBox,BoundingBoxes,ObjectCount
from math import atan2
#FOCAL is the focal state computed by getstate.py
FOCAL_LENGTH=449
IMAGE_WIDTH=640
IMAGE_HEIGHT=480
KNOWN_WIDTH=12.5
class Getstate:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,callback=self.stateCallback)
        self.publisher_x = rospy.Publisher("/y_state",Float64,queue_size=100)
        self.publisher_y = rospy.Publisher("/x_state",Float64,queue_size=100)
        self.publisher_x_setpoint = rospy.Publisher("/x_setpoint",Float64,queue_size=100)
        self.publisher_y_setpoint = rospy.Publisher("/y_setpoint",Float64,queue_size=100)
        self.enable_pid=rospy.Publisher("/enable_pid",Bool,queue_size=100)
        self.y_state=0
        self.x_state=0
        self.rate = rospy.Rate(100)
        rospy.spin()      

    def state_to_camera(self,knownWidth, focalLength, pixelWidth):
        return (knownWidth * focalLength) / pixelWidth

    def stateCallback(self, data):
        #rospy.loginfo(data)
        #print(len(data.bounding_boxes))
        if len(data.bounding_boxes):
            probability=0
            final_obj=None
            #width=0
            #height=0
            #center=(-1,-1)
            #angle=0
            #state=0
            for obj in data.bounding_boxes:
                #Class below is which class we need to detect, can be maintained be a global variable
                if obj.Class=='target':
                    if obj.probability>probability:
                        final_obj=obj
                        probability=obj.probability
                        #width=obj.xmax-obj.xmin
                        #height=obj.ymax-obj.ymin
                        #center=((obj.xmin+obj.xmax)/2,(obj.ymin+obj.ymax)/2)
                        #angle=atan2(-(center[1]-IMAGE_HEIGHT/2),(center[0]-IMAGE_WIDTH))
                        #state=self.state_to_camera(KNOWN_WIDTH,FOCAL_LENGTH,width)
            if probability>0:
                self.y_state = -((final_obj.xmax+final_obj.xmin)/2-IMAGE_WIDTH/2)
                self.x_state = ((final_obj.ymax+final_obj.ymin)/2-IMAGE_HEIGHT/2)

        self.publisher_x.publish(self.y_state)
        self.publisher_y.publish(self.x_state)
        self.publisher_x_setpoint.publish(0)
        self.publisher_y_setpoint.publish(0)
        self.enable_pid.publish(True)
        #elf.rate.sleep()

    
if __name__=='__main__':
    #Create a node 
    rospy.init_node("image_process_node",anonymous=True)
    ImageDealProcess = Getstate()
