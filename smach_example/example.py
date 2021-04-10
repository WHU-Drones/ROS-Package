#!/usr/bin/env python
import rospy
import smach
import smach_ros
import WHUState
from geometry_msgs.msg import Twist

class WAIT(WHUState.State):
    def __init__(self):
        WHUState.State.__init__(self, w_name_topic_in = ['/TOPIC_IN'], w_name_type_in = [Twist], w_name_topic_out = [], w_name_type_out = [], w_num_serve_in = 0, w_num_action_in = 0, w_num_serve_out = 0, w_num_action_out = 0, w_size_topic_out = [1000], w_input_keys = [], w_output_keys = [], w_num_trans = 1)
		self.refresh()
		self.c_got = self.w_got							# A list idicates whether getting msg from topic.
		self.c_data_sub = self.w_data_sub		# A list includes the data subscribed from topic.
		self.c_pub = self.w_pub   					# Publisher self.c_pub['publisherN'], where N is 1 to len(topic_out).

    def execute(self, userdata):
        self.c_got = self.w_got
        self.c_data_sub = self.w_data_sub
        # self.refresh()
        #------------------------------------------------------------#
        # outcomes: indicate the next state.
        # 'outcome_self': jump to self.
        # 'outcomtN' where N is w_num_trans: jump to other states.
        # When jump to other states, FUNC slef.refresh() is necessary before return.
        # Add your code here.
        if self.c_got[0] == 1:
            rospy.loginfo('Data Recieved!')    
            self.refresh()
            return 'outcome1'
        else:
            rospy.loginfo('No Data Recieved!')      
            return 'outcome_self'
        # Code end.
        #------------------------------------------------------------#

class FLAY_TO_A(WHUState.State):
    def __init__(self):
        WHUState.State.__init__(self, w_name_topic_in = ['/TOPIC_IN'], w_name_type_in = [Twist], w_name_topic_out = [], w_name_type_out = [], w_num_serve_in = 0, w_num_action_in = 0, w_num_serve_out = 0, w_num_action_out = 0, w_size_topic_out = [1000], w_input_keys = [], w_output_keys = [], w_num_trans = 1)
		self.refresh()
		self.c_got = self.w_got							# A list idicates whether getting msg from topic.
		self.c_data_sub = self.w_data_sub		# A list includes the data subscribed from topic.
		self.c_pub = self.w_pub   					# Publisher self.c_pub['publisherN'], where N is 1 to len(topic_out).
        
    def execute(self, userdata):
        self.c_got = self.w_got
        self.c_data_sub = self.w_data_sub
        # self.refresh()
        #------------------------------------------------------------#
        # outcomes: indicate the next state.
        # 'outcome_self': jump to self.
        # 'outcomtN' where N is w_num_trans: jump to other states.
        # When jump to other states, FUNC slef.refresh() is necessary before return.
        # Add your code here.
        if self.c_got[0] == 1:
            rospy.loginfo('Data Recieved!')
            self.refresh()      
            return 'outcome1'    
        else:
            rospy.loginfo('No Data Recieved!')      
            return 'outcome_self'
        # Code end.
        #------------------------------------------------------------#

class DROP(WHUState.State):
    def __init__(self):
        WHUState.State.__init__(self, w_name_topic_in = ['/TOPIC_IN'], w_name_type_in = [Twist], w_name_topic_out = [], w_name_type_out = [], w_num_serve_in = 0, w_num_action_in = 0, w_num_serve_out = 0, w_num_action_out = 0, w_size_topic_out = [1000], w_input_keys = [], w_output_keys = [], w_num_trans = 3)
		self.refresh()
		self.c_got = self.w_got							# A list idicates whether getting msg from topic.
		self.c_data_sub = self.w_data_sub		# A list includes the data subscribed from topic.
		self.c_pub = self.w_pub   					# Publisher self.c_pub['publisherN'], where N is 1 to len(topic_out).

    def execute(self, userdata):
        self.c_got = self.w_got
        self.c_data_sub = self.w_data_sub
        # self.refresh()
        #------------------------------------------------------------#
        # outcomes: indicate the next state.
        # 'outcome_self': jump to self.
        # 'outcomtN' where N is w_num_trans: jump to other states.
        # When jump to other states, FUNC slef.refresh() is necessary before return.
        # Add your code here.
        if self.c_got[0] == 1 and self.c_data_sub[0] == 1:
            rospy.loginfo('Data Recieved!')
            self.refresh()      
            return 'outcome1'
        elif self.c_got[0] == 1 and self.c_data_sub[0] == 10:
            rospy.loginfo('Data Recieved!')
            self.refresh()      
            return 'outcome2'
        elif self.c_got[0] == 1 and self.c_data_sub[0] == 100:
            rospy.loginfo('Data Recieved!')
            self.refresh()      
            return 'outcome3'   
        else:
            rospy.loginfo('No Data Recieved!')      
            return 'outcome_self'
        # Code end.
        #------------------------------------------------------------#

class FLAY_ABOVE_BD(WHUState.State):
    def __init__(self):
        WHUState.State.__init__(self, w_name_topic_in = ['/TOPIC_IN'],  w_name_type_in = [Twist], w_name_topic_out = [], w_name_type_out = [], w_num_serve_in = 0, w_num_action_in = 0, w_num_serve_out = 0, w_num_action_out = 0, w_size_topic_out = [1000], w_input_keys = [], w_output_keys = [], w_num_trans = 1)
		self.refresh()
		self.c_got = self.w_got							# A list idicates whether getting msg from topic.
		self.c_data_sub = self.w_data_sub		# A list includes the data subscribed from topic.
		self.c_pub = self.w_pub   					# Publisher self.c_pub['publisherN'], where N is 1 to len(topic_out).

    def execute(self, userdata):
        self.c_got = self.w_got
        self.c_data_sub = self.w_data_sub
        # self.refresh()
        #------------------------------------------------------------#
        # outcomes: indicate the next state.
        # 'outcome_self': jump to self.
        # 'outcomtN' where N is w_num_trans: jump to other states.
        # When jump to other states, FUNC slef.refresh() is necessary before return.
        # Add your code here.
        if self.c_got[0] == 1:
            rospy.loginfo('Data Recieved!')
            self.refresh()      
            return 'outcome1'    
        else:
            rospy.loginfo('No Data Recieved!')      
            return 'outcome_self'
        # Code end.
        #------------------------------------------------------------#

class FLAY_TO_B(WHUState.State):
    def __init__(self):
        WHUState.State.__init__(self, w_name_topic_in = ['/TOPIC_IN'], w_name_type_in = [Twist], w_name_topic_out = [], w_name_type_out = [], w_num_serve_in = 0, w_num_action_in = 0, w_num_serve_out = 0, w_num_action_out = 0, w_size_topic_out = [1000], w_input_keys = [], w_output_keys = [], w_num_trans = 1)
		self.refresh()
		self.c_got = self.w_got							# A list idicates whether getting msg from topic.
		self.c_data_sub = self.w_data_sub		# A list includes the data subscribed from topic.
		self.c_pub = self.w_pub   					# Publisher self.c_pub['publisherN'], where N is 1 to len(topic_out).

    def execute(self, userdata):
        self.c_got = self.w_got
        self.c_data_sub = self.w_data_sub
        # self.refresh()
        #------------------------------------------------------------#
        # outcomes: indicate the next state.
        # 'outcome_self': jump to self.
        # 'outcomtN' where N is w_num_trans: jump to other states.
        # When jump to other states, FUNC slef.refresh() is necessary before return.
        # Add your code here.
        if self.c_got[0] == 1:
            rospy.loginfo('Data Recieved!')
            self.refresh()      
            return 'outcome1'    
        else:
            rospy.loginfo('No Data Recieved!')      
            return 'outcome_self'
        # Code end.
        #------------------------------------------------------------#

class FLAY_ABOVE_L(WHUState.State):
    def __init__(self):
        WHUState.State.__init__(self, w_name_topic_in = ['/TOPIC_IN'], w_name_type_in = [Twist], w_name_topic_out = [], w_name_type_out = [], w_num_serve_in = 0, w_num_action_in = 0, w_num_serve_out = 0, w_num_action_out = 0, w_size_topic_out = [1000], w_input_keys = [], w_output_keys = [], w_num_trans = 1)
		self.refresh()
		self.c_got = self.w_got							# A list idicates whether getting msg from topic.
		self.c_data_sub = self.w_data_sub		# A list includes the data subscribed from topic.
		self.c_pub = self.w_pub   					# Publisher self.c_pub['publisherN'], where N is 1 to len(topic_out).

    def execute(self, userdata):
        self.c_got = self.w_got
        self.c_data_sub = self.w_data_sub
        # self.refresh()
        #------------------------------------------------------------#
        # outcomes: indicate the next state.
        # 'outcome_self': jump to self.
        # 'outcomtN' where N is w_num_trans: jump to other states.
        # When jump to other states, FUNC slef.refresh() is necessary before return.
        # Add your code here.
        if self.c_got[0] == 1:
            rospy.loginfo('Data Recieved!')
            self.refresh()      
            return 'outcome1'    
        else:
            rospy.loginfo('No Data Recieved!')      
            return 'outcome_self'
        # Code end.
        #------------------------------------------------------------#

class FLAY_TO_C(WHUState.State):
    def __init__(self):
        WHUState.State.__init__(self, w_name_topic_in = ['/TOPIC_IN'], w_name_type_in = [Twist], w_name_topic_out = [], w_name_type_out = [], w_num_serve_in = 0, w_num_action_in = 0, w_num_serve_out = 0, w_num_action_out = 0, w_size_topic_out = [1000], w_input_keys = [], w_output_keys = [], w_num_trans = 1)
		self.refresh()
		self.c_got = self.w_got							# A list idicates whether getting msg from topic.
		self.c_data_sub = self.w_data_sub		# A list includes the data subscribed from topic.
		self.c_pub = self.w_pub   					# Publisher self.c_pub['publisherN'], where N is 1 to len(topic_out).

    def execute(self, userdata):
        self.c_got = self.w_got
        self.c_data_sub = self.w_data_sub
        # self.refresh()
        #------------------------------------------------------------#
        # outcomes: indicate the next state.
        # 'outcome_self': jump to self.
        # 'outcomtN' where N is w_num_trans: jump to other states.
        # When jump to other states, FUNC slef.refresh() is necessary before return.
        # Add your code here.
        if self.c_got[0] == 1:
            rospy.loginfo('Data Recieved!')
            self.refresh()      
            return 'outcome1'    
        else:
            rospy.loginfo('No Data Recieved!')      
            return 'outcome_self'
        # Code end.
        #------------------------------------------------------------#

class FLAY_BACK(WHUState.State):
    def __init__(self):
        WHUState.State.__init__(self, w_name_topic_in = ['/TOPIC_IN'], w_name_type_in = [Twist], w_name_topic_out = [], w_name_type_out = [], w_num_serve_in = 0, w_num_action_in = 0, w_num_serve_out = 0, w_num_action_out = 0, w_size_topic_out = [1000], w_input_keys = [], w_output_keys = [], w_num_trans = 1)
		self.refresh()
		self.c_got = self.w_got							# A list idicates whether getting msg from topic.
		self.c_data_sub = self.w_data_sub		# A list includes the data subscribed from topic.
		self.c_pub = self.w_pub   					# Publisher self.c_pub['publisherN'], where N is 1 to len(topic_out).

    def execute(self, userdata):
        self.c_got = self.w_got
        self.c_data_sub = self.w_data_sub
        # self.refresh()
        #------------------------------------------------------------#
        # outcomes: indicate the next state.
        # 'outcome_self': jump to self.
        # 'outcomtN' where N is w_num_trans: jump to other states.
        # When jump to other states, FUNC slef.refresh() is necessary before return.
        # Add your code here.
        if self.c_got[0] == 1:
            rospy.loginfo('Data Recieved!')
            self.refresh()      
            return 'outcome1'    
        else:
            rospy.loginfo('No Data Recieved!')      
            return 'outcome_self'
        # Code end.
        #------------------------------------------------------------#

class LAND(WHUState.State):
    def __init__(self):
        WHUState.State.__init__(self, w_name_topic_in = ['/TOPIC_IN'], w_name_type_in = [Twist], w_name_topic_out = [], w_name_type_out = [], w_num_serve_in = 0, w_num_action_in = 0, w_num_serve_out = 0, w_num_action_out = 0, w_size_topic_out = [1000], w_input_keys = [], w_output_keys = [], w_num_trans = 1)
		self.refresh()
		self.c_got = self.w_got							# A list idicates whether getting msg from topic.
		self.c_data_sub = self.w_data_sub		# A list includes the data subscribed from topic.
		self.c_pub = self.w_pub   					# Publisher self.c_pub['publisherN'], where N is 1 to len(topic_out).

    def execute(self, userdata):
        self.c_got = self.w_got
        self.c_data_sub = self.w_data_sub
        # self.refresh()
        #------------------------------------------------------------#
        # outcomes: indicate the next state.
        # 'outcome_self': jump to self.
        # 'outcomtN' where N is w_num_trans: jump to other states.
        # When jump to other states, FUNC slef.refresh() is necessary before return.
        # Add your code here.
        if self.c_got[0] == 1:
            rospy.loginfo('Data Recieved!')
            self.refresh()      
            return 'outcome1'    
        else:
            rospy.loginfo('No Data Recieved!')      
            return 'outcome_self'
        # Code end.
        #------------------------------------------------------------#

# Here is an example.
def main():
    rospy.init_node("w_test")
    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add( 'WAIT', WAIT(), transitions={'outcome_self':'WAIT', 'outcome1':'FLAY_TO_A'})
        smach.StateMachine.add( 'FLAY_TO_A', FLAY_TO_A(), transitions={'outcome_self':'FLAY_TO_A', 'outcome1':'DROP'})
        smach.StateMachine.add( 'DROP', DROP(), transitions={'outcome_self':'DROP', 'outcome1':'FLAY_ABOVE_BD', 'outcome2':'FLAY_ABOVE_L', 'outcome3':'FLAY_BACK'})
        smach.StateMachine.add( 'FLAY_ABOVE_BD', FLAY_ABOVE_BD(), transitions={'outcome_self':'FLAY_ABOVE_BD', 'outcome1':'FLAY_TO_B'})
        smach.StateMachine.add( 'FLAY_TO_B', FLAY_TO_B(), transitions={'outcome_self':'FLAY_TO_B', 'outcome1':'DROP'})
        smach.StateMachine.add( 'FLAY_ABOVE_L', FLAY_ABOVE_L(), transitions={'outcome_self':'FLAY_ABOVE_L', 'outcome1':'FLAY_TO_C'})
        smach.StateMachine.add( 'FLAY_TO_C', FLAY_TO_C(), transitions={'outcome_self':'FLAY_TO_C', 'outcome1':'DROP'})
        smach.StateMachine.add( 'FLAY_BACK', FLAY_BACK(), transitions={'outcome_self':'FLAY_BACK', 'outcome1':'LAND'})
        smach.StateMachine.add( 'LAND', LAND(), transitions={'outcome_self':'LAND', 'outcome1':'DONE'})
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__=="__main__":
    main()
