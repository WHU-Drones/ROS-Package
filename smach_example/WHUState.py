#!/usr/bin/env python
import rospy
import smach
import smach_ros


# define state wState
# w_num_trans must be lager than 0, other parameters must be positive
class State(smach.State):

  def __init__( self,
                w_name_topic_in,  w_name_type_in,
                w_name_topic_out, w_name_type_out,
                w_num_serve_in,   w_num_action_in,
                w_num_serve_out,  w_num_action_out,
                w_size_topic_out,
                w_num_trans ):

    w_num_topic_in = len(w_name_topic_in)
    w_num_topic_out = len(w_name_topic_out)

    w_trans_name_list = []
    w_input_name_list = []
    w_output_name_list = []

    #transition name list
    for i in range(1, (w_num_trans + 1)):
      w_trans_name_list.append('outcome' + str(i))

    #IN
    #topic subscriber
    w_topic_input_name_list = []
    self.w_got = []
    self.w_data_sub = []
    self.w_sub = {}
    for i in range(1, (w_num_topic_in + 1)):
      w_topic_input_name_list.append('subscriber' + str(i))
      self.w_got.append(float('inf'))
      self.w_data_sub.append(float('inf'))
      self.w_sub['subscriber' + str(i)] = rospy.Subscriber(w_name_topic_in[i - 1],
                                                        w_name_type_in[i - 1], self.callback, (i))
    #serve server
    w_serve_input_name_list = []
    for i in range(1, (w_num_serve_in + 1)):
      w_serve_input_name_list.append('server' + str(i))
    #action action_server
    w_action_input_name_list = []
    for i in range(1, (w_num_action_in + 1)):
      w_action_input_name_list.append('actionserver' + str(i))

    #OUT
    #topic publisher
    w_topic_output_name_list = []
    self.w_pub = {}
    for i in range(1, (w_num_topic_out + 1)):
      w_topic_output_name_list.append('publisher' + str(i))
      self.w_pub['publisher' + str(i)] = rospy.Publisher( w_name_topic_out[i - 1], w_name_type_out[i - 1],
                                                        queue_size = w_size_topic_out[i - 1])
    #serve client
    w_serve_output_name_list = []
    for i in range(1, (w_num_serve_out + 1)):
      w_serve_output_name_list.append('client' + str(i))
    #action action_client
    w_action_output_name_list = []
    for i in range(1, (w_num_action_out + 1)):
      w_action_output_name_list.append('actionclient' + str(i))

    #INIT
    w_input_name_list = w_topic_input_name_list + w_serve_input_name_list + w_action_input_name_list
    smach.State.__init__( self,
                          outcomes = w_trans_name_list + ['outcome_self'],
                          input_keys = w_input_name_list,
                          output_keys = w_output_name_list  )

  def callback(self, data, args):
    #for i in range(1, len(self.w_data_sub) + 1):
    self.w_data_sub[args - 1] = data
    self.w_got[args - 1] = 1
    # if self.w_data_sub[args - 1] == float('inf'):
    #   #rospy.loginfo('Not Got!')
    #   self.w_got[args - 1] = float('inf')
    # else:
    #   #rospy.loginfo('Got!')
    #   self.w_got[args - 1] = 1

  def refresh(self):      
      self.w_got = [float('inf') for _ in self.w_got]
      self.w_data_sub = [float('inf') for _ in self.w_data_sub]
 
  def execute(self, userdata):
    raise NotImplementedError()