#!/usr/bin/env python


import rospy
import std_msgs.msg
import phidgets.msg
import geometry_msgs.msg
import math
from nav_msgs.msg import Odometry
import tf
#####################################################
#               Initialize Variables                #
#####################################################
ENCODER_LEFT = 0
ENCODER_RIGHT = 0
LINEAR_VELOCITY = 0.0
ANGULAR_VELOCITY = 0.0
RESET = False

#####################################################
#             /left_motor/encoder Callback          #
#####################################################
def update_feedback_enc_left(feedback_enc):
    global ENCODER_LEFT
    ENCODER_LEFT = ENCODER_LEFT + feedback_enc.count_change


#	self.FEEDBACK_ENC_UPDATED = True

#####################################################
#             /right_motor/encoder Callback         #
#####################################################
def update_feedback_enc_right(feedback_enc):
    global ENCODER_RIGHT
    # NOTE THE MINUS SIGN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ENCODER_RIGHT = ENCODER_RIGHT -feedback_enc.count_change

#####################################################
#                  /odom_reset Callback             #
#####################################################
def reset_feedback(feedback_reset):
    global RESET
    if feedback_reset.data == True:
        RESET = True
    else:
        pass


#####################################################
#               Initialize Publisher                #
#####################################################
rospy.init_node('odom_publish_node', anonymous=True)
pub_odom = rospy.Publisher('/robot_odom', Odometry, queue_size=1)
rate = rospy.Rate(10)

rospy.Subscriber('/left_motor/encoder', phidgets.msg.motor_encoder, update_feedback_enc_left)
rospy.Subscriber('/right_motor/encoder', phidgets.msg.motor_encoder, update_feedback_enc_right)
rospy.Subscriber('/odom_reset', std_msgs.msg.Bool, reset_feedback)
#rospy.spin()

#####################################################
#            Controller Function                    #
#####################################################
def publisher():
    global LINEAR_VELOCITY, ANGULAR_VELOCITY, ENCODER_LEFT, ENCODER_RIGHT,RESET

    ODOM = Odometry()
    base = 0.21 
    wheel_radius = 0.0485 
    sita = 0
    x = 0
    y = 0
    
    pi = 3.14
    control_frequency = 10
    dt = 1.0/control_frequency
    ticks_per_rev = 897.96
    #ticks_per_rev = 3591.84
    while not rospy.is_shutdown():
        current_time = rospy.get_rostime()
        # enc_left and enc_right are local variables for left and right encoders
        encoder_left = ENCODER_LEFT
        encoder_right = ENCODER_RIGHT
        diff = 0	# if difference b/w left and right encoder becomes less than diff, make them equal
    
	'''
        if (abs(ENCODER_RIGHT - ENCODER_LEFT) <= diff):
            #print('entered loop')
            max_val = max(ENCODER_RIGHT, ENCODER_LEFT)
            #print(max_val)
            encoder_left = max_val
            encoder_right = max_val
        #print(encoder_left)	
        '''

        v_left = wheel_radius*(encoder_left * 2 * pi * control_frequency) / (ticks_per_rev)
        v_right = wheel_radius*(encoder_right * 2 * pi * control_frequency) / (ticks_per_rev)
        sita = sita + (1/base) * dt * (v_right - v_left)
        #print(sita )
        x = x + 0.5 * (v_left + v_right) * math.cos(sita)*dt
        y = y + 0.5 * (v_left + v_right) * math.sin(sita)*dt
        if RESET:
            x = 0
            y = 0
            sita = 0
            RESET = False
        else:
            pass       

        print(x, y)
        ODOM.pose.pose.position.x = x
        ODOM.pose.pose.position.y = y
        
        quaternion = tf.transformations.quaternion_from_euler(0,0,sita)
        #ODOM.pose.pose.orientation.z = sita
        #ODOM.pose.pose.orientation.w = sita
        #print(quaternion[3])
        ODOM.pose.pose.orientation.z = quaternion[2]
        ODOM.pose.pose.orientation.w = quaternion[3]
	#ODOM.twist.twist.angular.z = sita
        ODOM.header.stamp = current_time
        ODOM.header.frame_id = "odom"
        ODOM.child_frame_id = "base_link"
        pub_odom.publish(ODOM)

	# flush the encoders
	ENCODER_RIGHT = 0
	ENCODER_LEFT = 0

        rate.sleep()


#####################################################
#                Main Function                      #
#####################################################
if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
