#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de
from cmath import pi
from re import S
import rospy,math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import Imu

Target_Angle = 0
Position_KP =0

lf_angle = 0
lb_angle = 0
rf_angle = 0
rb_angle = 0
base_angle  = 0

body_angle = 0

def imu_calc(data):

    rf_x   = data.orientation.x
    rf_y   = data.orientation.y
    rf_z   = data.orientation.z
    rf_w  = data.orientation.w
    global body_angle 
    body_angle = math.atan2(2 * (rf_w * rf_z + rf_x * rf_y), 1 - 2 * (rf_y**2 + rf_z**2))
    pub_debug_body_angle = rospy.Publisher('/debug/feedback_body_angle', Float64, queue_size=1)
    pub_debug_body_angle.publish(body_angle)


def get_target_angle(data):
    global Target_Angle
    Target_Angle  =data.data

def get_kp(data):
    global Position_KP
    Position_KP  =data.data

def constrain_npi_pi(value):
    if(value>pi):
        value = value - pi
    elif value < -pi:
        value = value + pi

def constrain_bias(value):
    if(value>pi):
        value = 2*pi - value
    elif value < -pi:
        value = value + 2*pi    
    return value

def set_steering_wheel_velocity(data):

    global lf_angle 
    global lb_angle 
    global rf_angle 
    global rb_angle

    global Position_KP 
    global Target_Angle

    # Position_KI =0.1
    # Position_KD =500

    Target_Angle = data.angular.z

    # Target_Angle = 0.7586

    Bias_1 =  lf_angle- Target_Angle
    Bias_2 =  lb_angle- Target_Angle
    Bias_3 =  rf_angle- Target_Angle
    Bias_4 =  rb_angle- Target_Angle

    # Integral_bias = Bias + Integral_bias
    # Correct_V_lf=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias)
    # Last_Bias=Bias
    # return Correct_V_lf

    Bias_1_out = constrain_bias(Bias_1)
    Bias_2 = constrain_bias(Bias_2)
    Bias_3 = constrain_bias(Bias_3)
    Bias_4 = constrain_bias(Bias_4)

    Position_KP = 5

    pid_out_1 = Bias_1_out * Position_KP
    pid_out_2 = Bias_2 * Position_KP
    pid_out_3 = Bias_3 * Position_KP
    pid_out_4 = Bias_4 * Position_KP

    # pub_Bias_1 = rospy.Publisher('/debug/Bias_1', Float64, queue_size=1)
    # pub_Bias_1.publish(Bias_1)
    # pub_Bias_1_out = rospy.Publisher('/debug/Bias_1_out', Float64, queue_size=1)
    # pub_Bias_1_out.publish(Bias_1_out)
    # pub_pid_out_1 = rospy.Publisher('/debug/pid_out_1', Float64, queue_size=1)
    # pub_pid_out_1.publish(pid_out_1)

    # a  = 0.095    #for mini_mec
    # b  = 0.0875    #for mini_mec

    Init_V = 2

    Vlf_1 = -(data.linear.x + pid_out_1)
    Vlf_2 = -(data.linear.x -  pid_out_1)

    Vlb_1= -(data.linear.x  + pid_out_2)
    Vlb_2= -(data.linear.x - pid_out_2)

    Vrf_1= -(data.linear.x + pid_out_3)
    Vrf_2= -(data.linear.x  - pid_out_3)

    Vrb_1= -(data.linear.x + pid_out_4)
    Vrb_2= -(data.linear.x  - pid_out_4)

    pub_vel_steering_wheel_left_1_front_wheel = rospy.Publisher('/ehang/steering_wheel_left_front_1_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_steering_wheel_left_2_front_wheel = rospy.Publisher('/ehang/steering_wheel_left_front_2_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_steering_wheel_left_1_rear_wheel = rospy.Publisher('/ehang/steering_wheel_left_rear_1_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_steering_wheel_left_2_rear_wheel = rospy.Publisher('/ehang/steering_wheel_left_rear_2_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_steering_wheel_right_1_front_wheel = rospy.Publisher('/ehang/steering_wheel_right_front_1_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_steering_wheel_right_2_front_wheel = rospy.Publisher('/ehang/steering_wheel_right_front_2_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_steering_wheel_right_1_rear_wheel = rospy.Publisher('/ehang/steering_wheel_right_rear_1_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_steering_wheel_right_2_rear_wheel = rospy.Publisher('/ehang/steering_wheel_right_rear_2_wheel_velocity_controller/command', Float64, queue_size=1)


    pub_vel_steering_wheel_left_1_front_wheel.publish(Vlf_1)
    pub_vel_steering_wheel_left_2_front_wheel.publish(Vlf_2)
    pub_vel_steering_wheel_left_1_rear_wheel.publish(Vlb_1)
    pub_vel_steering_wheel_left_2_rear_wheel.publish(Vlb_2)
    pub_vel_steering_wheel_right_1_front_wheel.publish(Vrf_1)
    pub_vel_steering_wheel_right_2_front_wheel.publish(Vrf_2)
    pub_vel_steering_wheel_right_1_rear_wheel.publish(Vrb_1)
    pub_vel_steering_wheel_right_2_rear_wheel.publish(Vrb_2)


    # pub_pos_turntable_lf_steering_hinge.publish(wl)
    # pub_pos_turntable_lb_teering_hinge.publish(wr)
    # pub_pos_turntable_rf_steering_hinge.publish(wl)
    # pub_pos_turntable_rb_teering_hinge.publish(wr)


def calculate_steering_wheel_joint_posotion_steering(data):
    status = data
    s=data.pose

    for i in range(0,len(s)):
        if (status.name[i] == "ehang::turntable_lf_Link" ):
            lf_x   = s[i].orientation.x
            lf_y   = s[i].orientation.y
            lf_z   = s[i].orientation.z
            lf_w  = s[i].orientation.w
            global lf_angle 
            lf_angle = math.atan2(2 * (lf_w * lf_z + lf_x * lf_y), 1 - 2 * (lf_y**2 + lf_z**2))
            pub_debug_lf = rospy.Publisher('/debug/feedback_lf_angle', Float64, queue_size=1)
            pub_debug_lf.publish(lf_angle)

        if (status.name[i] == "ehang::turntable_lb_Link" ):
            lb_x   = s[i].orientation.x
            lb_y   = s[i].orientation.y
            lb_z   = s[i].orientation.z
            lb_w  = s[i].orientation.w
            global lb_angle 
            lb_angle = math.atan2(2 * (lb_w * lb_z + lb_x * lb_y), 1 - 2 * (lb_y**2 + lb_z**2))
            pub_debug_lb = rospy.Publisher('/debug/feedback_lb_angle', Float64, queue_size=1)
            pub_debug_lb.publish(lb_angle)

        if (status.name[i] == "ehang::turntable_rf_Link" ):
            rf_x   = s[i].orientation.x
            rf_y   = s[i].orientation.y
            rf_z   = s[i].orientation.z
            rf_w  = s[i].orientation.w
            global rf_angle 
            rf_angle = math.atan2(2 * (rf_w * rf_z + rf_x * rf_y), 1 - 2 * (rf_y**2 + rf_z**2))
            pub_debug_rf = rospy.Publisher('/debug/feedback_rf_angle', Float64, queue_size=1)
            pub_debug_rf.publish(rf_angle)

        if (status.name[i] == "ehang::turntable_rb_Link" ):
            rb_x   = s[i].orientation.x
            rb_y   = s[i].orientation.y
            rb_z   = s[i].orientation.z
            rb_w  = s[i].orientation.w
            global rb_angle 
            rb_angle = math.atan2(2 * (rb_w * rb_z + rb_x * rb_y), 1 - 2 * (rb_y**2 + rb_z**2))
            pub_debug_rb = rospy.Publisher('/debug/feedback_rb_angle', Float64, queue_size=1)
            pub_debug_rb.publish(rb_angle)

        if (status.name[i] == "ehang::dummy" ):
            base_x   = s[i].orientation.x
            base_y   = s[i].orientation.y
            base_z   = s[i].orientation.z
            base_w  = s[i].orientation.w
            global base_angle 
            base_angle = math.atan2(2 * (base_w * base_z + base_x * base_y), 1 - 2 * (base_y**2 + base_z**2))
            pub_debug_base = rospy.Publisher('/debug/feedback_base_angle', Float64, queue_size=1)
            pub_debug_base.publish(base_angle)


#   if(len(s) == 14): 
#      lf_x   = s[2].orientation.x
#      lf_y   = s[2].orientation.y
#      lf_z   = s[2].orientation.z
#      lf_w  = s[2].orientation.w

#      lb_x   = s[5].orientation.x
#      lb_y   = s[5].orientation.y
#      lb_z   = s[5].orientation.z
#      lb_w  = s[5].orientation.w        

#      rf_x   = s[8].orientation.x
#      rf_y   = s[8].orientation.y
#      rf_z   = s[8].orientation.z
#      rf_w  = s[8].orientation.w

#      rb_x   = s[11].orientation.x
#      rb_y   = s[11].orientation.y
#      rb_z   = s[11].orientation.z
#      rb_w  = s[11].orientation.w 

#      lf_angle[0] = math.atan2(2 * (lf_w * lf_x + lf_y * lf_z), 1 - 2 * (lf_x**2 + lf_y**2))
#      lf_angle[1] = math.asin(2 * (lf_w * lf_y - lf_z * lf_x))
#      lf_angle[2] = math.atan2(2 * (lf_w * lf_z + lf_x * lf_y), 1 - 2 * (lf_y**2 + lf_z**2))
     
#     #  lb_angle[0] = math.atan2(2 * (lb_w * lb_x + lb_y * lb_z), 1 - 2 * (lb_x**2 + lb_y**2))
#     #  lb_angle[1] = math.asin(2 * (lb_w * lb_y - lb_z * lb_x))
#     #  lb_angle[2] = math.atan2(2 * (lb_w * lb_z + lb_x * lb_y), 1 - 2 * (lb_y**2 + lb_z**2))

#     #  rf_angle[0] = math.atan2(2 * (rf_w * rf_x + rf_y * rf_z), 1 - 2 * (rf_x**2 + rf_y**2))
#     #  rf_angle[1] = math.asin(2 * (rf_w * rf_y - rf_z * rf_x))
#     #  rf_angle[2] = math.atan2(2 * (rf_w * rf_z + rf_x * rf_y), 1 - 2 * (rf_y**2 + rf_z**2))
     
#     #  rb_angle[0] = math.atan2(2 * (rb_w * rb_x + rb_y * rb_z), 1 - 2 * (rb_x**2 + rb_y**2))
#     #  rb_angle[1] = math.asin(2 * (rb_w * rb_y - rb_z * rb_x))
#     #  rb_angle[2] = math.atan2(2 * (rb_w * rb_z + rb_x * rb_y), 1 - 2 * (rb_y**2 + rb_z**2))


#     #  global lf_angle 
#     #  global lb_angle 
#     #  global rf_angle 
#     #  global rb_angle

#     #  lf_angle = math.atan2(2 * (lf_w * lf_z + lf_x * lf_y), 1 - 2 * (lf_y**2 + lf_z**2))
#     #  lb_angle =math.atan2(2 * (lb_w * lb_z + lb_x * lb_y), 1 - 2 * (lb_y**2 + lb_z**2))
#     #  rf_angle = math.atan2(2 * (rf_w * rf_z + rf_x * rf_y), 1 - 2 * (rf_y**2 + rf_z**2))
#     #  rb_angle = math.atan2(2 * (rb_w * rb_z + rb_x * rb_y), 1 - 2 * (rb_y**2 + rb_z**2))


#     #  rospy.loginfo("lf_angle:=%f ",  lf_angle)
#     #  rospy.loginfo("lb_angle :=%f ",  lb_angle )
#     #  rospy.loginfo("rf_angle:=%f ",  rf_angle)
#     #  rospy.loginfo("rb_angle:=%f ",  rb_angle)
     
#     #  pub_debug_lf = rospy.Publisher('/debug/feedback_lf_angle', Float64, queue_size=1)
#      pub_debug_lf0 = rospy.Publisher('/debug/feedback_lf_angle0', Float64, queue_size=1)
#      pub_debug_lf1 = rospy.Publisher('/debug/feedback_lf_angle1', Float64, queue_size=1)
#      pub_debug_lf2 = rospy.Publisher('/debug/feedback_lf_angle2', Float64, queue_size=1)


#     #  pub_debug_lb = rospy.Publisher('/debug/feedback_lb_angle', Float64, queue_size=1)
#     #  pub_debug_rf = rospy.Publisher('/debug/feedback_rf_angle', Float64, queue_size=1)
#     #  pub_debug_rb = rospy.Publisher('/debug/feedback_rb_angle', Float64, queue_size=1)

#      pub_debug_lf0.publish(lf_angle[0])
#      pub_debug_lf1.publish(lf_angle[1])
#      pub_debug_lf2.publish(lf_angle[2])
#     #  pub_debug_lb.publish(lb_angle)
#     #  pub_debug_rf.publish(rf_angle)
#     #  pub_debug_rb.publish(rb_angle)

def Sub_cmd_vel_steering_wheel():

    rospy.init_node('Sub_cmd_vel_steering_wheel', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, set_steering_wheel_velocity, queue_size=1,buff_size=52428800)

    rospy.Subscriber("/gazebo/link_states", LinkStates, calculate_steering_wheel_joint_posotion_steering,queue_size=1,buff_size=52428800)
    # rospy.Subscriber("/gazebo/link_states", LinkStates, calculate_steering_wheel_link_velocity_steering)


    rospy.Subscriber("/imu", Imu, imu_calc, queue_size=1,buff_size=52428800)

    # rospy.ERROR("test")

    # rospy.Subscriber("/debug/target_angle", Float64, get_target_angle, queue_size=1,buff_size=52428800)
    # rospy.Subscriber("/debug/kp", Float64, get_kp, queue_size=1,buff_size=52428800)


    # pub_vel_steering_wheel_left_1_front_wheel = rospy.Publisher('/ehang/steering_wheel_left_front_1_wheel_velocity_controller/command', Float64, queue_size=1)
    # pub_vel_steering_wheel_left_2_front_wheel = rospy.Publisher('/ehang/steering_wheel_left_front_2_wheel_velocity_controller/command', Float64, queue_size=1)
    # pub_vel_steering_wheel_left_1_front_wheel.publish(0.1)
    # pub_vel_steering_wheel_left_2_front_wheel.publish(-0.1)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        Sub_cmd_vel_steering_wheel()
    except rospy.ROSInterruptException:
        pass