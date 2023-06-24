#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de
from cmath import pi
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

lf_vel = 0
lb_vel = 0
rf_vel = 0
rb_vel = 0

lf_direction = 0
lb_direction = 0
rf_direction = 0
rb_direction = 0

lf_Ang = 0
lb_Ang = 0
rf_Ang = 0
rb_Ang = 0

vx = 0
vy = 0

t = 0

def imu_calc(data):
    rf_x   = data.orientation.x
    rf_y   = data.orientation.y
    rf_z   = data.orientation.z
    rf_w  = data.orientation.w
    global body_angle 
    body_angle = math.atan2(2 * (rf_w * rf_z + rf_x * rf_y), 1 - 2 * (rf_y**2 + rf_z**2))
    pub_debug_body_angle = rospy.Publisher('/debug/feedback_body_angle', Float64, queue_size=1)
    pub_debug_body_angle.publish(body_angle)

    

def AngleLimit(angle):
    if(angle>pi):
        angle = angle - 2*pi
    elif angle < -pi:
        angle = angle + 2*pi
    return angle

def relative_AngleLimit(angle):
    if(angle>pi):
        angle = angle - 2*pi
    elif angle < -pi:
        angle = angle + 2*pi
    return angle

def lf_calcwheel(vel_x,vel_y,omega,angleN,postureAngle):
    velX = 0
    velY = 0
    velN = 0
    velNDirection = 0
    sumVelX = 0
    sumVelY = 0
    velX = vel_x
    velY = vel_y
    velN = omega * 0.1
    velNDirection = angleN + postureAngle
    velNDirection = AngleLimit(velNDirection)
    sumVelX = velX + velN * math.cos(velNDirection)
    sumVelY = velY + velN * math.sin(velNDirection)
    global lf_vel
    global lf_direction
    lf_vel = math.sqrt(sumVelX * sumVelX + sumVelY * sumVelY)
    lf_direction = math.atan2(sumVelY, sumVelX)

def lb_calcwheel(vel_x,vel_y,omega,angleN,postureAngle):
    velX = 0
    velY = 0
    velN = 0
    velNDirection = 0
    sumVelX = 0
    sumVelY = 0
    velX = vel_x
    velY = vel_y
    velN = omega *0.1
    velNDirection = angleN + postureAngle
    velNDirection = AngleLimit(velNDirection)
    sumVelX = velX + velN * math.cos(velNDirection)
    sumVelY = velY + velN * math.sin(velNDirection)
    global lb_vel
    global lb_direction
    lb_vel = math.sqrt(sumVelX * sumVelX + sumVelY * sumVelY)
    lb_direction = math.atan2(sumVelY, sumVelX)

def rf_calcwheel(vel_x,vel_y,omega,angleN,postureAngle):
    velX = 0
    velY = 0
    velN = 0
    velNDirection = 0
    sumVelX = 0
    sumVelY = 0
    velX = vel_x
    velY = vel_y
    velN = omega * 0.1
    velNDirection = angleN + postureAngle
    velNDirection = AngleLimit(velNDirection)
    sumVelX = velX + velN * math.cos(velNDirection)
    sumVelY = velY + velN * math.sin(velNDirection)
    global rf_vel
    global rf_direction
    rf_vel = math.sqrt(sumVelX * sumVelX + sumVelY * sumVelY)
    rf_direction = math.atan2(sumVelY, sumVelX)

def rb_calcwheel(vel_x,vel_y,omega,angleN,postureAngle):
    velX = 0
    velY = 0
    velN = 0
    velNDirection = 0
    sumVelX = 0
    sumVelY = 0
    velX = vel_x
    velY = vel_y
    velN = omega * 0.1
    velNDirection = angleN + postureAngle
    velNDirection = AngleLimit(velNDirection)
    sumVelX = velX + velN * math.cos(velNDirection)
    sumVelY = velY + velN * math.sin(velNDirection)
    global rb_vel
    global rb_direction
    rb_vel = math.sqrt(sumVelX * sumVelX + sumVelY * sumVelY)
    rb_direction = math.atan2(sumVelY, sumVelX)


def lf_JudgeVelDirection():
    global lf_vel
    global lf_direction
    global lf_Ang
    n = 0
    angleErr = 0
    n = int(lf_Ang) - int(0.5*lf_Ang)
    lf_direction = n*2*pi + lf_direction

    angleErr = lf_direction - lf_Ang
    angleErr = AngleLimit(angleErr)

    if(math.fabs(angleErr)>pi/2):
        lf_vel = -(lf_vel)
        lf_direction = lf_direction +pi
        if(lf_direction>pi):
            lf_direction = lf_direction - 2*pi
        elif lf_direction < -pi:
            lf_direction = lf_direction + 2*pi

def lb_JudgeVelDirection():
    global lb_vel
    global lb_direction
    global lb_Ang
    n = 0
    angleErr = 0
    n = int(lb_Ang) - int(0.5*lb_Ang)
    lb_direction = n*2*pi + lb_direction

    angleErr = lb_direction
    angleErr = AngleLimit(angleErr)

    if(math.fabs(angleErr)>pi/2):
        lb_vel = -(lb_vel)
        lb_direction = lb_direction +pi
        if(lb_direction>pi):
            lb_direction = lb_direction - 2*pi
        elif lb_direction < -pi:
            lb_direction = lb_direction + 2*pi

def rf_JudgeVelDirection():
    global rf_vel
    global rf_direction
    global rf_Ang
    n = 0
    angleErr = 0
    n = int(rf_Ang) - int(0.5*rf_Ang)
    rf_direction = n*2*pi + rf_direction

    angleErr = rf_direction
    angleErr = AngleLimit(angleErr)

    if(math.fabs(angleErr)>pi/2):
        rf_vel = -(rf_vel)
        rf_direction = rf_direction +pi
        if(rf_direction>pi):
            rf_direction = rf_direction - 2*pi
        elif rf_direction < -pi:
            rf_direction = rf_direction + 2*pi

def rb_JudgeVelDirection():
    global rb_vel
    global rb_direction
    global rb_Ang
    n = 0
    angleErr = 0
    n = int(rb_Ang) - int(0.5*rb_Ang)
    rb_direction = n*2*pi + rb_direction

    angleErr = rb_direction
    angleErr = AngleLimit(angleErr)

    if(math.fabs(angleErr)>pi/2):
        rb_vel = -(rb_vel)
        rb_direction = rb_direction +pi
        if(rb_direction>pi):
            rb_direction = rb_direction - 2*pi
        elif rb_direction < -pi:
            rb_direction = rb_direction + 2*pi


def lf_TurnInferiorArc():
    global lf_vel
    global lf_direction
    global lf_Ang

    if(lf_direction - lf_Ang >pi):
        return (lf_direction - 2*pi)
    elif lf_direction - lf_Ang < -pi:
        return (lf_direction + 2*pi)
    else :
        return lf_direction

def lb_TurnInferiorArc():
    global lb_vel
    global lb_direction
    global lb_Ang

    if(lb_direction - lb_Ang>pi):
        return (lb_direction - 2*pi)
    elif lb_direction -lb_Ang < -pi:
        return (lb_direction + 2*pi)
    else :
        return lb_direction

def rf_TurnInferiorArc():
    global rf_vel
    global rf_direction
    global rf_Ang

    if(rf_direction - rf_Ang>pi):
        return (rf_direction - 2*pi)
    elif rf_direction - rf_Ang < -pi:
        return (rf_direction + 2*pi)
    else :
        return rf_direction

def rb_TurnInferiorArc():
    global rb_vel
    global rb_direction
    global rb_Ang

    if(rb_direction - rb_Ang>pi):
        return (rb_direction - 2*pi)
    elif rb_direction - rb_Ang < -pi:
        return (rb_direction + 2*pi)
    else :
        return rb_direction


def Transform2RobotCoodinate():
    global body_angle 

    global  lf_direction
    global lb_direction
    global rf_direction
    global  rb_direction

    lf_direction =  lf_direction - body_angle
    lb_direction =  lb_direction - body_angle
    rf_direction =  rf_direction - body_angle
    rb_direction =  rb_direction - body_angle

    lf_direction = AngleLimit(lf_direction)
    lb_direction = AngleLimit(lb_direction)
    rf_direction = AngleLimit(rf_direction)
    rb_direction = AngleLimit(rb_direction)

def Transform2WheelCoodinate():
    global  lf_direction
    global lb_direction
    global rf_direction
    global  rb_direction

    lf_direction = lf_direction -1.57
    lb_direction = lb_direction -1.57
    rf_direction = rf_direction -1.57
    rb_direction = rb_direction -1.57

    lf_direction = AngleLimit(lf_direction)
    lb_direction = AngleLimit(lb_direction)
    rf_direction = AngleLimit(rf_direction)
    rb_direction = AngleLimit(rb_direction)
    
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

def Output_Wheel(vel_x, vel_y,omega):
    global body_angle
    postureAngle = body_angle
    lf_calcwheel(vel_x,vel_y,omega,0.78540,postureAngle)
    lb_calcwheel(vel_x,vel_y,omega,2.35619,postureAngle)
    rf_calcwheel(vel_x,vel_y,omega,-0.78540,postureAngle)
    rb_calcwheel(vel_x,vel_y,omega,-2.35619,postureAngle)

def Circular_trajectory():
    global vx
    global vy
    global t
    if(t <= 2):

        vx = 5*math.sin(t*pi)
        vy = 5*math.cos(t*pi)

        pub_debug_vx = rospy.Publisher('/debug/feedback_vx', Float64, queue_size=1)
        pub_debug_vx.publish(vx)

        Output_Wheel(vx,vy,0)

        t = t+0.01

    elif (t>2):
        t = 0
    
def set_steering_wheel_velocity(data):
    global lf_angle 
    global lb_angle 
    global rf_angle 
    global rb_angle
    global Position_KP 
    global Target_Angle
    global body_angle 

    global lf_vel
    global lb_vel
    global  rf_vel
    global  rb_vel

    global  lf_direction
    global lb_direction
    global rf_direction
    global  rb_direction

    global lf_Ang
    global lb_Ang
    global rf_Ang
    global rb_Ang

    global vx
    global vy

    # Circular_trajectory()

    Output_Wheel(data.linear.x,data.linear.y,data.angular.z)

    # Output_Wheel(5,5,0)

    Transform2RobotCoodinate()
    Transform2WheelCoodinate()
    
    lf_JudgeVelDirection()
    lb_JudgeVelDirection()
    rf_JudgeVelDirection()
    rb_JudgeVelDirection()

    lf_Ang = lf_TurnInferiorArc()
    lb_Ang = lb_TurnInferiorArc()
    rf_Ang = rf_TurnInferiorArc()
    rb_Ang = rb_TurnInferiorArc()

    Position_KP = 10
    # Target_Angle = data.angular.z
    # lf_direction = -0.785

    relative_lf_angle = lf_angle - body_angle
    relative_lb_angle = lb_angle - body_angle
    relative_rf_angle = rf_angle - body_angle
    relative_rb_angle = rb_angle - body_angle

    relative_lf_angle = AngleLimit(relative_lf_angle)
    relative_lb_angle = AngleLimit(relative_lb_angle)
    relative_rf_angle = AngleLimit(relative_rf_angle)
    relative_rb_angle = AngleLimit(relative_rb_angle)

    Bias_1 =  relative_lf_angle- lf_Ang
    Bias_2 =  relative_lb_angle- lb_Ang
    Bias_3 =  relative_rf_angle- rf_Ang
    Bias_4 =  relative_rb_angle- rb_Ang

    # pub_debug_lf_direction = rospy.Publisher('/debug/feedback_lf_direction', Float64, queue_size=1)
    # pub_debug_lf_direction.publish(lf_direction)

    Bias_1 = constrain_bias(Bias_1)
    Bias_2 = constrain_bias(Bias_2)
    Bias_3 = constrain_bias(Bias_3)
    Bias_4 = constrain_bias(Bias_4)

    pid_out_1 = Bias_1* Position_KP
    pid_out_2 = Bias_2 * Position_KP
    pid_out_3 = Bias_3 * Position_KP
    pid_out_4 = Bias_4 * Position_KP

    # pub_debug_pid_out_1 = rospy.Publisher('/debug/feedback_pid_out_1', Float64, queue_size=1)
    # pub_debug_pid_out_1.publish(pid_out_1)

    Vlf_1 = -(lf_vel + pid_out_1)
    Vlf_2 = -(lf_vel -  pid_out_1)
    Vlb_1= -(lb_vel  + pid_out_2)
    Vlb_2= -(lb_vel - pid_out_2)
    Vrf_1= -(rf_vel + pid_out_3)
    Vrf_2= -(rf_vel  - pid_out_3)
    Vrb_1= -(rb_vel + pid_out_4)
    Vrb_2= -(rb_vel  - pid_out_4)
    

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