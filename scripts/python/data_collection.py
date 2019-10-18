import numpy as np
import rospy
import yaml
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped
import tf
import std_msgs

def event_in_cb(msg):
    global event_in
    event_in = msg.data

def cc_command_cb(msg):
    global event_in
    global cc_commands
    if event_in == "start":
        command = [msg.header.stamp.to_sec(), msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                   msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
        cc_commands.append(command)

def wrench_cb(msg):
    global event_in
    global ft
    if event_in == "start":    
        readings = [rospy.Time.now().to_sec(), msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        ft.append(readings)

def tcp_wrench_cb(msg):
    global event_in
    global ft_tcp
    if event_in == "start":    
        readings = [rospy.Time.now().to_sec(), msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        ft_tcp.append(readings)

def cc_pose_cb(msg):
    global door_trajectory
    door_trajectory.append(msg.header.stamp.to_sec(),
                           msg.pose.position.x,
                           msg.pose.position.y,
                           msg.pose.position.z,
                           msg.pose.orientation.x,
                           msg.pose.orientation.y,
                           msg.pose.orientation.z,
                           msg.pose.orientation.w)

def policy_param_cb(msg):
    global policy_parameters
    policy_parameters.append(msg.data)

def new_policy_param_cb(msg):
    global new_policy_parameters
    new_policy_parameters.append(msg.data)

def reward_cb(msg):
    global reward_l
    reward_l.append(msg.data)

def avg_reward_cb(msg):
    global avg_reward_l
    avg_reward_l.append(msg.data)

""" data """
cc_commands = []
ft = []
ft_tcp = []
door_trajectory = []
policy_parameters = []
new_policy_parameters = []
reward_l = []
avg_reward_l = []
# wrench_topic_ = "/ft_sensor/ft_compensated"
tcp_wrench_topic_ = "/wrench"
tcp_pose_topic_ = "/cartesian_pose"
tcp_twist_topic_ = "/cartesian_twist"
cc_command_topic_ = "/mcr_manipulation/mcr_arm_cartesian_control/cartesian_velocity_command"
reward_topic = "/reward"
avg_reward_topic = "/reward_avg"
policy_param_topic = "/policy_parameters"
new_policy_param_topic = "/new_policy_parameters"
event_in_topic_ = "/data_collection/event_in"
base_link_ = "base_link"
# right_arm_tcp_ = "right_hand_static_grasp_link"
rospy.init_node("experimental_data_collection_node")
rospy.loginfo("Node Initialized")
tf_listener_ = tf.TransformListener()
event_in = "stop"
r = rospy.Rate(200)

rospy.Subscriber(event_in_topic_, std_msgs.msg.String, event_in_cb)
# rospy.Subscriber(tcp_twist_topic_, TwistStamped, cc_command_cb)
# rospy.Subscriber(tcp_pose_topic_, PoseStamped, cc_pose_cb)

rospy.Subscriber(policy_param_topic, std_msgs.msg.Float64MultiArray, policy_param_cb)
rospy.Subscriber(new_policy_param_topic, std_msgs.msg.Float64MultiArray, new_policy_param_cb)
rospy.Subscriber(reward_topic, std_msgs.msg.Float64, reward_cb)
rospy.Subscriber(avg_reward_topic, std_msgs.msg.Float64, avg_reward_cb)

# rospy.Subscriber(wrench_topic_, WrenchStamped, wrench_cb)
rospy.Subscriber(tcp_wrench_topic_, WrenchStamped, tcp_wrench_cb)



while not rospy.is_shutdown() and event_in != 'e_done':
#     while not rospy.is_shutdown():
#         try:
#             (trans,rot) = tf_listener_.lookupTransform(base_link_, right_arm_tcp_, rospy.Time(0))
#             break
#         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             r.sleep()
#             continue
#     if event_in == "start":    
#         door_trajectory.append([[rospy.Time.now().to_sec()], trans, rot])
    r.sleep()


data = {'policy_parameters': np.asarray(policy_parameters).tolist(),
        'new_policy_parameters': np.asarray(new_policy_parameters).tolist(),
        'reward': np.asarray(reward_l).tolist(),
        'avg_reward': np.asarray(avg_reward_l).tolist()}
with open(str(rospy.Time.now().to_sec())+ "_0809_" + "exp.yaml", "w") as f:
    yaml.dump(data, f)
