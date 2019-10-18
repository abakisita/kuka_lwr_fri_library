#!/usr/bin/env python2

import rospy

import numpy as np
import matplotlib.pyplot as plt
import std_msgs.msg

class policy():
    
    def __init__(self):
        
        self.A = 0.5
        self.B = 0.5
        self.C = 1
        self.D = 0.0
        self.var = 0.55
        self.lr = 0.005
        
    
    def action(self, state):
        
        f =  self.C * (1 - state[2]) + self.B * (0.5 ** 2 - (0.5 - state[1])**2) \
            - self.A * state[0] 
        
        return np.random.normal(f, self.var), f
        
    
    def log_prob_grad(self, s):
        c = -((s[0] - s[1])*np.exp(-(s[0]-s[1])**2/(2 * self.var**2)))/(np.sqrt(2*np.pi)*self.var**2)
        
        grad = np.array([[-s[2]],
                         [(0.25 - (0.5 - s[3])**2)],
                         [(1 - s[4])]])
        
        grad = grad * c 
        if np.linalg.norm(grad) > 1000:
            print (s)
        return grad
        
    def update(self, grad):
        
        self.A += grad[0]
        self.B += grad[1]
        self.C += grad[2]       
      
rollout_data = None
done = False
episode_data = []

def calaculate_gradient(episode_data):

    reward_l = []
    psi_sum_l = []
    vg_l = []
    for r in episode_data:
        psi_sum = np.zeros((3,1))
        reward = 0.0
        for temp in range(r.shape[0]):
            row = r[temp, :]
            psi_sum = psi_sum + p.log_prob_grad(row) 
            reward += (discount ** temp) * row[5]
        reward_l.append(reward)
        psi_sum_l.append(psi_sum)
        vg_l.append(psi_sum * reward)
    print reward
    fisher_matrix = np.zeros((3, 3))
    vg = np.zeros((3, 1))
    eligibilty = np.zeros((3, 1))
    average_r = 0.0
    for i in range(len(reward_l)):
        fisher_matrix += psi_sum_l[i].dot(psi_sum_l[i].T)
        eligibilty += psi_sum_l[i]
        vg += vg_l[i]
    average_r = np.sum(reward_l) / n_rl
    fisher_matrix = fisher_matrix / n_rl
    eligibilty = eligibilty / n_rl
    vg = vg / n_rl

    q = (1 + eligibilty.T.dot(np.linalg.pinv(n_rl*fisher_matrix - eligibilty.dot(eligibilty.T))).dot(eligibilty))/n_rl
    b = q[0,0] * (average_r - eligibilty.T.dot(np.linalg.pinv(fisher_matrix).dot(vg)))
    grad = np.linalg.pinv(fisher_matrix).dot((vg - eligibilty*b))
    return grad[:,0], average_r


def rollout_data_cb(msg):
    global rollout_data
    global done
    global rollout_data
    rows = msg.layout.dim[0].size
    cols = msg.layout.dim[1].size
    rollout_data = np.zeros((rows, cols))

    for i in range(rows):
        for j in range(cols):
            rollout_data[i, j] = msg.data[cols * i + j]
    print (rollout_data)
    print (i, j)
    episode_data.append(rollout_data)
    done = True

def calculate_reward(episode_data):
    reward = 0.0
    for r in episode_data:
        reward = 0.0
        for temp in range(r.shape[0]):
            row = r[temp, :]
            reward += (discount ** temp) * row[5]
    reward_pub.publish(std_msgs.msg.Float64(reward))


def calaculate_vanilla_gradient(episode_data):
        
    bar = np.zeros((3,1))
    b_sqr = np.zeros((3,1))
    baseline = np.zeros((3,1))
    reward_l = []
    bar_l = []
    b_l = []
    for r in episode_data:
        b = np.zeros((3,1))
        reward = 0.0
        for temp in range(r.shape[0]):
            row = r[temp, :]
            b = b + p.log_prob_grad(row) 
            reward += (discount ** temp) * row[5]
        reward_l.append(reward)
        b_l.append(b)
        bar += (b ** 2) * reward 
        b_sqr += (b ** 2)
    print reward_l
    
    if b_sqr[0] == 0.0 or b_sqr[1] == 0.0 or b_sqr[2] == 0.0:
        print ("invalid b")
    else:
        baseline[0] = bar[0] / b_sqr[0]
        baseline[1] = bar[1] / b_sqr[1]
        baseline[2] = bar[2] / b_sqr[2]
    grad0 = 0.0
    grad1 = 0.0
    grad2 = 0.0
    for n in range(len(reward_l)):
        grad0 += b_l[n][0,0]*(reward_l[n] - baseline[0,0])
        grad1 += b_l[n][1,0]*(reward_l[n] - baseline[1,0])
        grad2 += b_l[n][2,0]*(reward_l[n] - baseline[2,0])
    return np.array([grad0, grad1, grad2]), np.sum(reward_l) / n_rl

# print (episode_data)

def calculate_power_gradient(episode_data):
    reward_l = []
    for r in episode_data:
        reward = 0.0
        for temp in range(r.shape[0]):
            row = r[temp, :]
            reward += (discount ** temp) * row[5]
        reward_l.append(reward)
    print reward_l
    theta_n = np.array([p.A, p.B, p.C])
    
    trm = np.zeros(3)
    for i in range(len(reward_l)):
        trm = trm + (new_params[i] - theta_n)*reward_l[i]

    grad = trm/np.sum(reward_l)

    return grad, np.sum(reward_l)/n_rl




if __name__ == "__main__":

    rospy.init_node("reinforcemnt_learning_node")
    print ("initialized node")
    rospy.Subscriber("/episode_data", std_msgs.msg.Float64MultiArray, rollout_data_cb, queue_size=1)
    event_in_pub = rospy.Publisher("/event_in", std_msgs.msg.String, queue_size=1)
    policy_param_pub = rospy.Publisher("/policy_parameters", std_msgs.msg.Float64MultiArray, queue_size=1)
    new_policy_param_pub = rospy.Publisher("/new_policy_parameters", std_msgs.msg.Float64MultiArray, queue_size=1)
    reward_pub = rospy.Publisher("/reward", std_msgs.msg.Float64, queue_size=1)
    avg_reward_pub = rospy.Publisher("/reward_avg", std_msgs.msg.Float64, queue_size=1)
    data_collection_pub = rospy.Publisher("/data_collection/event_in", std_msgs.msg.String, queue_size=1)
    rospy.sleep(1.0)
    p = policy()
    n_e = 10
    n_rl = 12
    discount = 1
    step_size = 0.04
    final_reward = []
    p.lr = 0.005

    param_msg = std_msgs.msg.Float64MultiArray()
    param_msg.data = [p.A, p.B, p.C]
    policy_param_pub.publish(param_msg)
    rospy.sleep(1.0)



    for e in range(n_e):
        data = []
        episode_data = []
        command = raw_input("Press any key to start new episode")
        pp = np.array([p.A, p.B, p.C])
        new_params = np.random.normal(pp, 1.5, (n_rl,3))
        param_msg = std_msgs.msg.Float64MultiArray()
        param_msg.data = [p.A, p.B, p.C]
        policy_param_pub.publish(param_msg)
        new_policy_param_pub.publish(param_msg)

        # reference run
        done = False

        event_in_pub.publish(std_msgs.msg.String("e_start"))

        while (not done and not rospy.is_shutdown()):
            rospy.sleep(0.03)
        print "Done"
        # calculate and publish reward
        calculate_reward(episode_data)

        episode_data = []
        command = raw_input("Press any key to start new episode")

        for rl in range(n_rl):
            rl_data = []
            print ("Episode " + str(e) + " " + "Rollout " + str(rl))
            param_msg = std_msgs.msg.Float64MultiArray()
            param_msg.data = [new_params[rl ,0], new_params[rl ,1], new_params[rl,2]]
            policy_param_pub.publish(param_msg)
            previous_a = 0.0
            done = False
            event_in_pub.publish(std_msgs.msg.String("e_start"))
            while (not done and not rospy.is_shutdown()):
                rospy.sleep(0.03)
            print "Done"

        grad, average_reward = calculate_power_gradient(episode_data)
        print grad, average_reward
        final_reward.append(average_reward)
        p.update(grad)
        param_msg = std_msgs.msg.Float64MultiArray()
        param_msg.data = [p.A, p.B, p.C]
        policy_param_pub.publish(param_msg)
        avg_reward_pub.publish(std_msgs.msg.Float64(average_reward))
        rospy.sleep(1.0)
    data_collection_pub.publish(std_msgs.msg.String("e_done"))

# if __name__ == "__main__":

#     rospy.init_node("reinforcemnt_learning_node")
#     print ("initialized node")
#     rospy.Subscriber("/episode_data", std_msgs.msg.Float64MultiArray, rollout_data_cb, queue_size=1)
#     event_in_pub = rospy.Publisher("/event_in", std_msgs.msg.String, queue_size=1)
#     policy_param_pub = rospy.Publisher("/policy_parameters", std_msgs.msg.Float64MultiArray, queue_size=1)
#     reward_pub = rospy.Publisher("/reward", std_msgs.msg.Float64, queue_size=1)
#     data_collection_pub = rospy.Publisher("/data_collection/event_in", std_msgs.msg.String, queue_size=1)
#     rospy.sleep(1.0)
#     p = policy()
#     n_e = 10
#     n_rl = 10
#     discount = 1
#     step_size = 0.04
#     final_reward = []
#     p.lr = 0.005

#     param_msg = std_msgs.msg.Float64MultiArray()
#     param_msg.data = [p.A, p.B, p.C]
#     policy_param_pub.publish(param_msg)
#     rospy.sleep(1.0)

#     for e in range(n_e):
#         data = []
#         episode_data = []
#         command = raw_input("Press any key to start new episode")
#         for rl in range(n_rl):
#             print ("Episode " + str(e) + " " + "Rollout " + str(rl))
#             rl_data = []
#             previous_a = 0.0
#             done = False
#             event_in_pub.publish(std_msgs.msg.String("e_start"))
#             while (not done and not rospy.is_shutdown()):
#                 rospy.sleep(0.01)
#             print "Done"

#         grad, average_reward = calaculate_vanilla_gradient(episode_data)
#         print grad, average_reward
#         final_reward.append(average_reward)
#         p.update(grad)
#         param_msg = std_msgs.msg.Float64MultiArray()
#         param_msg.data = [p.A, p.B, p.C]
#         policy_param_pub.publish(param_msg)
#         reward_pub.publish(std_msgs.msg.Float64(average_reward))
#         rospy.sleep(1.0)
#     data_collection_pub.publish(std_msgs.msg.String("e_done"))