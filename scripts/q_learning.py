#!/usr/bin/env python3

import rospy
import numpy as np
import os
import random
import pandas as pd
import time
from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveObjectToTag
from q_learning_project.msg import QMatrix
# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")
        print("beginning of init")
        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # publish the reward you get from move object to tag

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))


        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))
        # num rows in Q-matrix = number of states (rows of states matrix)
        self.state_num = len(self.states)
        # num cols in Q-matrix = number of actions (rows of actions matrix --> 9)
        self.action_num = len(self.actions)
        # initializing empty q_matrix that is 's_t by a_t'
        # fill it all with 0s since we don't know anything about the initial state of the matrix
        self.m = QMatrix()
        self.m.q_matrix = np.zeros((self.state_num, self.action_num), dtype = float)
        # define the decay factor
        self.decay = 0.5
        # publish the q_matrix
        self.q_matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)
        # publish the robot action
        self.pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)
        # subscribe to the rewards ROS topic
        self.current_reward = QLearningReward()
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.reward_callback_handler)
        # print("init done")
        time.sleep(1)

    def reward_callback_handler(self, data):
        # callback function to collect reward from reward ROS topic after action is published
        self.current_reward = data

    def valid_actions_given_curr_state(self, s):
        # loop through action_matrix
        # given current state, return all the valid actions (aka the value of the cell)
        # and also return the next state you will end up in (aka the column number of the action)
        row_to_loop_through = self.action_matrix[s]
        # create array to fil with tuples of valid actions and corresponding column number
        valid_actions = []
        index = 0
        for a in row_to_loop_through:
            if a != -1:
               # create tuple
               new_entry = (a, index)
               # append tuple to valid_actions
               valid_actions.append(new_entry)
            index +=1
        return valid_actions

    def update_q_matrix(self):
        # convergence criteria: create a flag that becomes 'False'  every time q_matrix is updated
        # once q_matrix has converged, then it will not be updated and converged flag will stay 'True'
        # create a counter to count number of iterations we've gone through q_matrix to update it.
        # Force Q-matrix to undergo at least 1000 iterations before fully considering if it's converged.
        iterations = 0
        curr_state = 0
        converged = False
        while (True):
            converged = True
            list_of_valid_actions = self.valid_actions_given_curr_state(curr_state)
            if list_of_valid_actions == []:
                curr_state = 0
                list_of_valid_actions = self.valid_actions_given_curr_state(curr_state)
            # select a random value from 0 to the number of valid action rows
            rand_action = random.choices(list_of_valid_actions)
            # using random value, find corresponding random action & rand next state
            # rand_action = list_of_valid_actions[rand_value]
            rand_action_id = int(rand_action[0][0])
            rand_next_state = rand_action[0][1]
            # extract action from action matrix using the rand_action_row
            action = self.actions[int(rand_action_id)]
            action_object = action["object"]
            action_tag = action["tag"]
            # construct the RobotMoveObjectToTag object to publish
            action_to_publish = RobotMoveObjectToTag()
            action_to_publish.robot_object = action_object
            action_to_publish.tag_id = action_tag
            # publish action
            self.pub.publish(action_to_publish)
            time.sleep(.01)
            # find the q_matrix max val for the next state after the action
            max_next_state_q_val = max(self.m.q_matrix[rand_next_state])
            # find the cell in the q_matrix with corresponding state and valid action
            # and update that cell. We leave alpha as 1.
            initial_q_matrix_cell_val = self.m.q_matrix[curr_state][rand_action_id]
            self.m.q_matrix[curr_state][rand_action_id] = self.current_reward.reward + self.decay*max_next_state_q_val
            self.q_matrix_pub.publish(self.m)
            time.sleep(.01)
            # check if the updated cell has the same value as the original. If the
            # cell value has changed, then the q_matrix has not converged yet...
            if initial_q_matrix_cell_val != self.m.q_matrix[curr_state][rand_action_id]:
                converged = False
            # print("curr_state", curr_state)
            curr_state = rand_next_state
            # print("iterations: ", iterations)
            iterations += 1
            # check if no cells have been updated by the 10th iteration
            if iterations >= 1000 and converged:
                # print message and break out of while loop if q_matrix is converged
                print("q_matrix converged!")
                break
        return

    def save_q_matrix(self):
        # use python built-in string to csv converter to convert matrix to csv file! :)
        pd.DataFrame(self.m.q_matrix).to_csv('converged_q_matrix.csv')
        return

if __name__ == "__main__":
    node = QLearning()
    # call functions from ROS node to run
    node.update_q_matrix()
    node.save_q_matrix()
    rospy.spin()
