#!/usr/bin/env python3

import rospy
import numpy as np
import os
import random
import pandas as pd

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

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

        # initializing empty q_matrix that is 's_t by a_t'
        # fill it all with 0s since we don't know anything about the initial state of the matrix
        # num rows in Q-matrix = number of states (rows of states matrix)
        states = self.states.shape[0]
        # num cols in Q-matrix = number of actions (rows of actions matrix --> 9)
        actions = self.actions.shape[0]
        # now initialize and return Q-matrix
        # q_matrix =  [[0 for i in range(states)] for j in range(actions)]
        self.q_matrix = np.zeros((states, actions), dtype = int)

        # define the decay factor
        self.decay = 0.8

        # set topic name
        # self.reward_topic = "reward"
        # subscribe to the rewards ROS topic
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.update_q_matrix)

#    def init_starting_q_matrix(self):

    def update_q_matrix(self, data):
        # convergence criteria: create a flag that becomes 'False'  every time q_matrix is updated
        # once q_matrix has converged, then it will not be updated and converged flag will stay 'True'

        # create a counter to count number of iterations we've gone through q_matrix to update it
        iterations = 0
        while (True):
            converged = True
            # iterate through each state
            for s in range(0, states):
                # select random value from 0 .. num_rows to end up in a random next state
                # make sure the action taken to get to that next state is valid (cell val != -1)
                while(True):
                    rand_next_state = random.randint(0, states)
                    if self.action_matrix[s][rand_next_state] != -1:
                        # save the valid action
                        action = self.action_matrix[s][rand_next_state]
                        # break out of the regeneration loop
                        break
                # extract the reward from the ROS topic
                reward = data
                # find the q_matrix max val for the next state after the action
                max_next_state_q_val = max(q_matrix[rand_next_state])
                # find the cell in the q_matrix with corresponding state and valid action
                # and update that cell. We leave alpha as 1.
                initial_q_matrix_cell_val = self.q_matrix[s][action]
                self.q_matrix[s][action] = reward + self.decay*max_next_state_q_val
                # check if the updated cell has the same value as the original. If the
                # cell value has changed, then the q_matrix has not converged yet...
                if initial_q_matrix_cell_val != self.q_matrix[s][action]:
                    converged = False
            # update number of times we've updated entire q_matrix
            iterations += 1
            # check if no cells have been updated by the 10th iteration
            if iterations >= 10 and converged:
                # print message and break out of while loop if q_matrix is converged
                print("q_matrix converged!")
                break

    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        # use python built-in string to csv converter to convert matrix to csv file! :)
        pd.DataFrame(self.q_matrix).to_csv('converged_q_matrix.csv')
        return

if __name__ == "__main__":
    node = QLearning()

