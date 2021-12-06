#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import os
import json
import numpy as np
import random
import time
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from collections import deque
from std_msgs.msg import Float32MultiArray
from src.turtlebot3_dqn.environment_stage_1 import Env
from tensorflow.keras.models import Sequential, load_model
from tensorflow.keras.optimizers import RMSprop
from tensorflow.keras.layers import Flatten, Dense, Dropout, Activation, Conv2D, MaxPooling2D
import matplotlib.pyplot as plt

EPISODES = 3000

class ReinforceAgent():
    def __init__(self, state_size, action_size):
        self.pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
        self.dirPath = os.path.dirname(os.path.realpath(__file__))
        self.dirPath = self.dirPath.replace('turtlebot3_dqn/nodes', 'turtlebot3_dqn/save_model/stage_1_')
        self.result = Float32MultiArray()

        self.load_model = False
        self.load_episode = 0
        self.state_size = state_size
        self.action_size = action_size
        self.episode_step = 6000
        self.target_update = 2000
        self.discount_factor = 0.99
        self.learning_rate = 0.00025
        self.epsilon = 1.0
        self.epsilon_decay = 0.99
        self.epsilon_min = 0.05
        self.batch_size = 64
        self.train_start = 64
        self.memory = deque(maxlen=1000000)

        self.model = self.buildModel()
        self.target_model = self.buildModel()

        self.updateTargetModel()

        if self.load_model:
            self.model.set_weights(load_model(self.dirPath+str(self.load_episode)+".h5").get_weights())

            with open(self.dirPath+str(self.load_episode)+'.json') as outfile:
                param = json.load(outfile)
                self.epsilon = param.get('epsilon')

        # Config to input space state image to CNN
        self.grid_num = state_size
        self.range_max = 3.5
        self.resolution = 2*self.range_max / self.grid_num
        self.angle_min = 0.0
        self.angle_max = 6.28318977356
        self.angle_increment = 0.0175019223243

    def real2grid_index(self, x, y, w, h, resolution):
        return int(np.floor(x/resolution + w/2)), int(np.floor(y/resolution + h/2))

    def obstacle_map(self, scan_range_list, yaw, pos):
        obs_map = np.zeros([self.grid_num, self.grid_num])
        laser_angle = self.angle_min

        for i in range(len(scan_range_list)):
            x_idx, y_idx = self.real2grid_index(
                                scan_range_list[i] * np.cos(laser_angle + yaw) + pos.x,
                                scan_range_list[i] * np.sin(laser_angle + yaw) + pos.y,
                                self.grid_num, self.grid_num, self.resolution
                                )
            if x_idx > (self.grid_num - 1):
                x_idx = (self.grid_num - 1)
            if y_idx > (self.grid_num - 1):
                y_idx = (self.grid_num - 1)
            obs_map[(self.grid_num - 1) - y_idx][x_idx] = 1
            robot_pos = self.real2grid_index(pos.x, pos.y,
                                self.grid_num, self.grid_num, self.resolution)
            obs_map[(self.grid_num - 1) - robot_pos[1]][robot_pos[0]] = 10
            laser_angle += self.angle_increment
        return obs_map

    def simple_plot(self, scan_range_list, yaw, pos):
        laser_angle = self.angle_min
        plot = np.zeros([len(scan_range_list), 2])
        for i in range(len(scan_range_list)):
            x = scan_range_list[i] * np.cos(laser_angle + yaw) + pos.x
            y = scan_range_list[i] * np.sin(laser_angle + yaw) + pos.y
            laser_angle += self.angle_increment
            plot[i,0] = x
            plot[i,1] = y
        return plot

    def buildModel(self):
        model = Sequential()
        dropout = 0.2
        model.add(Conv2D(filters=32, kernel_size=(3,3), strides=1, padding='same', activation="relu", input_shape=(self.state_size, self.state_size, 1)))
        model.add(MaxPooling2D(2,2))
        model.add(Flatten())

        model.add(Dense(32, activation='relu', kernel_initializer='lecun_uniform'))
        model.add(Dropout(dropout))

        model.add(Dense(self.action_size, kernel_initializer='lecun_uniform'))
        model.add(Activation('linear'))
        model.compile(loss='mse', optimizer=RMSprop(lr=self.learning_rate, rho=0.9, epsilon=1e-06))
        model.summary()

        return model

    def getQvalue(self, reward, next_target, done):
        if done:
            return reward
        else:
            return reward + self.discount_factor * np.amax(next_target)

    def updateTargetModel(self):
        self.target_model.set_weights(self.model.get_weights())

    def getAction(self, state):
        if np.random.rand() <= self.epsilon:
            self.q_value = np.zeros(self.action_size)
            return random.randrange(self.action_size)
        else:
            state = self.obstacle_map(state[:-2])
            q_value = self.model.predict(state.reshape(1, self.state_size, self.state_size, 1))
            self.q_value = q_value
            return np.argmax(q_value[0])

    def appendMemory(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def trainModel(self, target=False):
        mini_batch = random.sample(self.memory, self.batch_size)
        X_batch = np.empty((0, self.state_size, self.state_size), dtype=np.float64)
        Y_batch = np.empty((0, self.action_size), dtype=np.float64)

        for i in range(self.batch_size):
            states = mini_batch[i][0]
            actions = mini_batch[i][1]
            rewards = mini_batch[i][2]
            next_states = mini_batch[i][3]
            dones = mini_batch[i][4]

            states = self.obstacle_map(states[:-2],env.yaw,env.position)
            next_states = self.obstacle_map(next_states[:-2],env.yaw,env.position)

            q_value = self.model.predict(states.reshape(1, self.state_size, self.state_size, 1))
            self.q_value = q_value

            if target:
                next_target = self.target_model.predict(next_states.reshape(1, self.state_size, self.state_size, 1))

            else:
                next_target = self.model.predict(next_states.reshape(1, self.state_size, self.state_size, 1))

            next_q_value = self.getQvalue(rewards, next_target, dones)

            X_batch = np.r_[X_batch, states.reshape(1, self.state_size, self.state_size).copy()]
            Y_sample = q_value.copy()

            Y_sample[0][actions] = next_q_value
            Y_batch = np.append(Y_batch, np.array([Y_sample[0]]), axis=0)

            if dones:
                X_batch = np.r_[X_batch, next_states.reshape(1, self.state_size, self.state_size).copy()]
                Y_batch = np.append(Y_batch, np.array([[rewards] * self.action_size]), axis=0)

        self.model.fit(X_batch.reshape(-1, self.state_size, self.state_size, 1),
                       Y_batch,
                       batch_size=self.batch_size, epochs=1, verbose=0)

def np_queue(src, add_data):
    dst = np.roll(src, -1, axis=0)
    if len(dst.shape) > 1:
        dst[-1, :] = add_data
    else:
        dst[-1] = add_data
    return dst

if __name__ == '__main__':
    rospy.init_node('turtlebot3_dqn_stage_1')
    pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
    pub_get_action = rospy.Publisher('get_action', Float32MultiArray, queue_size=5)
    result = Float32MultiArray()
    get_action = Float32MultiArray()

    # show pattern
    # grid_map, grid_pile_map, plot_map
    show_mode = "grid_pile_map"

    state_size = 80
    action_size = 5

    env = Env(action_size)

    agent = ReinforceAgent(state_size, action_size)
    scores, episodes = [], []
    global_step = 0
    start_time = time.time()

    fig, ax = plt.subplots(1, 1)
    ax.set_aspect('equal')

    map_layer_num = 5
    map_queue = np.zeros((map_layer_num, state_size, state_size))

    for e in range(agent.load_episode + 1, EPISODES):
        done = False
        state = env.reset()
        score = 0
        for t in range(agent.episode_step):

            # ax.cla()
            if show_mode == "grid_map":
                grid_obs_map = agent.obstacle_map(state[:-2], env.yaw, env.position)
                ax.imshow(grid_obs_map)
            elif show_mode == "grid_pile_map":
                grid_obs_map = agent.obstacle_map(state[:-2], env.yaw, env.position)
                map_queue = np_queue(src=map_queue, add_data=grid_obs_map)
                grid_pile_map = np.sum(map_queue, axis=0)
                ax.imshow(grid_pile_map)
            elif show_mode == "plot_map":
                plot_map = agent.simple_plot(state[:-2], env.yaw, env.position)
                vis_rot = np.pi/2
                x_pos = plot_map[:,0]*np.cos(vis_rot) - plot_map[:,1]*np.sin(vis_rot)
                y_pos = plot_map[:,0]*np.sin(vis_rot) + plot_map[:,1]*np.cos(vis_rot)
                ax.scatter(x_pos, y_pos, s=0.5, color="blue")
                ax.scatter(env.position.x*np.cos(vis_rot) - env.position.y*np.sin(vis_rot),
                           env.position.x*np.sin(vis_rot) + env.position.y*np.cos(vis_rot),
                           color="red", s=1)
            else:
                print("Please choice the show_mode !")
                sys.exit(-1)

            plt.pause(0.2)

            action = agent.getAction(state)

            next_state, reward, done = env.step(action)

            agent.appendMemory(state, action, reward, next_state, done)

            if len(agent.memory) >= agent.train_start:
                if global_step <= agent.target_update:
                    agent.trainModel()
                else:
                    agent.trainModel(True)

            score += reward
            state = next_state
            get_action.data = [action, score, reward]
            pub_get_action.publish(get_action)

            if e % 10 == 0:
                agent.model.save(agent.dirPath + str(e) + '.h5')
                with open(agent.dirPath + str(e) + '.json', 'w') as outfile:
                    json.dump(param_dictionary, outfile)

            if t >= 500:
                rospy.loginfo("Time out!!")
                done = True

            if done:
                result.data = [score, np.max(agent.q_value)]
                pub_result.publish(result)
                agent.updateTargetModel()
                scores.append(score)
                episodes.append(e)
                m, s = divmod(int(time.time() - start_time), 60)
                h, m = divmod(m, 60)

                rospy.loginfo('Ep: %d score: %.2f memory: %d epsilon: %.2f time: %d:%02d:%02d',
                              e, score, len(agent.memory), agent.epsilon, h, m, s)
                param_keys = ['epsilon']
                param_values = [agent.epsilon]
                param_dictionary = dict(zip(param_keys, param_values))
                break

            global_step += 1
            if global_step % agent.target_update == 0:
                rospy.loginfo("UPDATE TARGET NETWORK")

        if agent.epsilon > agent.epsilon_min:
            agent.epsilon *= agent.epsilon_decay
