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

    #def real2grid_index(self, x, y, w, h, resolution):
        #return int(np.floor(x/resolution + w/2)), int(np.floor(y/resolution + h/2))
    def real2grid_index(self, x, w, resolution):
        return (np.floor(x/resolution + w/2)).astype(int)
    def obstacle_map(self, scan_range_list, yaw, pos):
        obs_map = np.zeros([self.grid_num, self.grid_num])

        laser_angle = [0.0, 0.0175019223243, 0.0350038446486, 0.0525057669729, 0.0700076892972, 0.0875096116215, 0.1050115339458, 0.12251345627009999, 0.1400153785944, 0.1575173009187, 0.175019223243, 0.1925211455673, 0.2100230678916, 0.2275249902159, 0.24502691254019998, 0.2625288348645, 0.2800307571888, 0.2975326795131, 0.3150346018374, 0.3325365241617, 0.350038446486, 0.3675403688103, 0.3850422911346, 0.4025442134589, 0.4200461357832, 0.43754805810749997, 0.4550499804318, 0.4725519027561, 0.49005382508039996, 0.5075557474047, 0.525057669729, 0.5425595920533, 0.5600615143776, 0.5775634367019, 0.5950653590262, 0.6125672813505, 0.6300692036748, 0.6475711259990999, 0.6650730483234, 0.6825749706477, 0.700076892972, 0.7175788152963, 0.7350807376206, 0.7525826599449, 0.7700845822692, 0.7875865045935, 0.8050884269178, 0.8225903492421, 0.8400922715664, 0.8575941938907, 0.8750961162149999, 0.8925980385393, 0.9100999608636, 0.9276018831878999, 0.9451038055122, 0.9626057278365, 0.9801076501607999, 0.9976095724851, 1.0151114948094, 1.0326134171337, 1.050115339458, 1.0676172617822999, 1.0851191841066, 1.1026211064309, 1.1201230287552, 1.1376249510795, 1.1551268734038, 1.1726287957281, 1.1901307180524, 1.2076326403767, 1.225134562701, 1.2426364850253, 1.2601384073496, 1.2776403296739, 1.2951422519981999, 1.3126441743225, 1.3301460966468, 1.3476480189711, 1.3651499412954, 1.3826518636197, 1.400153785944, 1.4176557082683, 1.4351576305926, 1.4526595529169, 1.4701614752412, 1.4876633975655, 1.5051653198898, 1.5226672422140999, 1.5401691645384, 1.5576710868627, 1.575173009187, 1.5926749315113, 1.6101768538356, 1.6276787761598999, 1.6451806984842, 1.6626826208085, 1.6801845431328, 1.6976864654571, 1.7151883877814, 1.7326903101057, 1.7501922324299999, 1.7676941547543, 1.7851960770786, 1.8026979994029, 1.8201999217272, 1.8377018440515, 1.8552037663757999, 1.8727056887001, 1.8902076110244, 1.9077095333487, 1.925211455673, 1.9427133779973, 1.9602153003215999, 1.9777172226458999, 1.9952191449702, 2.0127210672945, 2.0302229896188, 2.0477249119431, 2.0652268342674, 2.0827287565917, 2.100230678916, 2.1177326012403, 2.1352345235645998, 2.1527364458889, 2.1702383682132, 2.1877402905375, 2.2052422128618, 2.2227441351861, 2.2402460575104, 2.2577479798346998, 2.275249902159, 2.2927518244833, 2.3102537468076, 2.3277556691319, 2.3452575914562, 2.3627595137805, 2.3802614361048, 2.3977633584291, 2.4152652807534, 2.4327672030777, 2.450269125402, 2.4677710477262997, 2.4852729700506, 2.5027748923749, 2.5202768146992, 2.5377787370235, 2.5552806593478, 2.5727825816721, 2.5902845039963998, 2.6077864263207, 2.625288348645, 2.6427902709693, 2.6602921932936, 2.6777941156179, 2.6952960379422, 2.7127979602664998, 2.7302998825908, 2.7478018049151, 2.7653037272394, 2.7828056495637, 2.800307571888, 2.8178094942123, 2.8353114165366, 2.8528133388609, 2.8703152611852, 2.8878171835095, 2.9053191058338, 2.9228210281580997, 2.9403229504824, 2.9578248728067, 2.975326795131, 2.9928287174553, 3.0103306397796, 3.0278325621039, 3.0453344844281998, 3.0628364067525, 3.0803383290768, 3.0978402514011, 3.1153421737254, 3.1328440960497, 3.150346018374, 3.1678479406983, 3.1853498630226, 3.2028517853469, 3.2203537076712, 3.2378556299955, 3.2553575523197997, 3.2728594746441, 3.2903613969684, 3.3078633192927, 3.325365241617, 3.3428671639413, 3.3603690862656, 3.3778710085898997, 3.3953729309142, 3.4128748532385, 3.4303767755628, 3.4478786978871, 3.4653806202114, 3.4828825425357, 3.5003844648599998, 3.5178863871843, 3.5353883095086, 3.5528902318329, 3.5703921541572, 3.5878940764814997, 3.6053959988058, 3.6228979211301, 3.6403998434544, 3.6579017657787, 3.675403688103, 3.6929056104273, 3.7104075327515997, 3.7279094550759, 3.7454113774002, 3.7629132997245, 3.7804152220488, 3.7979171443731, 3.8154190666974, 3.8329209890216998, 3.850422911346, 3.8679248336703, 3.8854267559946, 3.9029286783189, 3.9204306006431997, 3.9379325229675, 3.9554344452917998, 3.9729363676161, 3.9904382899404, 4.0079402122647, 4.025442134589, 4.0429440569133, 4.0604459792376, 4.0779479015619, 4.0954498238862, 4.1129517462105, 4.1304536685348, 4.1479555908591, 4.1654575131834, 4.1829594355076996, 4.200461357832, 4.2179632801563, 4.2354652024806, 4.2529671248049, 4.2704690471291995, 4.2879709694535, 4.3054728917778, 4.3229748141021, 4.3404767364264, 4.3579786587507, 4.375480581075, 4.3929825033993, 4.4104844257236, 4.4279863480479, 4.4454882703722, 4.4629901926965, 4.4804921150208, 4.4979940373451, 4.5154959596693995, 4.5329978819937, 4.550499804318, 4.5680017266423, 4.5855036489666, 4.6030055712908995, 4.6205074936152, 4.6380094159395, 4.6555113382638, 4.6730132605881, 4.6905151829124, 4.7080171052367, 4.725519027561, 4.7430209498853, 4.7605228722096, 4.7780247945339, 4.7955267168582, 4.8130286391825, 4.8305305615068, 4.8480324838310995, 4.8655344061554, 4.8830363284797, 4.900538250804, 4.9180401731283, 4.9355420954525995, 4.9530440177769, 4.9705459401012, 4.9880478624255, 5.0055497847498, 5.0230517070741, 5.0405536293984, 5.0580555517227, 5.075557474047, 5.0930593963713, 5.1105613186956, 5.1280632410199, 5.1455651633442, 5.1630670856685, 5.1805690079927995, 5.1980709303171, 5.2155728526414, 5.2330747749657, 5.25057669729, 5.2680786196142995, 5.2855805419386, 5.3030824642629, 5.3205843865872, 5.3380863089115, 5.3555882312358, 5.3730901535601, 5.3905920758844, 5.4080939982087, 5.4255959205329995, 5.4430978428573, 5.4605997651816, 5.4781016875059, 5.4956036098302, 5.5131055321544995, 5.5306074544788, 5.5481093768031, 5.5656112991274, 5.5831132214517, 5.600615143776, 5.6181170661003, 5.6356189884246, 5.6531209107489, 5.6706228330732, 5.6881247553975, 5.7056266777218, 5.7231286000461, 5.7406305223704, 5.7581324446946995, 5.775634367019, 5.7931362893433, 5.8106382116676, 5.8281401339919, 5.8456420563161995, 5.8631439786405, 5.8806459009648, 5.8981478232891, 5.9156497456134, 5.9331516679377, 5.950653590262, 5.9681555125863, 5.9856574349106, 6.0031593572349, 6.0206612795592, 6.0381632018835, 6.0556651242078, 6.0731670465321, 6.0906689688563995, 6.1081708911807, 6.125672813505, 6.1431747358293, 6.1606766581536, 6.1781785804778995, 6.1956805028022, 6.2131824251265, 6.2306843474508, 6.2481862697751, 6.2656881920994, 6.2831901144237]

        x_idx = self.real2grid_index(np.array(scan_range_list) * np.cos(np.array(laser_angle)+np.array(yaw)),self.grid_num, self.resolution)
        y_idx = self.real2grid_index(np.array(scan_range_list) * np.sin(np.array(laser_angle)+np.array(yaw)),self.grid_num, self.resolution)
        y_idx = self.grid_num-1-y_idx

        def func1(lst, value_a, value_b):
          return [h for h, x in enumerate(lst) if value_a <= x <= value_b]
        x_n_idx = func1(x_idx,0,self.grid_num-1)
        y_n_idx = func1(y_idx,0,self.grid_num-1)

        xy_idx = (x_n_idx+np.array(y_n_idx)*self.grid_num).astype(int)

        np.put(obs_map,xy_idx,1,mode='raise')

        print(obs_map,type(obs_map))

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
