import gym
from gym import error, spaces, utils
from gym.utils import seeding
import pybullet as p
import time
import pybullet_data
import os
from pprint import pprint
import numpy as np


class Ur5ReachEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.assets = os.path.abspath(__file__ + "/../../") + '/assets'
        # or p.DIRECT for non-graphical version
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setGravity(0, 0, -10)
        self.robot_id = 0
        self.robot_start_pos = [0, 0, 0]
        self.joint_list = list(range(1,7))
        self.max_forces = [150.0, 150.0, 150.0, 28.0, 28.0, 28.0]
        self.robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])

    def step(self, action):
        transformed_action = [k*np.pi for k in action]
        p.setJointMotorControlArray(bodyIndex=self.robot_id, 
                            jointIndices=self.joint_list,       
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocities = transformed_action,
                            forces = self.max_forces)
        p.stepSimulation()
        reward, done = self.get_reward()
        state = self.get_observation()
        return state, reward, done

    def reset(self):
        p.resetSimulation()
        planeId = p.loadURDF("plane.urdf")

        print(self.joint_list)

        self.robot_id = p.loadURDF(self.assets+"/ur_description/ur5.urdf", self.robot_start_pos,
                                  self.robot_start_orientation, flags=p.URDF_USE_SELF_COLLISION)
        for i in range(1,7):
            p.enableJointForceTorqueSensor(self.robot_id,i)
        p.stepSimulation()
        reward = self.get_reward
    def render(self, mode='human', close=False):
        ...
    def get_reward(self):
        reward = 0
        done = False
        return reward, done
    def get_observation(self):
        observation = []
        
        return observation
    def terminate(self):
        p.disconnect()

if __name__ == ('__main__'):
    env = Ur5ReachEnv()
    env.reset()
    env.step([1,1,1,1,1,1])
