from gym_pybullet.envs import Ur5ReachEnv
import numpy as np
import gym

def Test():
    env = Ur5ReachEnv()
    episode_count = 2000
    max_steps = 100000
    reward = 0
    state = []
    done = False
    for i in range(episode_count):
        steps = 0

        print("Episode : " + str(i) )

        state = env.reset()
     
        for j in range(max_steps):
            action = np.random.uniform(-1,1,6)
            state, reward, done= env.step(action)
            if done:
                break

    env.terminate()
    print("Finish.")

if __name__ == "__main__":
    Test()