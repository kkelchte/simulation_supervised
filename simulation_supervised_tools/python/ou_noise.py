# --------------------------------------
# Reference: https://github.com/rllab/rllab/blob/master/rllab/exploration_strategies/ou_strategy.py
# --------------------------------------

import numpy as np
import numpy.random as nr


class OUNoise:
    """ docstring for OUNoise """
    def __init__(self,action_dimension,mu=0, theta=0.15, sigma=0.3):
        self.action_dimension = action_dimension
        self.mu = mu
        self.theta = theta
        self.sigma = sigma
        self.state = np.ones(self.action_dimension) * self.mu
        self.reset()

    def reset(self):
        self.state = np.ones(self.action_dimension) * self.mu

    def noise(self):
        x = self.state
        dx = self.theta * (self.mu - x) + self.sigma * nr.randn(len(x))
        self.state = x + dx
        return self.state

if __name__ == '__main__':
    # for theta, sigma in [ (0.0015,0.00003), (0.003,0.00015), (0.006,0.0006), (0.015,0.003), (0.03,0.015), (0.06,0.06), (0.15,0.3) ]: 
    for theta in [ 0.0015, 0.003, 0.006, 0.015, 0.03, 0.06, 0.15 ]:
        print('theta: ', theta)
        ou = OUNoise(4,0,theta,0.3)
        states = []
        total=100
        for i in range(total):
          #if i%100: print(i," of ",total)
          #states.append(nr.rand()-0.5)
          states.append(ou.noise())
        import matplotlib.pyplot as plt
        plt.plot(states)
        plt.show()
