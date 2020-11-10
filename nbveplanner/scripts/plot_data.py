#!/usr/bin/python

import numpy as np
import sys
from matplotlib import pyplot as plt

if __name__ == '__main__':
    if len(sys.argv) > 1:
        path = sys.argv[1]
        if path[-1] != '/':
           path += '/'

        exploration_file = np.loadtxt(path+'exploration.txt', delimiter=';')
        plt.plot((exploration_file[:,0]-exploration_file[0,0])/60.0, exploration_file[:, 2])
        plt.xlabel('Time [min]')
        plt.ylabel('Explored Volume [m3]')
        plt.title('Volume explored over time')
        plt.show()
        plt.plot(range(len(exploration_file[:, 1])), exploration_file[:, 1])
        plt.xlabel('Iteration')
        plt.ylabel('Computation time [s]')
        plt.title('Computation time per iteration')
        plt.show()
else:
        print('Specify the data folder path to read from')