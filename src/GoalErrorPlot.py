#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

def GoalErrorPlot():
    Terror = []
    Rerror = []
    Tpara = [10, 0.1, 1, 1, 2, 1]
    Rpara = [30, 5, 1, 1, 2, 1]
    # text = '[Translation]\nw_u = %f\nw_l = %f\nsigma = %f\nalpha = %f\neta = %f\ngain = %f\n\n[Rotation]\nw_u = %f\nw_l = %f\nsigma = %f\nalpha = %f\neta = %f\ngain = %f' % (Tpara[0], Tpara[1], Tpara[2], Tpara[3], Tpara[4], Tpara[5], Rpara[0], Rpara[1], Rpara[2], Rpara[3], Rpara[4], Rpara[5])
    text = ('[Translation]\n'
            '  w_u = %.2f\n'
            '  w_l = %.2f\n'
            '  sigma = %.2f\n'
            '  alpha = %.2f\n'
            '  eta = %.2f\n'
            '  gain = %.2f\n\n'
            '[Rotation]\n'
            '  w_u = %.2f\n'
            '  w_l = %.2f\n'
            '  sigma = %.2f\n'
            '  alpha = %.2f\n'
            '  eta = %.2f\n'
            '  gain = %.2f' 
            % (Tpara[0], Tpara[1], Tpara[2], Tpara[3], Tpara[4], Tpara[5], 
               Rpara[0], Rpara[1], Rpara[2], Rpara[3], Rpara[4], Rpara[5]))

    with open('/home/aiRobots/Calvin/src/calvin/PathPlanning/src/GoalError.txt', 'r') as f:
        for line in f.readlines():
            tmp = line.split(" ")
            Terror.append(float(tmp[0]))
            Rerror.append(float(tmp[1]))
    time = np.arange(len(Terror))
    plt.figure(figsize=(10, 6))
    plt.plot(time, Terror, label='Translation Error')
    plt.plot(time, Rerror, label='Rotation Error')
    plt.xlim(0, 1000)
    plt.ylim(0, 3)
    plt.title('Goal Error')
    plt.text(1030, 2.9, text, fontsize=8, verticalalignment="top", horizontalalignment="left", bbox=dict(facecolor='none', edgecolor='black'))
    plt.subplots_adjust(right=0.8)
    plt.xlabel('Time Step')
    plt.ylabel('Error')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    GoalErrorPlot()
