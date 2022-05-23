# -*- coding: utf-8 -*-
"""
Created on Sat May 21 15:23:16 2022

@author: alejo, Abel

"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

noise = np.random.normal(0, 1, 100)

# 0 is the mean of the normal distribution you are choosing from
# 1 is the standard deviation of the normal distribution
# 100 is the number of elements you get in array noise


col_list = ["Time", "Altitude", "Velocity", "Acceleration"]

df = pd.read_csv('BV_timealtvelaccel.csv', comment='#', names=col_list)
# plt.plot(df)


IndMaxVal = df['Altitude'].idxmax()
MaxVal = df["Altitude"].max()
NumEl = len(df['Altitude'])

scale = 10
noise = np.random.normal(0, 1, NumEl) * scale

# plt.plot(noise)

NSignal = df["Altitude"] + noise


# plt.plot(NSignal)
# plt.hist(NSignal)

N = 10 # number of samples the rocket will read at a time

newH = [0]
newA = [0]
vEstimation = [0]


for i in range(0, (len(df["Time"]) - 20), 1):

    # simulate how the rocket will read the data the last N samples
    rocketTime = df["Time"][i:i + N].to_numpy()
    rocketAlt = df["Altitude"][i:i + N].to_numpy()
    rocketAccel = -1 * df["Acceleration"][i:i + (N // 2)].to_numpy()

    # change in altitude component
    num = rocketTime.sum() * rocketAlt.sum() - N * (np.multiply(rocketTime, rocketAlt)).sum()
    denom = rocketTime.sum() ** 2 - N * np.square(rocketTime).sum()
    h = num / denom

    # change in acceleration component
    for t in range(0, N // 2 - 1):
        a = (1 / 2) * ((rocketAccel[t] + rocketAccel[t + 1]) *
                       (rocketTime[t + 1] - rocketTime[t]))

    #add components to get current velocity
    v = h + a


    newH.append(h)
    newA.append(a)
    vEstimation.append(v)

plt.plot(df["Velocity"], label="real velocity")
plt.plot(vEstimation, label="estimated velocity")

# plt.plot(newH, label=" a")
# plt.plot(newA, label="h")

plt.legend()
plt.show()
