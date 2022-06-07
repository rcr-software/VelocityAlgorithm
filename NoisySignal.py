# -*- coding: utf-8 -*-
"""
Created on Sat May 21 15:23:16 2022

@author: alejo, Abel

"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import plotly.express as px
import plotly.graph_objects as go


#### Noise description
# noise = np.random.normal(0, 1, 100)
# 0 is the mean of the normal distribution you are choosing from
# 1 is the standard deviation of the normal distribution
# 100 is the number of elements you get in array noise

col_list = ["Time","Altitude", "Velocity", "Acceleration"]

df = pd.read_csv('Data_CSV.csv', comment='#', names=col_list)
# plt.plot(df)
#Comment Alejandro

timeX=df["Time"]

df["TimeAlt"]=timeX
df["TimeAcc"]=timeX

IndMaxVal = df['Altitude'].idxmax()
MaxVal = df["Altitude"].max()
NumEl = len(df['Altitude'])

"""
Adding noise to the signal in each dataframe
"""
#noise for Altitude
noiseAlt = np.random.normal(0, 0.389, NumEl) + 0.82 #Normal noise + bias

#noise for Time Altitude
noiseTimeAlt = np.random.normal(0.041, 0.006, NumEl)

#noise for Time Acceleration
noiseTimeAcc = np.random.normal(0.0108, 0.0018, NumEl)

#noise for Acceleration
noiseAcc = np.random.normal(0, 0.0148, NumEl) - 0.374 #normal noise + bias

df['Altitude']     = noiseAlt + df['Altitude']
df['TimeAlt']      = noiseTimeAlt + df['TimeAlt']

df['Acceleration'] = noiseAlt + df['Acceleration']
df['TimeAcc']      = noiseTimeAcc + df['TimeAcc']


N = 10 # number of samples the rocket will read at a time

newH = [0]
newA = [0]
vEstimation = [0]

for i in range(0, (len(df["Time"]) - 20), 1):

    # simulate how the rocket will read the data the last N samples
    rocketTimeAlt = df["TimeAlt"][i:i + N].to_numpy()
    rocketTimeAcc = df["TimeAcc"][i:i + N].to_numpy()
    
    rocketAlt = df["Altitude"][i:i + N].to_numpy()
    rocketAccel = -1 * df["Acceleration"][i:i + (N // 2)].to_numpy()

    # change in altitude component
    num = rocketTimeAlt.sum() * rocketAlt.sum() - N * (np.multiply(rocketTimeAlt, rocketAlt)).sum()
    denom = rocketTimeAlt.sum() ** 2 - N * np.square(rocketTimeAlt).sum()
    h = num / denom

    # change in acceleration component
    for t in range(0, N // 2 - 1):
        a = (1 / 2) * ((rocketAccel[t] + rocketAccel[t + 1]) *
                       (rocketTimeAcc[t + 1] - rocketTimeAcc[t]))

    #add components to get current velocity
    v = h + a
  
    newH.append(h)
    newA.append(a)
    vEstimation.append(v)

# plt.plot(df["Velocity"], label="real velocity")
# plt.plot(vEstimation, label="estimated velocity")

# plt.legend()
# plt.show()

### graph in different pltots
# fig = px.line(df, y="Velocity", x="TimeAlt")
# fig = px.line(df, y="Acceleration", x="TimeAcc")
# fig.write_html("Graph.html")
# fig.show()


fig = go.Figure()
fig.add_trace(go.Scatter(x=df["TimeAlt"], y=vEstimation,
                    mode='lines',
                    name='Estimation'))




fig.add_trace(go.Scatter(x=df["TimeAlt"], y=df["Velocity"],
                    mode='lines',
                    name='Velocity'))

fig.add_trace(go.Scatter(x=df["TimeAcc"], y=df["Acceleration"],
                    mode='lines',
                    name='lines'))
fig.add_trace(go.Scatter(x=df["TimeAlt"], y=df["Altitude"],
                    mode='lines',
                    name='Altitude'))


fig.write_html("Graph.html")
fig.show()

