import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

df = pd.read_csv("build/meas_jnt_pos.csv", names=["q0", "q1", "q2", "q3", "q4", "q5", "q6", "v0", "v1", "v2", "v3", "v4", "v5", "v6"])
df2 = pd.read_csv("build/setpoint.csv", names=["t", "setpoint"])
df3 = pd.read_csv("build/setpoint_wrench.csv", names=["t","setpoint"])
df4 = pd.read_csv("build/meas_wrench.csv", names= ["w0", "w1", "w2", "w3", "w4", "w5"])

time = np.arange(0, 4*len(df), 4)
df["time"] = time
# dataArray = np.genfromtxt('build/meas_jnt_pos.csv', delimiter=',')

position_func = lambda t_ms: -(3.14*2*3.14*0.3/4)*np.sin(0.3*2*3.14*t_ms/1000)

fig, [[ax1, ax3],[ax2, ax4]] = plt.subplots(2, 2, figsize=(8, 12))

ax1.plot(df["time"], df["q6"], label="q6")
# ax.plot(df["time"], position_func(df["time"]), label="setpoint")
ax1.plot(df2["t"] * 1000, df2["setpoint"], ".-", label="setpoint")

ax2.plot(df["time"], df["v6"], label="v6")

ax3.plot(df["time"], df4["w5"], label="wrench 5")
# ax.plot(df["time"], position_func(df["time"]), label="setpoint")
ax3.plot(df3["t"] * 1000, df3["setpoint"], ".-", label="setpoint")

for axis in [ax1, ax2, ax3]:
    axis.grid(True)
    axis.legend()

plt.show()



# plt.figure()
# for col_i in range(dataArray.shape[1]):
    # plt.plot(dataArray[:,col_i])
# plt.legend()
# plt.show()
