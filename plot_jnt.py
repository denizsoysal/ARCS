import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

df = pd.read_csv("build/meas_jnt_pos.csv", names=["q0", "q1", "q2", "q3", "q4", "q5", "q6", "v0", "v1", "v2", "v3", "v4", "v5", "v6"])
df2 = pd.read_csv("build/setpoint.csv", names=["t", "setpoint"])

time = np.arange(0, 4*len(df), 4)
df["time"] = time
# dataArray = np.genfromtxt('build/meas_jnt_pos.csv', delimiter=',')

# position_func = lambda t_ms: -(3.14*2*3.14*0.3/4)*np.sin(0.3*2*3.14*t_ms/1000)

fig, [ax1, ax2] = plt.subplots(2, 1, figsize=(8, 12))

ax1.plot(df["time"], df["q6"], label="q6")
ax1.set(xlabel="time (ms)", ylabel="Joint position (rad)")
# ax.plot(df["time"], position_func(df["time"]), label="setpoint")
ax1.plot(df2["t"] * 1000, df2["setpoint"], ".-", label="setpoint")

ax2.plot(df["time"], df["v6"], label="v6")
ax2.set(xlabel="time (ms)", ylabel="Joint velocity (rad/s)")

for axis in [ax1, ax2]:
    axis.grid(True)
    axis.legend()

plt.show()



# plt.figure()
# for col_i in range(dataArray.shape[1]):
    # plt.plot(dataArray[:,col_i])
# plt.legend()
# plt.show()
