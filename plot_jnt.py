import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

df = pd.read_csv("build/meas_jnt_pos.csv", names=["j0", "j1", "j2", "j3", "j4", "j5", "j6"])
df2 = pd.read_csv("build/setpoint.csv", names=["t", "setpoint"])

time = np.arange(0, 10*len(df), 10)
df["time"] = time
# dataArray = np.genfromtxt('build/meas_jnt_pos.csv', delimiter=',')

position_func = lambda t_ms: -(3.14*2*3.14*0.3/4)*np.sin(0.3*2*3.14*t_ms/1000)

fig, ax = plt.subplots(1, 1, figsize=(8, 6))

ax.plot(df["time"], df["j6"], label="j6")
# ax.plot(df["time"], position_func(df["time"]), label="setpoint")
ax.plot(df2["t"] * 1000, df2["setpoint"], label="setpoint")
ax.grid(True)
ax.legend()
plt.show()


# plt.figure()
# for col_i in range(dataArray.shape[1]):
    # plt.plot(dataArray[:,col_i])
# plt.legend()
# plt.show()
