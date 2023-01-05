import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv("../data/logging2.csv", header=0)

df.head()

fig, [ax1,ax2] = plt.subplots(2, 1, figsize=(8, 12))

ax1.plot(df["time_us"], df["local_vz"])
ax1.set(xlabel="time (µs)", ylabel="EE speed (m/s)")

ax2.plot(df["time_us"], df["abag_control"])
ax2.set(xlabel="time (µs)", ylabel="Abag control (N)")

plt.show()
