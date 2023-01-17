import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv("/home/brendan/Desktop/data/iiwase_log_1.csv", header=None, 
                names=["time", "activity", "stream", "signal", "value"], parse_dates=["time"])

df1 = df[df["signal"]==" ek_bar"]
df2 = df[df["signal"]==" bias"]
df3 = df[df["signal"]==" gain"]
df4 = df[df["signal"]==" control"]

fig, [ax1,ax2] = plt.subplots(2, 1, figsize=(8, 12))

ax1.plot(df1["time"], df1["value"], label='ek_bar')
ax1.plot(df2["time"], df2["value"], label='bias')
ax1.plot(df3["time"], df3["value"], label='gain')
ax1.plot(df4["time"], df4["value"], label='control')

dfa = df[df["signal"]==" meas_jnt_vel[1]"]
dfb = df[df["signal"]==" meas_jnt_vel[5]"]
dfc = df[df["signal"]==" meas_jnt_vel[3]"]

ax2.plot(dfa["time"], dfa["value"], label="vel1")
ax2.plot(dfb["time"], dfb["value"], label="vel5")
ax2.plot(dfc["time"], dfc["value"], label="vel3")

dft = df[df["signal"]==" cycle_time_us"]

# ax2.plot(dft["time"], dft["value"], label="cycle_time_us")

ax1.legend()
ax2.legend()

# ax1.set(xlabel="time (µs)", ylabel="EE speed (m/s)")

# ax2.plot(df["time_us"], df["abag_control"])
# ax2.set(xlabel="time (µs)", ylabel="Abag control (N)")

plt.show()
