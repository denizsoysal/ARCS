import matplotlib.pyplot as plt
import numpy as np

dataArray = np.genfromtxt('build/meas_jnt_pos.csv', delimiter=',')

plt.figure()
for col_i in range(dataArray.shape[1]):
    plt.plot(dataArray[:,col_i])
plt.legend()
plt.show()
