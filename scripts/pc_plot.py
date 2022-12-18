import numpy as np
import matplotlib.pyplot as plt

pc = np.load('pc.npy')
pc_ = np.load('pc_.npy')

ipc = np.argwhere(pc[:,1] < 1.5)
pc = np.delete(pc, ipc, axis = 0)
ipc = np.argwhere(pc[:,1] > 2)
pc = np.delete(pc, ipc, axis = 0)

fig = plt.figure()
ax = plt.axes(projection='3d')


ax.scatter(pc.T[0], pc.T[2], pc.T[1], color = [(0.8,0.8,0.8)])
ax.scatter(pc_.T[0], pc_.T[2], pc_.T[1], color = 'black')
plt.show()