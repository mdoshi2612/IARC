from turtle import color
import PID
import numpy as np
import matplotlib.pyplot as plt

p = PID.PID()
o = []
rel_vel = 10
rel_pos = 0
for i in range(0,500):
    
    out  = -1*p.update(rel_pos, rel_vel)
    rel_vel += out/20
    rel_pos += rel_vel/20
    o.append([rel_pos, rel_vel, out])



plt.plot(np.array(o).T[0], color = 'red')
plt.plot(np.array(o).T[1], color = 'blue')
plt.plot(np.array(o).T[2], color = 'gray')
plt.show()
