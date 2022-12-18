import numpy as np
import matplotlib.pyplot as plt
import plane_fitting as plane
import transformation as trans

def point_cloud(center, depth,  Fx, Fy, k = 20, sitl = True):
    if sitl:
        center_depth = np.array(center, dtype=int)
    else:
        center_depth = np.array(center*360/480, dtype = int)
        center_depth = center_depth[0]
    data = depth[center_depth[1]-k:center_depth[1]+k +1,center_depth[0]-k:center_depth[0]+k+1]
    A = np.arange((2*k)+1)
    A = A-k
    x = []
    for i in range ((2*k)+1):
        x.append(A)
    x = np.array(x)*(-1)
    y = (x.T)
    x = x + center[0]
    y = y + center[1]
    x = x/Fx
    y = y/Fy
    x = x*data
    y = y*data
    points = []
    points_sense = []
    points.append(x)
    points.append(y)
    points.append(data)
    points = np.array(points)
    points_sense.append(x)
    points_sense.append(data)
    points_sense = np.array(points_sense)
    points_sense = np.array(points_sense[:,8:12,:])
    points_sense = np.reshape(points_sense, (2, points_sense.shape[1]*points_sense.shape[2]))
    pc = np.reshape(points, (3,((2*k)+1)**2))
    pc = pc.T
    points_sense = points_sense.T

    ipc = np.argwhere(pc[:,2] > 5.5)
    ip_sense = np.argwhere(points_sense[:,1] > 5.5)
    pc = np.delete(pc, ipc, axis = 0)
    points_sense = np.delete(points_sense, ip_sense, axis = 0)
    return pc, points_sense


def plot_plane(a,b,c,d):
    pass

def main():
    i = 2
    depth = np.load('pc/depth_' + str(i) + '.npy')
    b_box = np.load('pc/bounding_box_' + str(i) +'.npy')
    center = np.array((b_box[0] + b_box[1])/2, dtype= int)
    c_patch = np.array([(b_box[0,0] + b_box[1,0])/2 , (b_box[0,1]*3 + b_box[1,1])/4], dtype=int)
    Px, Py = b_box[2, 0], b_box[2,1]
    #pc, points_sense = point_cloud(center, depth, Px, Py, k = 25)
    pc_patch, points = point_cloud(c_patch, depth, Px, Py, k = 5)
    m = plane.face_vector(pc_patch)
    print(m)
    
    a ,b, c, d = m
    angle = trans.angle_with_z([a, b ,c])
    print(angle)


    print(b_box[1,1] - b_box[0,1])
    print(b_box[1,0] - b_box[0,0])
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    #ax.scatter(pc.T[0], pc.T[1], pc.T[2], color = 'gray')
    ax.scatter((c_patch[0]/Px)*z_patch, (c_patch[1]/Py)*z_patch, z_patch, color = 'red')
    ax.scatter(pc_patch.T[0], pc_patch.T[1], pc_patch.T[2], color= 'red')
    
    plt.show()
    plt.close('all')
    




if __name__ == '__main__':
    main()
