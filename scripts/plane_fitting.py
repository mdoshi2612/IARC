#!/usr/bin/env python3
import numpy as np
from ransac import *

goal_inliers = 21*21 * 0.3
max_iterations = 1009


def point_cloud(center, center_patch, depth,  Fx, Fy, kx = 10, ky = 5, sitl = True , t = False):
    if kx < 2:
        kx = 2
    if ky < 2:
        ky = 2
    z_threshold = 10.5
    if sitl:
        center_depth = np.array(center_patch, dtype=int)
    else:
        center_depth = np.array(center*360/480, dtype = int)
        center_depth = center_depth
    #data = depth[center_depth[1]-ky:center_depth[1]+ky +1,center_depth[0]-kx:center_depth[0]+kx+1]
    # if center_depth[1]-ky 
    
    data = depth[center_depth[1]-ky:center_depth[1]+ky +1,center_depth[0]-kx:center_depth[0]+kx+1]
    data = np.reshape(data, (data.shape[0], data.shape[1]))
    if t:
        data = depth[1:,1:]
    A = np.arange((2*kx)+1)
    A = A-kx
    x = []
    for i in range ((2*ky)+1):
        x.append(A)
    x = np.array(x)
    x = x*(-1)
    A = np.arange((2*ky)+1)
    A = A-ky
    y = []
    for i in range ((2*kx)+1):
        y.append(A)
    y = np.array(y)*(-1)
    y = y.T
    x = x + center[0]
    y = y + center[1]
    y = y
    x = x/Fx
    y = y/Fy
    try:
        x = x*data
        y = y*data
    except:
        print(center_depth-ky)
        print(depth.shape)
        print(data.shape)
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
    pc = np.reshape(points, (3,(points.shape[1]*points.shape[2])))
    pc = pc.T
    points_sense = points_sense.T

    ipc = np.argwhere(pc[:,2] > z_threshold)
    ip_sense = np.argwhere(points_sense[:,1] > z_threshold)
    pc = np.delete(pc, ipc, axis = 0)
    points_sense = np.delete(points_sense, ip_sense, axis = 0)

    return pc, points_sense, points



def face_vector(pc):
    goal_inliers = pc.shape[0]*0.6
    max_iterations = 10090000
    m, best_inliers, best_fit_found = run_ransac(pc, estimate, lambda x, y: is_inlier(x, y, 0.01), 3, goal_inliers, max_iterations)
    return m, best_fit_found




def augment(xyzs):
    axyz = np.ones((len(xyzs), 4))
    axyz[:, :3] = xyzs
    return axyz

def estimate(xyzs):
    axyz = augment(xyzs[:3])
    return np.linalg.svd(axyz)[-1][-1, :]

def is_inlier(coeffs, xyz, threshold):
    return np.abs(coeffs.dot(augment([xyz]).T)) < threshold

if __name__ == '__main__':
    import matplotlib
    import matplotlib.pyplot as plt
    from mpl_toolkits import mplot3d
    fig = plt.figure()
    ax = mplot3d.Axes3D(fig)

    def plot_plane(a, b, c, d):
        xx, yy = np.mgrid[:10, :10]
        return xx, yy, (-d - a * xx - b * yy) / c

    n = 100
    max_iterations = 100
    goal_inliers = n * 0.3

    # test data
    xyzs = np.random.random((n, 3)) * 10
    xyzs[:50, 2:] = xyzs[:50, :1]

    ax.scatter3D(xyzs.T[0], xyzs.T[1], xyzs.T[2])

    # RANSAC
    m, best_inliers = run_ransac(xyzs, estimate, lambda x, y: is_inlier(x, y, 0.01), 3, goal_inliers, max_iterations)
    a, b, c, d = m
    xx, yy, zz = plot_plane(a, b, c, d)
    ax.plot_surface(xx, yy, zz, color=(0, 1, 0, 0.5))

    plt.show()
