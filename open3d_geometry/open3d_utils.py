import numpy as np


def rotz(t):
    ''' Rotation about the z-axis. '''
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, -s,  0],
                     [s,  c,  0],
                     [0,  0,  1]])


def get_box_arrow(dim):
    h = dim[0]
    # w = dim[1]
    l = dim[2]
    x = dim[3]
    y = dim[4]
    z = dim[5]
    yaw = dim[6]

    # get direction arrow
    dx = l/2.0*np.cos(yaw)
    dy = l/2.0*np.sin(yaw)
    # a_start = [x, y, z+h]
    # a_end = [x+dx, y+dy, z+h]
    # arrow = [a_start, a_end]
    arrow = [x, y, z+h, x+dx, y+dy, z+h] # [x0, y0, z0, x1, y1, z1], point0--->point1
    return arrow


def box_dim2corners(dim):
    '''
    dim: [h, w, l, x, y, z, yaw]

    8 corners: np.array = n*8*3(x, y, z)
    #         7 -------- 6
    #        /|         /|
    #       4 -------- 5 .
    #       | |        | |
    #       . 3 -------- 2            
    #       |/         |/
    #       0 -------- 1

                ^ x(l)
                |
                |
                |
    y(w)        |
    <-----------O
    '''
    h = dim[0]
    w = dim[1]
    l = dim[2]
    x = dim[3]
    y = dim[4]
    z = dim[5]
    yaw = dim[6]

    # 3d bounding box corners
    Box = np.array([[-l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2],
                    [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2],
                    [0, 0, 0, 0, h, h, h, h]])

    R = rotz(yaw)
    corners_3d = np.dot(R, Box) # corners_3d: (3, 8)

    corners_3d[0,:] = corners_3d[0,:] + x
    corners_3d[1,:] = corners_3d[1,:] + y
    corners_3d[2,:] = corners_3d[2,:] + z

    return np.transpose(corners_3d)
