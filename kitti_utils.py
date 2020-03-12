import os
import numpy as np

# classes
class_list = ['Car', 'Van' , 'Truck' , 'Pedestrian' , 'Person_sitting' , 'Cyclist' , 'Tram', 'Misc']
colors_list = [[1, 0, 0],  		 # car------------------>red
				[0.5, 1, 0], 	 # Van
				[0, 1, 0.5], 	 # Truck
				[0, 1, 0], 		 # Pedestrian----------->green
				[0.2, 0.5, 0], 	 # Person_sitting
				[0, 0, 1], 		 # Cyclist-------------->blue
				[0.7, 0.7, 0.3], # Tram
				[0.2, 0.5, 0.7]] # Misc


def load_kitti_calib(calib_file):
    """
    load projection matrix
    """
    with open(calib_file) as fi:
        lines = fi.readlines()
        assert (len(lines) == 8)

    obj = lines[0].strip().split(' ')[1:]
    P0 = np.array(obj, dtype=np.float32)
    obj = lines[1].strip().split(' ')[1:]
    P1 = np.array(obj, dtype=np.float32)
    obj = lines[2].strip().split(' ')[1:]
    P2 = np.array(obj, dtype=np.float32)
    obj = lines[3].strip().split(' ')[1:]
    P3 = np.array(obj, dtype=np.float32)
    obj = lines[4].strip().split(' ')[1:]
    R0 = np.array(obj, dtype=np.float32)
    obj = lines[5].strip().split(' ')[1:]
    Tr_velo_to_cam = np.array(obj, dtype=np.float32)
    obj = lines[6].strip().split(' ')[1:]
    Tr_imu_to_velo = np.array(obj, dtype=np.float32)

    return {'P2': P2.reshape(3, 4),
            'R0': R0.reshape(3, 3),
            'Tr_velo2cam': Tr_velo_to_cam.reshape(3, 4)}


def project_cam2velo(cam, Tr):
    T = np.zeros([4, 4], dtype=np.float32)
    T[:3, :] = Tr
    T[3, 3] = 1
    T_inv = np.linalg.inv(T)
    lidar_loc_ = np.dot(T_inv, cam)
    lidar_loc = lidar_loc_[:3]
    return lidar_loc.reshape(1, 3)

def ry_to_rz(ry):
    angle = -ry - np.pi / 2

    if angle >= np.pi:
        angle -= np.pi
    if angle < -np.pi:
        angle = 2*np.pi + angle

    return angle



class KittiObject(object):
    ''' kitti 3d object label '''
    def __init__(self, label_file_line):
        data = label_file_line.split(' ')
        data[1:] = [float(x) for x in data[1:]]

        # extract label, truncation, occlusion
        self.type = data[0] # 'Car', 'Pedestrian', ...
        self.truncation = data[1] # truncated pixel ratio [0..1]
        self.occlusion = int(data[2]) # 0=visible, 1=partly occluded, 2=fully occluded, 3=unknown
        self.alpha = data[3] # object observation angle [-pi..pi]

        # extract 2d bounding box in 0-based coordinates
        self.xmin = data[4] # left
        self.ymin = data[5] # top
        self.xmax = data[6] # right
        self.ymax = data[7] # bottom
        self.box2d = np.array([self.xmin,self.ymin,self.xmax,self.ymax])
        
        # extract 3d bounding box information
        self.h = data[8] # box height
        self.w = data[9] # box width
        self.l = data[10] # box length (in meters)
        self.t = (data[11],data[12],data[13]) # location (x,y,z) in camera coord.
        self.ry = data[14] # yaw angle (around Y-axis in camera coordinates) [-pi..pi]

    def __str__(self):
        str0 = ('Type, truncation, occlusion, alpha: %s, %d, %d, %f\n' % \
            (self.type, self.truncation, self.occlusion, self.alpha))
        str1 = ('2d bbox (x0,y0,x1,y1): %f, %f, %f, %f\n' % \
            (self.xmin, self.ymin, self.xmax, self.ymax))
        str2 = ('3d bbox h,w,l: %f, %f, %f\n' % \
            (self.h, self.w, self.l))
        str3 = ('3d bbox location, ry: (%f, %f, %f), %f\n' % \
            (self.t[0],self.t[1],self.t[2],self.ry))

        return (str0 + str1 + str2 + str3)


def get_obj_type(obj_str):
    obj_type = -1
    for i in range(len(class_list)):
        if obj_str == class_list[i]:
            obj_type = i
    return obj_type


def read_objs2velo(label_file, Tr_velo2cam):
    '''
    Tr_velo2cam: (3, 4)
    '''

    lines = [line.rstrip() for line in open(label_file)]
    objs_velo = []
    objs_type = []
    for line in lines:
        obj = KittiObject(line)
        if obj.type == 'DontCare':
            continue
        obj_type = get_obj_type(obj.type)
        h = obj.h
        w = obj.w
        l = obj.l
        x = obj.t[0]
        y = obj.t[1]
        z = obj.t[2]
        ry = obj.ry

        rz = ry_to_rz(ry) # ry in camera, rz in velo

        pos_cam = np.ones([4, 1])
        pos_cam[0] = x
        pos_cam[1] = y
        pos_cam[2] = z
        pos_velo = project_cam2velo(pos_cam, Tr_velo2cam) # pos_velo: (1,3)
        x_velo = pos_velo[0][0]
        y_velo = pos_velo[0][1]
        z_velo = pos_velo[0][2]
        obj_velo = [h, w, l, x_velo, y_velo, z_velo, rz, obj_type]
        objs_type.append(obj_type)

        objs_velo.append(obj_velo)
    objs_velo = np.array(objs_velo) #(n, 8)

    return objs_velo, objs_type 
