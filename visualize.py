import argparse
import os
import numpy as np

from kitti_utils import load_kitti_calib, read_objs2velo, colors_list
from PointCloudVis import PointCloudVis


def vis_kitti(data_path, frame):

    # get data
    lidar_path = os.path.join(data_path, 'velodyne')
    label_path = os.path.join(data_path, 'label_2')
    calib_path = os.path.join(data_path, 'calib')

    lidar_file = os.path.join(lidar_path, '%06d.bin'%frame)
    label_file = os.path.join(label_path, '%06d.txt'%frame)
    calib_file = os.path.join(calib_path, '%06d.txt'%frame)

    # get calibration
    calib = load_kitti_calib(calib_file)

    # get points and boxes
    boxes_velo, objs_type = read_objs2velo(label_file, calib['Tr_velo2cam'])
    points = np.fromfile(lidar_file, dtype=np.float32).reshape(-1, 4)

    #--------------------------------------------------------------
    # get colors for each box
    #--------------------------------------------------------------
    box_colors = []
    if len(objs_type) == 0:
        box_colors = [[1, 0, 0] for i in range(boxes_velo.shape[0])] # red
    else:
        box_colors = [colors_list[int(i)] for i in objs_type]

    # draw boxes with arrows
    PointCloudVis.draw_points_with_boxes(points, boxes_velo, box_colors)



if __name__ == "__main__":
	
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", "--d", type=str, default="./data/KITTI/training", help="kitti data path")
    parser.add_argument("--frame", "--f", type=int, default=500, help="frame of the data")
    opt = parser.parse_args()

    data_path = opt.data
    frame = opt.frame

    vis_kitti(data_path, frame)
