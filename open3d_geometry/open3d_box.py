import numpy as np
import open3d as o3d

from .open3d_arrow import create_arrow # the dot '.' is essential
from .open3d_utils import box_dim2corners, get_box_arrow


def create_box_from_corners(corners, color=None):
	'''
	corners: 8 corners(x, y, z)
	corners: array = 8*3
	#         7 -------- 6
	#        /|         /|
	#       4 -------- 5 .
	#       | |        | |
	#       . 3 -------- 2
	#       |/         |/
	#       0 -------- 1
	'''
	# 12 lines in a box
	lines = [[0, 1], [1, 2], [2, 3], [3, 0], 
				[4, 5], [5, 6], [6, 7], [7, 4],
				[0, 4], [1, 5], [2, 6], [3, 7]]
	if color == None:
		color = [1, 0, 0] # red
	colors = [color for i in range(len(lines))]
	line_set = o3d.geometry.LineSet()
	line_set.points = o3d.utility.Vector3dVector(corners)
	line_set.lines = o3d.utility.Vector2iVector(lines)
	line_set.colors = o3d.utility.Vector3dVector(colors)

	return line_set


def create_box_from_dim(dim, color=None):
    '''
    dim: list(8) [h, w, l, x, y, z, yaw]
    '''
    box_corners = box_dim2corners(dim)
    box = create_box_from_corners(box_corners, color)
    return box


def create_box_from_dim_with_arrow(dim, color=None):
    '''
    dim: list(8) [h, w, l, x, y, z, yaw]
    '''

    box = create_box_from_dim(dim, color)

    vec = get_box_arrow(dim)
    a_start = [vec[0], vec[1], vec[2]]
    a_end = [vec[3], vec[4], vec[5]]
    arrow = create_arrow(a_start, a_end, color)

    return box, arrow

