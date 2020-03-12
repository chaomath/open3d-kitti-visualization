import numpy as np
import open3d as o3d

from open3d_geometry.open3d_box import create_box_from_dim_with_arrow
from open3d_geometry.open3d_coordinate import create_coordinate

class PointCloudVis:

	@staticmethod
	def draw_points(points):
		'''
		points: numpy array-->(n*4) or (n*3)-->(x, y, z, intensity)
		'''
		points = points[:, :3]

		colors = [[0.5, 0.5, 0.5] for i in range(points.shape[0])]
		pc = o3d.geometry.PointCloud()

		pc.points = o3d.utility.Vector3dVector(points)
		pc.colors = o3d.utility.Vector3dVector(colors)

		vis = o3d.visualization.Visualizer()
		vis.create_window()
		vis.add_geometry(pc)
		vis.get_render_option().point_size = 2
		vis.run()
		vis.destroy_window()

	@staticmethod
	def draw_pcd(pcd):
		vis = o3d.visualization.Visualizer()
		vis.create_window()
		vis.add_geometry(pcd)
		vis.get_render_option().point_size = 2
		vis.run()
		vis.destroy_window()


	@staticmethod
	def draw_points_with_boxes(points, boxes, box_colors=[]):
		'''
		points: (m, 4) or (m, 3) [x, y, z, intensity]
		boxes: np.array = n*7 (h, w, l, x, y, z, yaw)
		box_colors: list(n) color of each box
		'''

		#--------------------------------------------------------------
		# get colors for each box
		#--------------------------------------------------------------
		if len(box_colors) == 0:
			box_colors = [[1, 0, 0] for i in range(boxes.shape[0])] # red

		#--------------------------------------------------------------
		# create boxes
		#--------------------------------------------------------------
		boxes_o3d = []
		for i in range(boxes.shape[0]):
			dim = boxes[i]
			color = box_colors[i]
			box_o3d, arrow = create_box_from_dim_with_arrow(dim, color)
			boxes_o3d.append(box_o3d)
			boxes_o3d.append(arrow)

		#--------------------------------------------------------------
		# coordinate frame
		#--------------------------------------------------------------
		coordinate_frame = create_coordinate(size=2.0, origin=[0, 0, 0])

		#--------------------------------------------------------------
		# point cloud
		#--------------------------------------------------------------
		points = points[:, :3]
		point_color = [0.5, 0.5, 0.5]
		point_colors = [point_color for i in range(points.shape[0])] # points color
		pc = o3d.geometry.PointCloud()
		pc.points = o3d.utility.Vector3dVector(points)
		pc.colors = o3d.utility.Vector3dVector(point_colors)

		#--------------------------------------------------------------
		# draw geometry in open3d
		#--------------------------------------------------------------
		vis = o3d.visualization.Visualizer()
		vis.create_window()

		vis.add_geometry(coordinate_frame)
		vis.add_geometry(pc)

		[vis.add_geometry(element) for element in boxes_o3d]

		vis.get_render_option().point_size = 2
		vis.run()
		vis.destroy_window()
