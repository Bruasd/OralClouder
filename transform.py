# import open3d as o3d
# import numpy as np
#
# pcd = o3d.io.read_point_cloud("data\\target\\highpart.ply")
# xyz = np.asarray(pcd.points)
#
# xyz1 = xyz[[np.all(((xyz[i,1] >5)&((xyz[i,0]**2+xyz[i,2]**2)>6))|((xyz[i,1] >6)&((xyz[i,0]**2+xyz[i,2]**2)>2.5))|(xyz[i,1] >9.2)) for i in range(xyz.shape[0])], :]
# y=xyz1[:,1]
# print(xyz1.size)
# print(max(y))
# print(min(y))
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.vectors[3]dVector(xyz1)
# pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=50))
# print(pcd.normals)
# o3d.io.write_point_cloud("data\\target\\highpart_processed.ply", pcd)
# o3d.visualization.draw_geometries([pcd])

# pcd = o3d.io.read_point_cloud("data\\testmodel\\ModelLow2.ply")
# xyz = np.asarray(pcd.points)
#
# xyz1 = xyz[[np.all((xyz[i,1] >35)&(xyz[i,2] >-5)) for i in range(xyz.shape[0])], :]
# y=xyz1[:,1]
# print(xyz1.size)
# print(max(y))
# print(min(y))
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.vectors[3]dVector(xyz1)
# pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=50))
# print(pcd.normals)
# o3d.io.write_point_cloud("data\\testmodel\\Low2Processed.ply", pcd)
# o3d.visualization.draw_geometries([pcd])

import open3d as o3d
import numpy as np

# 读取点云
# pcd = o3d.io.read_point_cloud("data\\source\\low.ply")
#
# # 定义交错距离和拉伸缩小因子
# interleave_distance = 100  # 交错距离
# stretch_factor = 1.001  # 拉伸因子
#
# shrink_factor = 2-stretch_factor  # 缩小因子
#
# # 获取点云的坐标
# xyz = np.asarray(pcd.points)
#
# # 初始化结果点云数组
# modified_points = []
#
# # 分段进行拉伸和缩小
# for i in range(0, xyz.shape[0], interleave_distance):
#     segment = xyz[i:i + interleave_distance]
#
#     if i % 2 == 0:
#         # 拉伸
#         modified_segment = segment * stretch_factor
#     else:
#         # 缩小
#         modified_segment = segment * shrink_factor
#
#     modified_points.append(modified_segment)
#
# # 将所有段的修改后的点组合成新的点云
# modified_points = np.concatenate(modified_points, axis=0)
# modified_pcd = o3d.geometry.PointCloud()
# modified_pcd.points = o3d.utility.vectors[3]dVector(modified_points)
# modified_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=50))
# # 可视化原始点云和修改后的点云
# o3d.visualization.draw_geometries([modified_pcd])
# o3d.io.write_point_cloud("sources\\strech\\01both.ply",modified_pcd)


# def read_ply(file_path):
#     points = []
#
#     with open(file_path, 'r') as ply_file:
#         lines = ply_file.readlines()
#         for line in lines:
#             if line.strip() == 'end_header':
#                 break
#
#         for line in lines:
#             parts = line.strip().split()
#             if len(parts) == 3:
#                 x, y, z = map(float, parts)
#                 points.append((x, y, z))
#
#     return points
#
#
# def get_coordinate_range(points):
#     if not points:
#         return None
#
#     min_x = min(p[0] for p in points)
#     max_x = max(p[0] for p in points)
#     min_y = min(p[1] for p in points)
#     max_y = max(p[1] for p in points)
#     min_z = min(p[2] for p in points)
#     max_z = max(p[2] for p in points)
#
#     return (min_x, max_x, min_y, max_y, min_z, max_z)
#
#
# ply_file_path = 'data\\target\\lowref.ply'  # 替换为您的PLY文件路径
# points = read_ply(ply_file_path)
# coordinate_range = get_coordinate_range(points)
#
# if coordinate_range:
#     print("X range:", coordinate_range[0], "-", coordinate_range[1])
#     print("Y range:", coordinate_range[2], "-", coordinate_range[3])
#     print("Z range:", coordinate_range[4], "-", coordinate_range[5])
# else:
#     print("No points found in the PLY file.")





