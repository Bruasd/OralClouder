import open3d as o3d

# mesh = o3d.io.read_triangle_mesh("data\\target\\highpart.stl")
# # mesh.compute_vertex_normals()
# # o3d.visualization.draw_geometries([mesh])
# # 观察stl模型
# # pcd = mesh.sample_points_uniformly(number_of_points=50)
# # 该方法可以在曲面上均匀抽样产生点群
# pcd = mesh.sample_points_poisson_disk(number_of_points=10000, init_factor=5)
# # 该方法可以使曲面上抽样产生的点均匀分布
# o3d.visualization.draw_geometries([pcd])
# o3d.io.write_point_cloud("data\\target\\highpart.ply", pcd)

mesh = o3d.io.read_triangle_mesh("data\\group0\\3-25A.stl")
mesh.compute_vertex_normals()
# o3d.visualization.draw_geometries([mesh])
# 观察stl模型
# pcd = mesh.sample_points_uniformly(number_of_points=50)
# 该方法可以在曲面上均匀抽样产生点群
pcd = mesh.sample_points_poisson_disk(number_of_points=300000, init_factor=5)
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=50))
# 该方法可以使曲面上抽样产生的点均匀分布
o3d.visualization.draw_geometries([pcd])
o3d.io.write_point_cloud("sources\\group0\\model11.ply", pcd)