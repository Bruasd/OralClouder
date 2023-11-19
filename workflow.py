from reg import *
import open3d as o3d
import copy
import json

class Processing:

    def __init__(self, num, prod, pref, plow):
        # 这些数据分别是：连接杆的数量、连接杆文件、参考模型和低精度模型的路径，在考虑加上连接杆点的位置
        self.num=num
        self.stick=prod
        self.pref=pref
        self.plow=plow
        self.voxel_size = 0.05

    def draw_registration_result(self, source, target):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        o3d.visualization.draw_geometries([source_temp, target_temp],
                                          zoom=0.4559,
                                          front=[0.6452, -0.3036, -0.7011],
                                          lookat=[1.9892, 2.0208, 1.8945],
                                          up=[-0.2779, -0.9482, 0.1556])

    def segment(self):
        n=1
        target = o3d.io.read_point_cloud(self.plow)
        o3d.io.write_point_cloud("data\\result\\seg.ply", target)

        while n<=self.num:
            target.clear()
            target = o3d.io.read_point_cloud("data\\result\\seg.ply")
            source = o3d.io.read_point_cloud(self.stick)
            source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target,self.voxel_size)
            result_ransac = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, self.voxel_size)
            # source.transform(result_ransac.transformation)
            # source1 = copy.deepcopy(source)
            # source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=50))
            # target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=50))
            result_icp = refine_registration(source_down, target_down, self.voxel_size, result_ransac)
            # result_part = copy.deepcopy(source)
            source.transform(result_icp.transformation)
            # o3d.visualization.draw_geometries([source])
            dists = target.compute_point_cloud_distance(source)
            dists = np.asarray(dists)
            seg= np.where(dists < 6)[0]
            part = target.select_by_index(seg)
            o3d.io.write_point_cloud("data\\seg\\part{}.ply".format(n), part)
            ind = np.where(dists > 0.5)[0]
            # nd = np.where(dists < 1)[0]
            # target1=copy.deepcopy(target)
            # source1 = target1.select_by_index(nd)
            target = target.select_by_index(ind)
            #
            part.paint_uniform_color([1, 0.706, 0])
            target.paint_uniform_color([0, 0.651, 0.929])
            o3d.visualization.draw_geometries([part,target], zoom=0.3412, front=[0.4257, -0.2125, -0.8795],
                                              lookat=[2.6172, 2.0475, 1.532], up=[-0.0694, -0.9768, 0.2024])
            o3d.io.write_point_cloud("data\\result\\seg.ply", target)
            source.clear()
            part.clear()
            n+=1
        o3d.visualization.draw_geometries([target], zoom=0.3412, front=[0.4257, -0.2125, -0.8795],
                                          lookat=[2.6172, 2.0475, 1.532], up=[-0.0694, -0.9768, 0.2024])

    def reg(self):
        source = o3d.io.read_point_cloud("data\\result\\seg.ply")
        # source = o3d.io.read_point_cloud("data\\source\\low.ply")
        target = o3d.io.read_point_cloud(self.pref)
        source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target, self.voxel_size)
        start = time.time()
        result_ransac = execute_global_registration(source_down, target_down,
                                                    source_fpfh, target_fpfh,
                                                    self.voxel_size)
        o3d.visualization.draw_geometries([source.transform(result_ransac.transformation),target], zoom=0.3412, front=[0.4257, -0.2125, -0.8795],
                                          lookat=[2.6172, 2.0475, 1.532], up=[-0.0694, -0.9768, 0.2024])
        print("ransac配准花费了： %.3f 秒.\n" % (time.time() - start))

        # 这是下面是由于对于新一批的文件，单纯的ransac表现不好，故增加一次ICP来使得结果更加准确

        n=1
        reg = o3d.geometry.PointCloud()
        start = time.time()
        while n <= self.num:
            source1 = o3d.io.read_point_cloud("data\\seg\\part{}.ply".format(n))
            result_icp = refine_registration(source1, target,
                                             self.voxel_size, result_ransac)
            source1.transform(result_icp.transformation)
            o3d.io.write_point_cloud("data\\result\\result_part{}.ply".format(n), source1)
            reg+=source1
            source1.clear()
            n+=1
        print("ICP配准花费了： %.3f 秒.\n" % (time.time() - start))
        o3d.io.write_point_cloud("data\\result\\result.ply", reg)
        o3d.visualization.draw_geometries([reg, target], zoom=0.3412,
                                          front=[0.4257, -0.2125, -0.8795],
                                          lookat=[2.6172, 2.0475, 1.532], up=[-0.0694, -0.9768, 0.2024])
        reg.clear()

    def positioning(self):
        refpod = "self.prod"
        positions=[]
        vectors=[]
        for i in range(self.num):
            voxel_size = 0.05  # 下采样的距离。当这个参数增大时，精确度会迅速下降，当这个参数减小时，耗时会急剧增加
            source = o3d.io.read_point_cloud(refpod)
            target = o3d.io.read_point_cloud("data\\result\\result_part{}.ply".format(i+1))

            source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target,
                                                                                                 voxel_size)
            result_ransac = execute_global_registration(source_down, target_down,
                                                        source_fpfh, target_fpfh,
                                                        voxel_size)
            source.transform(result_ransac.transformation)

            source1 = o3d.io.read_point_cloud(refpod)
            result_icp = refine_registrationseg(source1, target,
                                                voxel_size, result_ransac, 0.8)
            result_part = source1
            result_part.transform(result_icp.transformation)
            position = np.squeeze(np.array([0, 0, 0, 1]))
            vector = np.squeeze(np.array([0, 0, 1]))

            trans = np.squeeze(result_icp.transformation)
            rotation = trans[0:3, 0:3]
            x = np.dot(trans, position)
            v = np.dot(rotation, vector)

            a = x[0:3]
            m = np.array(a)
            positions.append(m)
            vectors.append(v)

        # 根据 positions 中的第二个元素进行排序，注意，此处应根据你自己文件的情况自行进行修改
        sorted_indices = sorted(range(len(positions)), key=lambda i: positions[i][1], reverse=True)

        # 生成排序后的字典
        data = {}
        for i, index in enumerate(sorted_indices, 1):
            data[f'position{i}'] = positions[index]
            data[f'vector{i}'] = vectors[index]

        # 将数据写入 JSON 文件
        json_filename = 'data/JsonData.json'
        with open(json_filename, 'w') as json_file:
            json.dump(data, json_file, indent=4)
            # 根据需要写入需要的其他数据，如h，r1等

workf=Processing(6,"data\\target\\highpart0.ply","data\\target\\ref.ply","data\\source\\low.ply")
start = time.time()
workf.segment()
workf.reg()
workf.positioning()
print("整个流程共花费了： %.3f 秒.\n" % (time.time() - start))



