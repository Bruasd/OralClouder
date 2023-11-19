import open3d as o3d
import numpy as np
import copy
import time
from vis import draw_registration_result


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)
    # radius_normal = voxel_size * 2  #0.1
    radius_normal = 0.2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=50))#30
    # radius_feature = voxel_size * 5 #0.25
    radius_feature = 0.5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=150))#100
    return pcd_down, pcd_fpfh

def prepare_dataset(source,target,voxel_size):
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    # distance_threshold = voxel_size * 1.5
    # distance_threshold = voxel_size * 15
    distance_threshold =1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result =o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        # o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            # o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
            #     distance_threshold),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnNormal(0.9)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(max_iteration=1000000, confidence=0.99))
    if result.fitness > 0.7:
        print("ransac fitness is {}".format(result.fitness))
        return result
    else:
        print("error ransac fitness is {}".format(result.fitness))
        return execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size)

def execute_global_registration1(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    # distance_threshold = voxel_size * 1.5
    # distance_threshold = voxel_size * 15
    distance_threshold =1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result =o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        # o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            # o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
            #     distance_threshold),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnNormal(0.9)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(max_iteration=1000000, confidence=0.99))
    if result.fitness > 0.8:
        print("ransac fitness is {}".format(result.fitness))
        return result
    else:
        print("error ransac fitness is {}".format(result.fitness))
        return execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size)

def execute_global_registrationseg(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size,n:float):
    # distance_threshold = voxel_size * 1.5
    # distance_threshold = voxel_size * 15
    distance_threshold =1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result =o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        # o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            # o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
            #     distance_threshold),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnNormal(0.9)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(max_iteration=1000000, confidence=0.99))
    if result.fitness > n:
        print("ransac fitness is {}".format(result.fitness))
        return result
    else:
        print("error ransac fitness is {}".format(result.fitness))
        return execute_global_registrationseg(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size,n)

def refine_registration(source, target, voxel_size,result_ransac):  #icp
    distance_threshold = voxel_size * 6
    # distance_threshold = 0.1
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100000))
    print(result)
    return result

def refine_registration2(source, target, voxel_size,result_ransac):  #icp
    distance_threshold = voxel_size * 8
    # distance_threshold = voxel_size * 4
    # distance_threshold = 0.1
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.0000005,relative_rmse=0.0000003,max_iteration=100000))
        # o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100000))
    print(result)
    return result
# positioning_icp


def refine_registration3(source, target, voxel_size,result_ransac):  #icp
    distance_threshold = voxel_size * 8
    # distance_threshold = voxel_size * 4
    # distance_threshold = 0.1
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.0000005,relative_rmse=0.0000003,max_iteration=100000))
        # o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100000))
    if result.fitness > 0.95:
        print("icp fitness is {}".format(result.fitness))
        return result
    else:
        print("error icp fitness is {}".format(result.fitness))
        source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target, voxel_size)
        result_ransac=execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size)
        return refine_registration3(source, target, voxel_size,result_ransac)

def refine_registration4(source, target, voxel_size,result_ransac):  #icp
    distance_threshold = voxel_size * 8
    # distance_threshold = voxel_size * 4
    # distance_threshold = 0.1
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.0000005,relative_rmse=0.0000003,max_iteration=100000))
        # o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100000))
    if result.fitness > 0.75:
        print("icp fitness is {}".format(result.fitness))
        return result
    else:
        print("error icp fitness is {}".format(result.fitness))
        source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target, voxel_size)
        result_ransac=execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size)
        return refine_registration3(source, target, voxel_size,result_ransac)

def refine_registrationseg(source, target, voxel_size,result_ransac,n:float):  #icp
    distance_threshold = voxel_size * 8
    # distance_threshold = voxel_size * 4
    # distance_threshold = 0.1
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.0000005,relative_rmse=0.0000003,max_iteration=100000))
        # o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100000))
    if result.fitness > n:
        print("icp fitness is {}".format(result.fitness))
        return result
    else:
        print("error icp fitness is {}".format(result.fitness))
        source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target, voxel_size)
        result_ransac=execute_global_registrationseg(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size,n)
        return refine_registrationseg(source, target, voxel_size,result_ransac,n)

def refine_registrationreg(source, target, voxel_size,result_ransac,n:float):  #icp
    distance_threshold = voxel_size * 8
    # distance_threshold = voxel_size * 4
    # distance_threshold = 0.1
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.0000005,relative_rmse=0.0000003,max_iteration=100000))
        # o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100000))
    if result.fitness > n:
        print("icp fitness is {}".format(result.fitness))
        return result
    else:
        print("error icp fitness is {}".format(result.fitness))
        return refine_registrationreg(source, target, voxel_size,result_ransac,n)

if __name__ == '__main__':
    voxel_size = 0.05  # 下采样的距离。当这个参数增大时，精确度会迅速下降，当这个参数减小时，耗时会急剧增加
    # source = o3d.io.read_point_cloud("data\\target\\highpart_processed.ply")
    # target = o3d.io.read_point_cloud("data\\source\\lowS.ply")
    source = o3d.io.read_point_cloud("data\\target\\highpart0.ply")
    target = o3d.io.read_point_cloud("data\\source\\low.ply")
    # initial_transform=[[ 1,0,0,0] ,[0,1,0,0] ,[0,0,1,0], [ 0,0,0,0.95]]
    copy=copy.deepcopy(source)
    # source.transform(initial_transform)
    source, target, source_down, target_down, source_fpfh, target_fpfh =prepare_dataset(source,target,voxel_size)
    start = time.time()
    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
    print("ransac配准花费了： %.3f 秒.\n" % (time.time() - start))
    print(result_ransac)
    print(result_ransac.transformation)
    source.transform(result_ransac.transformation)
    draw_registration_result(source, target, result_ransac.transformation)

    # source1 = o3d.io.read_point_cloud("data\\seg\\part4.ply")
    # source1.transform(result_ransac.transformation)
    # draw_registration_result(source1, target, result_ransac.transformation)
    start = time.time()
    # source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=50))
    # target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=50))
    result_icp = refine_registration(source_down, target_down,
                                     voxel_size,result_ransac)
    print("ICP配准花费了： %.3f 秒.\n" % (time.time() - start))
    copy.transform(result_icp.transformation)
    draw_registration_result(copy, target, result_ransac.transformation)
    # o3d.visualization.draw_geometries(
    #     [copy.paint_uniform_color([0, 0.651, 0.929]), target.paint_uniform_color([0.9, 0.21, 0.19])], zoom=0.7412,
    #     front=[0.0257, 3.3125, -0.4795],
    #     lookat=[2.6172, 5.0475, -5.532], up=[-0.8694, -0.9768, 10.8024])
    print(result_ransac.transformation)
    print(result_icp)
    print(result_icp.transformation)
    # result_part=source
    # result_part.transform(result_icp.transformation)
    # o3d.io.write_point_cloud("data\\result\\result_part1.ply", result_part)
    # draw_registration_result(source, target, result_icp.transformation)