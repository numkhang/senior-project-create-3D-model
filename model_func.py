import numpy as np
import open3d as o3d
import os

def process_pc(res_path,voxel,nb_points,radius):
    file_path = os.path.join(res_path,"pointcloud.ply")
    pcd = o3d.io.read_point_cloud(f'{file_path}')
    pcd = pcd.voxel_down_sample(voxel_size=voxel)
    cl, ind = pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)
    inlier_cloud = pcd.select_by_index(ind)
    inlier_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
    return inlier_cloud


def show_pc(res_path,voxel,nb_points,radius):
    inlier_cloud = process_pc(res_path,voxel,nb_points,radius)
    o3d.visualization.draw_geometries([inlier_cloud],width = 700,height=700,point_show_normal=False)
    return inlier_cloud

def create_3d_alpha(res_path,voxel,nb_points,radius,alpha):
    inlier_cloud = process_pc(res_path,voxel,nb_points,radius)
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(inlier_cloud, alpha)
    mesh = mesh.filter_smooth_laplacian(number_of_iterations=10)
    mesh.remove_degenerate_triangles()
    mesh_path = os.path.join(res_path,"mesh.ply")
    o3d.io.write_triangle_mesh(mesh_path,mesh)    
    o3d.visualization.draw_geometries([mesh],width = 700,height=700)
    return mesh

def create_3d_poisson(res_path,voxel,nb_points,radius,depth,width,densities_threshold):
    inlier_cloud = process_pc(res_path,voxel,nb_points,radius)
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(inlier_cloud, depth=depth,width=width ,linear_fit =True)
    vertices_to_remove = densities < np.quantile(densities, densities_threshold)
    mesh.remove_vertices_by_mask(vertices_to_remove)
    mesh = mesh.filter_smooth_laplacian(number_of_iterations=10)
    mesh.remove_degenerate_triangles()
    mesh_path = os.path.join(res_path,"mesh.ply")
    o3d.io.write_triangle_mesh(mesh_path,mesh)    
    o3d.visualization.draw_geometries([mesh],width = 700,height=700)
    return mesh

