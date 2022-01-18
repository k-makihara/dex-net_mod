
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
#import open3d_tutorial as o3dtut

#mesh = o3dtut.get_bunny_mesh()
#pcd = mesh.sample_points_poisson_disk(750)
pcd = o3d.io.read_point_cloud("/home/user/Downloads/IROS2022_dex-fractal-20211224T053701Z-001/IROS2022_dex-fractal/3dfractal_render_modern_opengl/3dfractal_render_modern_opengl/testply/00000_00.ply")
#o3d.visualization.draw_geometries([pcd])

#pcd.points = o3d.utility.Vector3dVector(cluster)
pcd.estimate_normals()


#alpha = 0.5
#print(f"alpha={alpha:.3f}")
#try:
#    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
#except Exception as e:
#    print(f"Cluster {array['ClusterID'][0]} error {str(e)}")
#    mesh = o3d.geometry.TriangleMesh()
#mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
#mesh.compute_vertex_normals()
#o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

#pcd.points = o3d.utility.Vector3dVector(surface)
#o3d.visualization.draw_geometries([pcd])


#alpha = 0.01
#print(f"alpha={alpha:.3f}")
#tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(pcd)
#o3d.visualization.draw_geometries([tetra_mesh])
#o3d.io.write_triangle_mesh("/home/user/Downloads/IROS2022_dex-fractal-20211224T053701Z-001/IROS2022_dex-fractal/3dfractal_render_modern_opengl/3dfractal_render_modern_opengl/testply/00000_00.obj",
#                               tetra_mesh)

#for i in range(1,1000):
#    alpha = i * 0.001
#    #print(alpha)
#    try:
#        m = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
#        m.compute_vertex_normals()
#        print(alpha)
#        o3d.visualization.draw_geometries([m], mesh_show_back_face=True)
#    except Exception as e:
#        continue

radii = [5, 2, 2, 4]
rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    pcd, o3d.utility.DoubleVector(radii))
o3d.visualization.draw_geometries([pcd, rec_mesh])
"""
print('run Poisson surface reconstruction')
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=4)
print(mesh)
o3d.visualization.draw_geometries([mesh],
                                  zoom=0.664,
                                  front=[-0.4761, -0.4698, -0.7434],
                                  lookat=[1.8900, 3.2596, 0.9284],
                                  up=[0.2304, -0.8825, 0.4101])


print('visualize densities')
densities = np.asarray(densities)
density_colors = plt.get_cmap('plasma')(
    (densities - densities.min()) / (densities.max() - densities.min()))
density_colors = density_colors[:, :3]
density_mesh = o3d.geometry.TriangleMesh()
density_mesh.vertices = mesh.vertices
density_mesh.triangles = mesh.triangles
density_mesh.triangle_normals = mesh.triangle_normals
density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
o3d.visualization.draw_geometries([density_mesh],
                                  zoom=0.664,
                                  front=[-0.4761, -0.4698, -0.7434],
                                  lookat=[1.8900, 3.2596, 0.9284],
                                  up=[0.2304, -0.8825, 0.4101])

print('remove low density vertices')
vertices_to_remove = densities < np.quantile(densities, 0.01)
mesh.remove_vertices_by_mask(vertices_to_remove)
print(mesh)
o3d.visualization.draw_geometries([mesh],
                                  zoom=0.664,
                                  front=[-0.4761, -0.4698, -0.7434],
                                  lookat=[1.8900, 3.2596, 0.9284],
                                  up=[0.2304, -0.8825, 0.4101])
"""