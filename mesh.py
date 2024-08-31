import numpy as np
import open3d as o3d
from skimage import measure
from scipy.spatial import cKDTree

def load_and_process_point_cloud(filepath):
    # Load the point cloud data from CSV
    data = np.loadtxt(filepath, delimiter=',', skiprows=1)

    # Create a KD-tree for efficient nearest neighbor search
    tree = cKDTree(data)

    # Create a 3D grid
    x_min, y_min, z_min = data.min(axis=0)
    x_max, y_max, z_max = data.max(axis=0)
    grid_size = 100  # Adjust this value to change the resolution
    x, y, z = np.mgrid[x_min:x_max:grid_size*1j, y_min:y_max:grid_size*1j, z_min:z_max:grid_size*1j]
    grid_points = np.vstack((x.ravel(), y.ravel(), z.ravel())).T

    # Compute distances to nearest points
    distances, _ = tree.query(grid_points)
    distances = distances.reshape(x.shape)

    # Use Marching Cubes to create the mesh
    verts, faces, _, _ = measure.marching_cubes(distances, level=1)  # Adjust the level if needed

    # Scale and translate vertices back to original coordinate system
    verts[:, 0] = verts[:, 0] * (x_max - x_min) / grid_size + x_min
    verts[:, 1] = verts[:, 1] * (y_max - y_min) / grid_size + y_min
    verts[:, 2] = verts[:, 2] * (z_max - z_min) / grid_size + z_min

    # Create Open3D mesh
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(verts)
    mesh.triangles = o3d.utility.Vector3iVector(faces)

    # Compute vertex normals
    mesh.compute_vertex_normals()

    # Save mesh as OBJ
    output_path = 'pointcloud_mesh.obj'
    o3d.io.write_triangle_mesh(output_path, mesh)
    print(f"Mesh saved to {output_path}")

    # Create Open3D point cloud for visualization
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data[:, :3])

    # Visualize original point cloud and resulting mesh
    o3d.visualization.draw_geometries([pcd, mesh])

# Usage
filepath = "./point_cloud_random_orientation_with_floor.csv"
load_and_process_point_cloud(filepath)