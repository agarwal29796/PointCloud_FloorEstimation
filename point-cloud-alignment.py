import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
    

def reduce_pcd_size(pcd, reduction_factor=0.5):
    # Ensure reduction factor is valid
    reduction_factor = max(0, min(1, reduction_factor))
    
    # Get total number of points
    num_points = len(pcd.points)
    
    # Calculate number of points to keep
    num_points_to_keep = int(num_points * reduction_factor)
    
    # Randomly select indices
    selected_indices = np.random.choice(num_points, num_points_to_keep, replace=False)
    
    # Create new point cloud with selected points
    reduced_pcd = pcd.select_by_index(selected_indices)
    
    return reduced_pcd


def add_noise_to_pcd(pcd, noise_std=1):
    points = np.asarray(pcd.points)
    noise = np.random.normal(0, noise_std, points.shape)    
    noisy_points = points + noise
    
    # Create new point cloud with noisy points
    noisy_pcd = o3d.geometry.PointCloud()
    noisy_pcd.points = o3d.utility.Vector3dVector(noisy_points)
    
    return noisy_pcd

def load_point_cloud(file_path):
    points = np.loadtxt(file_path, delimiter=',',skiprows=1)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def detect_floor_pca(pcd):
    # Compute the covariance matrix
    points = np.asarray(pcd.points)
    mean = np.mean(points, axis=0)
    centered_points = points - mean
    cov = np.cov(centered_points.T)
    
    # Perform eigendecomposition
    eigenvalues, eigenvectors = np.linalg.eig(cov)
    
    # Sort eigenvectors by eigenvalues in descending order
    sort_indices = np.argsort(eigenvalues)[::-1]
    eigenvectors = eigenvectors[:, sort_indices]
    
    # The floor normal is the eigenvector with the smallest eigenvalue
    floor_normal = eigenvectors[:, 2]
    
    # Ensure the normal points upwards
    if floor_normal[1] < 0:
        floor_normal = -floor_normal
    
    # Calculate the distance to the origin (d in the plane equation ax + by + cz + d = 0)
    d = -np.dot(floor_normal, mean)
    
    return np.append(floor_normal, d)


# This is an alternative approach to PCA
def detect_floor(pcd, distance_threshold=0.01, ransac_n=3, num_iterations=1000):
    plane_model, inliers = pcd.segment_plane(distance_threshold, ransac_n, num_iterations)
    return plane_model

def align_to_yz(pcd, floor_model):
    floor_normal = floor_model[:3]
    d = floor_model[3]

    # Calculate rotation
    rotation = Rotation.align_vectors([[1, 0, 0]], [floor_normal])[0]
    rotation_matrix = rotation.as_matrix()

    # Combine rotation and translation
    transformation = np.eye(4)
    transformation[:3, :3] = rotation_matrix

    # Apply transformation
    pcd_aligned = pcd.transform(transformation)

    # Translate centroid to origin
    centroid = pcd_aligned.get_center()
    pcd_aligned.translate(-centroid)

    # Update transformation matrix with the translation
    transformation[:3, 3] = -centroid


    # Shift in x direction by xmin
    points = np.asarray(pcd_aligned.points)
    xmin = np.min(points[:, 0])
    shift = np.array([xmin, 0, 0])
    pcd_aligned.translate(-shift)

    # Update transformation matrix with the shift
    transformation[:3, 3] -= shift
    
    return pcd_aligned, transformation

def create_origin_and_axis(size=1.0):
    origin = o3d.geometry.TriangleMesh.create_sphere(radius=0.02*size)
    origin.paint_uniform_color([1, 1, 1])  # White color for origin
    
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=[0, 0, 0])
    
    return [origin, axis]


def visualize_point_clouds(pcd_original, pcd_aligned, floor_model, transformation):
    # Create origin and axis
    origin_axis = create_origin_and_axis(size=30)
        
    # Visualize original point cloud with detected plane
    original_geometries = [pcd_original] + origin_axis
    o3d.visualization.draw_geometries(original_geometries, window_name="Original Point Cloud with Detected Plane")
        
    # Visualize aligned point cloud with transformed plane
    aligned_geometries = [pcd_aligned] + origin_axis
    o3d.visualization.draw_geometries(aligned_geometries, window_name="Aligned Point Cloud with Transformed Plane")

def main(input_file, output_file):
    # Load point cloud
    pcd = load_point_cloud(input_file)
    pcd = add_noise_to_pcd(pcd)
    pcd = reduce_pcd_size(pcd)

    print("Total Points: ",len(pcd.points))
    pcd_original = o3d.geometry.PointCloud(pcd)

    # Detect floor
    # floor_model = detect_floor(pcd)
    floor_model = detect_floor_pca(pcd)
    
    # Align point cloud
    pcd_aligned, transformation = align_to_yz(pcd, floor_model)
    
    # Visualize point clouds
    visualize_point_clouds(pcd_original, pcd_aligned, floor_model, transformation)

if __name__ == "__main__":
    input_file = "point_cloud_random_orientation_with_floor.csv"
    output_file = "aligned_point_cloud.csv"
    main(input_file, output_file)