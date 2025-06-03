import open3d as o3d
from alive_progress import alive_bar
import numpy as np


class PointCloudWrapper:
    def __init__(self, path, force_cpu=False):
        self.cuda = o3d.core.cuda.is_available() and not force_cpu
        last_slash = path.rfind("/")
        self.base_path = path[:last_slash + 1]
        self.file_name = path[last_slash + 1:]
        self.pcd = self.load_point_cloud()

    def is_empty(self, pcd):
        if self.cuda:
            print("Using CUDA for point cloud operations.")
            return pcd.point["positions"].shape[0] == 0
        else:
            return pcd.is_empty()

    def load_point_cloud(self):
        if self.cuda:
            pcd = o3d.t.io.read_point_cloud(self.base_path + self.file_name)
        else:
            pcd = o3d.io.read_point_cloud(self.base_path + self.file_name)

        if self.is_empty(pcd):
            raise ValueError("Point cloud is empty.")
        return pcd

    def filter_pointcloud(self, n, m, num_iters=20, use_statistical_outlier_removal=True):
        old_size = self.get_size()
        print(f"Filtering point cloud with {n} neighbors and radius {m}...")
        print(f"Original point cloud size: {old_size}")
        if self.cuda:
            self.pcd =  filter_gpu(self.pcd, n, m, num_iters)
        else:
            self.pcd =  filter_cpu(self.pcd, n, m, use_statistical_outlier_removal)
        new_size = self.get_size()
        perc = 100 * (1 - new_size / old_size)
        print(f"Filtered point cloud size: {new_size} ({perc:.2f}% reduction)")

    def get_size(self):
        if self.cuda:
            return self.pcd.point["positions"].shape[0]
        else:
            return len(self.pcd.points)

    def add_points_in_z_direction(self, l, step=0.005):
        new_points = []              
        if self.cuda:
            points = self.pcd.point["positions"].numpy()
        else:
            points = np.asarray(self.pcd.points)
        
        with alive_bar(len(points), title="Adding points in z direction") as bar:
            for j, point in enumerate(points):
                for i in range(1, l+1):
                    new_points.append(point + np.array([0, 0,  i * step]))
                    new_points.append(point - np.array([0, 0, i * step]))
                bar()
        new_points = np.vstack((points, new_points))

        if self.cuda:
            self.pcd.point["positions"] = o3d.core.Tensor(new_points, dtype=o3d.core.float64)
        else:
            self.pcd.points = o3d.utility.Vector3dVector(new_points)

    def save_pointcloud(self, file_name="tmp.ply"):
        if self.cuda:
            saved = o3d.t.io.write_point_cloud(self.base_path + file_name, self.pcd)
        else:
            saved = o3d.io.write_point_cloud(self.base_path + file_name, self.pcd)
        
        if not saved:
            raise ValueError("Failed to save point cloud.")

        print(f"Point cloud saved as {self.base_path + file_name}")

    def visualize(self):
        if self.cuda:
            viz_pcd = o3d.geometry.PointCloud()
            points = self.pcd.point["positions"].numpy()
            viz_pcd.points = o3d.utility.Vector3dVector(points)
        else:
            viz_pcd = self.pcd
        o3d.visualization.draw_geometries([viz_pcd], window_name="Point Cloud Visualization", width=800, height=600)
    


def filter_gpu(pcd, n, m, num_iters=20, use_statistical_outlier_removal=True):
    size = pcd.point["positions"].shape[0]
    size_per_iter = size // num_iters
    results = []

    for i in range(num_iters):
        print(f"Iteration {i + 1}/{num_iters}")
        pcd_thinned = o3d.t.geometry.PointCloud()
        pcd_thinned.point["positions"] = pcd.point["positions"][i * size_per_iter: (i + 1) * size_per_iter]
        pcd_gpu = pcd_thinned.cuda(0)
        result, _ = pcd_gpu.remove_radius_outliers(nb_points=n, search_radius=m)
        
        result_cpu = result.cpu()
        results.append(result_cpu.point["positions"])

        del pcd_gpu
        del result
        del result_cpu
        del pcd_thinned
        o3d.core.cuda.release_cache()
    
    pcd_result = o3d.t.geometry.PointCloud()
    pcd_result.point["positions"] = o3d.core.concatenate(results)
    return pcd_result

def filter_cpu(pcd, n, m, use_statistical_outlier_removal=True):
    if use_statistical_outlier_removal:
        pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=n, std_ratio=m, print_progress=True)
    else:
        _, ind = pcd.remove_radius_outlier(nb_points=n, radius=m, print_progress=True)
    filtered_pcd = pcd.select_by_index(ind)
    return filtered_pcd
