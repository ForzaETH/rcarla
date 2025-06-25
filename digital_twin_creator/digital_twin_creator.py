import fire
import open3d as o3d
import pymeshlab
import yaml

from pcd_wrapper import PointCloudWrapper


def generate_mesh(file_path, base_path, samplenum):
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(base_path + file_path)
    print("Simplyfing point cloud...")
    ms.generate_simplified_point_cloud(samplenum=100000)
    print("Generating surface reconstruction using ball pivoting...")
    ms.generate_surface_reconstruction_ball_pivoting()
    print("Saving the mesh...")
    ms.save_current_mesh(base_path + "map.obj")

def get_n_m(use_statistical_outlier_removal, config):
    if use_statistical_outlier_removal:
        n = config.get('statistical_outlier_removal').get('number_of_neighbors', 100)
        m = config.get('statistical_outlier_removal').get('std_ratio', 0.1)
    else:
        n = config.get('outlier_removal').get('number_of_neighbors', 100)
        m = config.get('outlier_removal').get('search_radius', 1.0)
    return n, m


def main(file_path):
    with open("config.yaml", "r") as f:
        config = yaml.safe_load(f)
    force_cpu = config.get("gpu").get("force_cpu", False)
    use_statistical_outlier_removal = config.get("statistical_outlier_removal").get("use_statistical_outlier_removal", True)

    if use_statistical_outlier_removal:
        force_cpu = True  # Open3d does not reccomend using GPU for statistical outlier removal
    
    pcd_wrapper = PointCloudWrapper(file_path, force_cpu=force_cpu)

    first = True

    while True:
        if not first:
            pcd_wrapper.load_pointcloud()
            n = int(input("Enter the new number of neighbors (n): "))
            m = float(input("Enter the new radius (m): "))
        else:
            n, m = get_n_m(use_statistical_outlier_removal, config)
            first = False

        pcd_wrapper.filter_pointcloud(n, m, num_iters=config.get("gpu").get("num_iters", 20), use_statistical_outlier_removal=use_statistical_outlier_removal)

        print("Filtered point cloud:")

        if config.get("visualize"):
            pcd_wrapper.visualize()

        is_okay = input("Is this okay? (yes/no): ").strip().lower()

        if is_okay == "yes":
            pcd_wrapper.add_points_in_z_direction(config.get("num_new_points_z", 0))
            pcd_wrapper.save_pointcloud()
            
            if config.get("mesh").get("generate_mesh", True):
                generate_mesh("tmp.ply", pcd_wrapper.base_path, config.get("mesh").get("num_samples", 100000))
            break
        else:
            print("Let's try again with different parameters.")

if __name__ == "__main__":
    fire.Fire(main)