import cv2
import os
import argparse
import sys
import platform

try:
    from svo_cpp import *
except ImportError:
    try:
        project_root = os.path.dirname(os.path.abspath(__file__))
        lib_path = os.path.join(project_root, 'lib')
        sys.path.insert(0, lib_path)
        import svo_cpp
    except ImportError:
        print("Error: Could not import the compiled SVO module.")
        print("Please make sure you have compiled the project and are running this script from the project root.")
        exit(1)


class BenchmarkNode:
    """
    A Python adaptation of the C++ BenchmarkNode test class.
    """
    def __init__(self, dataset_path: str):
        """
        Initializes the camera based on the actual image size and then starts
        the SVO visual odometry handler.
        """
        if not os.path.exists(dataset_path):
            raise FileNotFoundError(
                f"Dataset not found at '{dataset_path}'. "
                "Please download the 'sin2_tex2_h1_v8_d' dataset and provide the correct path."
            )
        self.dataset_path = dataset_path

        first_img_path = os.path.join(self.dataset_path, "img", f"frame_{str(2).zfill(6)}_0.png")
        first_img = cv2.imread(first_img_path, cv2.IMREAD_GRAYSCALE)
        if first_img is None:
            raise FileNotFoundError(f"Could not load the first image to determine size: {first_img_path}")
        
        height, width = first_img.shape[:2]
        print(f"Detected image size: {width}x{height}")

        # Define a dictionary with default parameters
        default_config = {
            "n_pyr_levels": 3,
            "use_imu": False,
            "core_n_kfs": 3,
            "map_scale": 1.0,
            "grid_size": 25,
            "init_min_disparity": 50.0,
            "init_min_tracked": 50,
            "init_min_inliers": 40,
            "klt_max_level": 4,
            "klt_min_level": 2,
            "reproj_thresh": 2.0,
            "poseoptim_thresh": 2.0,
            "poseoptim_num_iter": 10,
            "structureoptim_max_pts": 20,
            "structureoptim_num_iter": 5,
            "loba_thresh": 2.0,
            "loba_robust_huber_width": 1.0,
            "loba_num_iter": 0,
            "kfselect_mindist": 0.12,
            "triang_min_corner_score": 20.0,
            "subpix_n_iter": 10,
            "max_n_kfs": 0,
            "img_imu_delay": 0.0,
            "max_fts": 120,
            "quality_min_fts": 50,
            "quality_max_drop_fts": 40
        }

        # Create your Jetson-specific config by copying the default and modifying it.
        arm64_config = default_config.copy()
        arm64_config.update({
            # # --- Robustness Parameters ---
            # "reproj_thresh": 4.0,
            # "poseoptim_thresh": 4.0,
            # "quality_min_fts": 30,
            # "quality_max_drop_fts": 60,
            
            # # --- Drone Motion Parameters ---
            # "grid_size": 30,
            
            # # --- Initialization Parameters ---
            # "init_min_disparity": 25.0,
            
            # # --- Performance Parameters ---
            # "max_fts": 100,
            # "poseoptim_num_iter": 8,
            # "subpix_n_iter": 5
        })

        # Check the machine's architecture and apply the correct config
        if platform.machine() == "aarch64":
            print("Applying JETSON-specific SVO configuration.")
            set_svo_config(arm64_config)
        else:
            print("Applying default PC SVO configuration.")
            set_svo_config(default_config)

        print("Initializing camera and SVO...")
        cam = PinholeCamera(width, height, 315.5, 315.5, width / 2.0, height / 2.0)

        self.vo = SVO(cam)
        self.vo.start()
        print("SVO handler created and started.")


    def run_from_folder(self):
        """
        Processes a sequence of images from the dataset folder.
        """
        for img_id in range(2, 188):
            filename = f"frame_{str(img_id).zfill(6)}_0.png"
            img_path = os.path.join(self.dataset_path, "img", filename)

            if img_id == 2:
                print(f"Reading first image from: {img_path}")

            img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                print(f"Error: Could not load image at {img_path}")
                continue
            self.vo.addImage(img, 0.01 * img_id)
            
            last_frame = self.vo.lastFrame()
            if last_frame:
                pose = last_frame.T_f_w.inverse()
                pos = pose.translation()

                print(
                    f"Frame-Id: {last_frame.id_} \t"
                    f"#Features: {self.vo.lastNumObservations()} \t"
                    f"Proc. Time: {self.vo.lastProcessingTime() * 1000:.2f}ms \t"
                    f"Pos: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
                )

def main():
    """Main execution function."""
    parser = argparse.ArgumentParser(description="Run the SVO wrapper on a directory of images.")
    parser.add_argument("dataset_dir", help="Path to the root directory of the dataset (e.g., sin2_tex2_h1_v8_d).")
    args = parser.parse_args()


    try:
        benchmark = BenchmarkNode(dataset_path=os.path.abspath(args.dataset_dir))
        benchmark.run_from_folder()
    except FileNotFoundError as e:
        print(f"\nERROR: {e}")
        print("Please update the 'dataset_dir' variable in the script to the correct path.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    print("\nBenchmarkNode finished.")


if __name__ == "__main__":
    main()