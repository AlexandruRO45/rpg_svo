import cv2
import os
import argparse
import sys
import platform
import numpy as np

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
    def __init__(self, dataset_path: str, run_in_vio_mode: bool):
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
        self.is_vio_mode = run_in_vio_mode

        first_img_path = os.path.join(self.dataset_path, "img", f"frame_{str(2).zfill(6)}_0.png")
        first_img = cv2.imread(first_img_path, cv2.IMREAD_GRAYSCALE)
        if first_img is None:
            raise FileNotFoundError(f"Could not load the first image to determine size: {first_img_path}")
        
        height, width = first_img.shape[:2]
        print(f"Detected image size: {width}x{height}")
        self.height = height
        self.width = width

        # Define a dictionary with default parameters
        config = {
            "n_pyr_levels": 3, "core_n_kfs": 3, "map_scale": 1.0,
            "grid_size": 25, "init_min_disparity": 50.0, "init_min_tracked": 50,
            "init_min_inliers": 40, "klt_max_level": 4, "klt_min_level": 2,
            "reproj_thresh": 2.0, "poseoptim_thresh": 2.0, "poseoptim_num_iter": 10,
            "structureoptim_max_pts": 20, "structureoptim_num_iter": 5,
            "loba_thresh": 2.0, "loba_robust_huber_width": 1.0, "loba_num_iter": 0,
            "kfselect_mindist": 0.12, "triang_min_corner_score": 20.0,
            "subpix_n_iter": 10, "max_n_kfs": 0, "img_imu_delay": 0.0,
            "max_fts": 120, "quality_min_fts": 50, "quality_max_drop_fts": 40,
            
            # --- VIO/VO Flags & Parameters ---
            "use_imu": False,
            "imu_gyro_noise": 1.6968e-4, "imu_acc_noise": 2.0e-3,
            "imu_gyro_walk": 1.9393e-5, "imu_acc_walk": 3.0e-3,
        }
        
        # --- Apply VIO specific settings if enabled ---
        if self.is_vio_mode:
            print("VIO mode is ENABLED.")
            config["use_imu"] = True
        else:
            print("VIO mode is DISABLED (running in pure VO mode).")

        # --- Platform-specific performance tweaks ---
        if platform.machine() == "aarch64":
            print("Applying JETSON-specific performance configuration.")
            config.update({
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
        else:
            print("Applying default PC SVO configuration.")

        # --- Set the final configuration ---
        self.config = config
        set_svo_config(config)

        print("Initializing camera and SVO...")
        cam = PinholeCamera(width, height, 315.5, 315.5, width / 2.0, height / 2.0)

        self.vo = SVO(cam)
        self.vo.start()
        print("SVO handler created and started.")


    def run_from_folder(self):
        """
        Processes a sequence of images from the dataset folder.
        """
        last_timestamp = -1.0
        
        synthetic_gyro = np.array([0.01, 0.08, 0.0]) 
        synthetic_accel = np.array([0.1, 0.0, -9.81]) # accel + gravity
        
        for img_id in range(2, 188):
            filename = f"frame_{str(img_id).zfill(6)}_0.png"
            img_path = os.path.join(self.dataset_path, "img", filename)

            if img_id == 2:
                print(f"Reading first image from: {img_path}")

            img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                print(f"Error: Could not load image at {img_path}")
                continue
            
            mask = np.zeros_like(img, dtype=np.uint8)

            # 2. Simulate a moving object by drawing a white rectangle on the mask.
            # The rectangle will move from left to right as the video progresses.
            box_width = 150
            box_height = 200
            # Calculate the top-left corner of the box based on the frame number
            progress = (img_id - 2) / (188 - 2) # Progress from 0.0 to 1.0
            box_x = int(progress * (self.width - box_width))
            box_y = int((self.height - box_height) / 2)
            # Draw the filled white rectangle (value 255) on the mask
            cv2.rectangle(mask, (box_x, box_y), (box_x + box_width, box_y + box_height), 255, -1)
            
            current_timestamp = 0.01 * img_id
            print("here")
            # 1. Create a fake IMU measurement dictionary.
            if self.is_vio_mode and last_timestamp > 0:
                print("here 2")
                imu_dt = 0.005  # 200 Hz
                imu_timestamp = last_timestamp
                while imu_timestamp < current_timestamp:
                    # Add some noise to make it more realistic
                    gyro_noise = np.random.normal(0, self.config["imu_gyro_noise"], 3)
                    accel_noise = np.random.normal(0, self.config["imu_acc_noise"], 3)
                    
                    # Pass measurement to the backend
                    self.vo.addImuMeasurement(
                        synthetic_gyro + gyro_noise,
                        synthetic_accel + accel_noise,
                        imu_timestamp
                    )
                    imu_timestamp += imu_dt
            
            self.vo.addImage(img, current_timestamp, mask=mask)
            last_timestamp = current_timestamp
            
            # ----- DISPLAY REQUIRED: Comment the code if run headless -----
            
            # display_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            # red_overlay = np.zeros_like(display_img)
            # red_overlay[mask == 255] = (0, 0, 255)
            # display_img = cv2.addWeighted(display_img, 0.7, red_overlay, 0.3, 0)
            # cv2.imshow("SVO Input with Dynamic Mask", display_img)
            # if cv2.waitKey(1) == ord('q'):
            #     break
            
            # ----- END OF BLOCK -----
            
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
    parser.add_argument("--vio", action="store_true", help="Enable VIO mode by simulating IMU data.")
    args = parser.parse_args()


    try:
        benchmark = BenchmarkNode(dataset_path=os.path.abspath(args.dataset_dir), run_in_vio_mode=args.vio)
        benchmark.run_from_folder()
    except FileNotFoundError as e:
        print(f"\nERROR: {e}")
        print("Please update the 'dataset_dir' variable in the script to the correct path.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    print("\nBenchmarkNode finished.")


if __name__ == "__main__":
    main()