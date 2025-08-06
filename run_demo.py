import cv2
import os
import argparse
import sys

# (This import block remains the same)
try:
    from lib import svo_cpp
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

        # --- FIX: Dynamically get image size before initializing the camera ---
        # 1. Load the first image to check its dimensions.
        first_img_path = os.path.join(self.dataset_path, "img", f"frame_{str(2).zfill(6)}_0.png")
        first_img = cv2.imread(first_img_path)
        if first_img is None:
            raise FileNotFoundError(f"Could not load the first image to determine size: {first_img_path}")
        
        height, width = first_img.shape[:2]
        print(f"Detected image size: {width}x{height}")

        # 2. Create a camera model with the CORRECT dimensions.
        #    NOTE: The focal lengths and principal point might need adjustment if you use a
        #    different dataset, but for this one, they are okay.
        print("Initializing camera and SVO...")
        cam = svo_cpp.PinholeCamera(width, height, 315.5, 315.5, width / 2.0, height / 2.0)

        # 3. Initialize the SVO handler with the correctly sized camera
        self.vo = svo_cpp.SVO(cam)
        self.vo.start()
        print("SVO handler created and started.")


    def run_from_folder(self):
        """
        Processes a sequence of images from the dataset folder.
        """
        # Loop through the same image range as the C++ test
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

# (The main() function and __name__ == "__main__" block remain the same)
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