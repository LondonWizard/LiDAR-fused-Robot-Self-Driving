import cv2
import numpy as np
import torch
import subprocess
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R
import time
from ultralytics import YOLO
# YOLOv9 model initialization

# Load a pretrained YOLOv8n model
model = YOLO("/home/orangepi/sam/linuxSDK_V2.1.0.43(240126)/demo/python_demo/model/yolov9_model/weights/best.pt")

# Robot's pose (x, y, z in inches and pitch, yaw, roll in degrees)
robot_pose = {
    'x': 0,
    'y': 0,
    'z': 0,
    'pitch': 0,
    'yaw': 90,
    'roll': 0
}

# Camera and LiDAR relative transformations
camera_coords = np.array([0, 0, 0])  # Replace with actual calibration values
camera_orientation = np.array([0, 0, 0])  # Replace with actual calibration values

lidar_coords = np.array([0, 0, 0])  # Replace with actual calibration values
lidar_orientation = np.array([0, 0, 0])  # Replace with actual calibration values

# Calculate transformations
camera_rotation = R.from_euler('xyz', camera_orientation, degrees=True).as_matrix()
lidar_rotation = R.from_euler('xyz', lidar_orientation, degrees=True).as_matrix()
translation = lidar_coords - camera_coords
rotation = lidar_rotation @ np.linalg.inv(camera_rotation)

# Function to fuse mask with LiDAR data
def fuse_mask_with_lidar(mask_points, lidar_points):
    transformed_mask_points = (mask_points @ rotation.T) + translation[:2]
    tree = KDTree(lidar_points[:, :2])
    distances, indices = tree.query(transformed_mask_points)
    return lidar_points[indices]

# Function to calculate object positions relative to the robot
def calculate_relative_position(lidar_point, robot_pose):
    rotation = R.from_euler('xyz', [robot_pose['pitch'], robot_pose['yaw'], robot_pose['roll']], degrees=True).as_matrix()
    translation = np.array([robot_pose['x'], robot_pose['y'], robot_pose['z']])
    relative_position = rotation @ lidar_point + translation
    return relative_position

# Main loop
def main():
    try:
        while True:
            # Call the photo.py script to capture an image
            subprocess.run(['python', 'photo.py'])
            time.sleep(1)
            # Load the saved image
            #frame = cv2.imread('snapshot.png')
            #if frame is None:
            # print("Failed to load the image.")
            # continue

            # Run YOLOv9 detection
            results = model(['snapshot.png'])
            for result in results:
                print(result)
            # Process each detected object
            for det in results.xyxy[0]:
                xmin, ymin, xmax, ymax, conf, cls = map(int, det)
                class_name = model.names[cls]
                print(f"Detected {class_name} with confidence {conf}")

                # Extract mask for detected object
                mask = results.masks[0].numpy()  # Assuming first mask corresponds to the object

                mask_points = np.argwhere(mask > 0)

                # Receive LiDAR data
                subprocess.run(['./lidarphoto', 'executable'])
                lidar_points = np.array(np.loadtxt('lidar_point_cloud.txt'))

                # Fuse mask with LiDAR data
                fused_points = fuse_mask_with_lidar(mask_points, lidar_points)

                # Calculate relative position
                for point in fused_points:
                    relative_position = calculate_relative_position(point, robot_pose)
                    print(f"Object {class_name} is at {relative_position} relative to the robot.")

    except KeyboardInterrupt:
        print("Program terminated.")

    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()