import subprocess
import numpy as np
import cv2
import mvsdk
import tflite_runtime.interpreter as tflite
from pycoral.utils import edgetpu
import threading
import time
from collections import deque
import platform

# Global variables for synchronization
testImage = cv2.imread("maxresdefault.jpg")
lidar_data_queue = deque()
lidar_data_lock = threading.Lock()
lidar_running = True

# List of class names
class_names = [
    'black_robot',
    'blue_display',
    'blue_robot',
    'note',
    'notes',
    'red_display',
    'red_robot',
    'speaker_blue',
    'speaker_red',
    'subwoofer_blue',
    'subwoofer_red'
]

# Placeholder calibration data (replace with actual calibration results)
camera_intrinsics = {
    'fx': 600.0,
    'fy': 600.0,
    'cx': 320.0,
    'cy': 240.0,
    'dist_coeffs': np.zeros(5)  # Assuming no distortion for simplicity
}

# Rotation matrix (LiDAR to camera)
rotation_matrix = np.eye(3)  # Replace with actual rotation matrix
# Translation vector (LiDAR to camera)
translation_vector = np.zeros(3)  # Replace with actual translation vector

# Function to capture images from the MindVision camera with timestamps
def capture_image():
    # Enumerate cameras
    DevList = mvsdk.CameraEnumerateDevice()
    nDev = len(DevList)
    if nDev < 1:
        print("No camera was found!")
        return None, None

    # Select the first camera
    DevInfo = DevList[0]

    # Open camera
    hCamera = 0
    try:
        hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    except mvsdk.CameraException as e:
        print("CameraInit Failed({}): {}".format(e.error_code, e.message))
        return None, None

    # Get camera capability
    cap = mvsdk.CameraGetCapability(hCamera)

    # Determine whether it's a monochrome camera
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

    # Set output format
    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

    # Set camera to continuous acquisition mode
    mvsdk.CameraSetTriggerMode(hCamera, 0)

    # Manual exposure, exposure time 30ms
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)

    # Start camera
    mvsdk.CameraPlay(hCamera)

    # Allocate frame buffer
    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (
        1 if monoCamera else 3)
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    # Capture frame
    try:
        timestamp = time.time()
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
        mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

        # Flip image if on Windows
        if platform.system() == "Windows":
            mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)

        # Convert to numpy array
        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth,
                               1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))

        # Convert monochrome to BGR
        if monoCamera:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    except mvsdk.CameraException as e:
        print("Camera exception: {}".format(e))
        frame = None
        timestamp = None
    finally:
        # Clean up
        mvsdk.CameraStop(hCamera)
        mvsdk.CameraUnInit(hCamera)
        mvsdk.CameraAlignFree(pFrameBuffer)

    return testImage, timestamp

# Function to collect LiDAR data continuously with timestamps
def collect_lidar_data():
    global lidar_running
    # Run the modified C++ LiDAR executable
    process = subprocess.Popen(['./get_lidar_data'], stdout=subprocess.PIPE, text=True)
    while lidar_running:
        line = process.stdout.readline()
        if not line:
            break
        try:
            # Parse the line
            # Format: timestamp_ms x y z intensity time ring
            tokens = line.strip().split()
            if len(tokens) != 7:
                continue  # Skip malformed lines

            timestamp_ms = time.time()
            x, y, z = map(float, tokens[1:4])
            intensity = float(tokens[4])
            point_time = float(tokens[5])
            ring = int(tokens[6])

            timestamp = timestamp_ms #/ 1000.0  # Convert ms to seconds

            # Store the point with timestamp
            with lidar_data_lock:
                lidar_data_queue.append({'point': [x, y, z], 'timestamp': timestamp})
                # Keep the queue size manageable
                if len(lidar_data_queue) > 100000:
                    lidar_data_queue.popleft()
        except ValueError:
            continue  # Handle any parsing errors

    process.terminate()

# Function to retrieve LiDAR data closest to a given timestamp
def get_lidar_data(timestamp):
    with lidar_data_lock:
        # Find LiDAR data within a time window around the timestamp
        time_window = 0.1  # 100 ms window
        relevant_points = [entry['point'] for entry in lidar_data_queue if abs(entry['timestamp'] - timestamp) <= time_window]
    return np.array(relevant_points)


# Function to fuse camera image and LiDAR data
def fuse_data(image, point_cloud):
    # Transform LiDAR points to camera coordinate frame
    transformed_points = transform_lidar_points(point_cloud, rotation_matrix, translation_vector)

    # Filter out points behind the camera
    valid_indices = transformed_points[:, 2] > 0
    transformed_points = transformed_points[valid_indices]

    # Project points onto image plane
    projected_points = project_points_to_image(transformed_points, camera_intrinsics)

    # Filter out points outside the image boundaries
    image_height, image_width = image.shape[:2]
    u, v = projected_points[:, 0], projected_points[:, 1]
    valid_indices = (u >= 0) & (u < image_width) & (v >= 0) & (v < image_height)
    projected_points = projected_points[valid_indices]
    transformed_points = transformed_points[valid_indices]

    # Create depth map (optional)
    #depth_map = np.zeros((image_height, image_width))
    #depth_map[v.astype(int), u.astype(int)] = transformed_points[:, 2]

    fused_data = {
        'image': image,
        #'depth_map': depth_map,
        'projected_points': projected_points,
        'transformed_points': transformed_points
    }
    return fused_data

def transform_lidar_points(point_cloud, rotation_matrix, translation_vector):
    # Convert point cloud to homogeneous coordinates
    ones = np.ones((point_cloud.shape[0], 1))
    points_homogeneous = np.hstack((point_cloud, ones))

    # Create transformation matrix
    transformation_matrix = np.hstack((rotation_matrix, translation_vector.reshape(-1, 1)))
    transformation_matrix = np.vstack((transformation_matrix, [0, 0, 0, 1]))

    # Transform points
    transformed_points = (transformation_matrix @ points_homogeneous.T).T[:, :3]
    return transformed_points

def project_points_to_image(points_3d, camera_intrinsics):
    fx = camera_intrinsics['fx']
    fy = camera_intrinsics['fy']
    cx = camera_intrinsics['cx']
    cy = camera_intrinsics['cy']

    x = points_3d[:, 0]
    y = points_3d[:, 1]
    z = points_3d[:, 2]

    u = (x * fx) / z + cx
    v = (y * fy) / z + cy

    projected_points = np.vstack((u, v)).T
    return projected_points

def bbox_to_pixel_coordinates(bbox, image_shape):
    # Assuming bbox is in normalized coordinates [x_min, y_min, x_max, y_max]
    height, width = image_shape[:2]
    x_min = int(bbox[0] * width)
    y_min = int(bbox[1] * height)
    x_max = int(bbox[2] * width)
    y_max = int(bbox[3] * height)
    return x_min, y_min, x_max, y_max

# Function to estimate object position
def estimate_object_position(detection, fused_data):
    bbox = detection['bbox']
    x_min, y_min, x_max, y_max = bbox_to_pixel_coordinates(bbox, fused_data['image'].shape)

    # Extract points within bounding box
    u, v = fused_data['projected_points'][:, 0], fused_data['projected_points'][:, 1]
    in_bbox = (u >= x_min) & (u <= x_max) & (v >= y_min) & (v <= y_max)

    points_in_bbox = fused_data['transformed_points'][in_bbox]

    if points_in_bbox.size > 0:
        # Estimate position (e.g., median to reduce effect of outliers)
        object_position = np.median(points_in_bbox, axis=0)
        detection['position'] = object_position.tolist()
    else:
        detection['position'] = None  # No LiDAR data within bounding box
    return detection

# Function to run object detection on the Coral TPU and include timestamps
def run_object_detection_with_predict(fused_data, timestamp):
    # Load the TFLite model (ensure the interpreter is initialized only once for efficiency)
    if not hasattr(run_object_detection_with_predict, 'model'):
        run_object_detection_with_predict.model = tflite.Interpreter(
            model_path='yolov9_model.tflite',
            experimental_delegates=[tflite.load_delegate('libedgetpu1-max')]
        )
        run_object_detection_with_predict.model.allocate_tensors()
        run_object_detection_with_predict.input_details = run_object_detection_with_predict.model.get_input_details()
        run_object_detection_with_predict.output_details = run_object_detection_with_predict.model.get_output_details()

    model = run_object_detection_with_predict.model
    input_details = run_object_detection_with_predict.input_details
    output_details = run_object_detection_with_predict.output_details

    # Preprocess image
    image = fused_data['image']
    input_shape = input_details[0]['shape']
    resized_image = cv2.resize(image, (input_shape[2], input_shape[1]))
    input_data = np.expand_dims(resized_image, axis=0).astype(np.float32)

    # Normalize image if required by the model
    if input_details[0]['dtype'] == np.float32:
        input_data = input_data / 255.0  # Normalize to [0,1] if required

    # Set tensor
    model.set_tensor(input_details[0]['index'], input_data)

    # Run inference
    model.invoke()

    # Get detection results
    # Adjust the indices based on your model's output
    boxes = model.get_tensor(output_details[0]['index'])  # Bounding box coordinates
    classes = model.get_tensor(output_details[1]['index'])  # Class IDs
    scores = model.get_tensor(output_details[2]['index'])  # Confidence scores
    num_detections = model.get_tensor(output_details[3]['index'])  # Number of detections

    num_detections = int(num_detections[0])
    boxes = boxes[0]
    classes = classes[0]
    scores = scores[0]

    # Process results
    detections = []
    for i in range(num_detections):
        if scores[i] > 0.5:  # Confidence threshold
            bbox = boxes[i].tolist()
            class_id = int(classes[i])
            class_name = class_names[class_id] if class_id < len(class_names) else 'Unknown'
            detections.append({
                'bbox': bbox,
                'class_id': class_id,
                'class_name': class_name,
                'score': float(scores[i]),
                'timestamp': timestamp  # Include timestamp
            })

    return detections

def main():
    # Start LiDAR data collection in a separate thread
    lidar_thread = threading.Thread(target=collect_lidar_data)
    lidar_thread.start()

    try:
        while True:
            # Capture image and timestamp
            image, image_timestamp = capture_image()
            if image is None:
                continue

            # Get LiDAR data closest to the image timestamp
            point_cloud = get_lidar_data(image_timestamp)

            if point_cloud.size == 0:
                print("No LiDAR data available for timestamp:", image_timestamp)
                continue

            # Fuse data
            fused_data = fuse_data(image, point_cloud)


            # Run object detection with the data timestamp
            detections = run_object_detection_with_predict(image, image_timestamp)

            # Calculate positions
            relative_positions = []
            for detection in detections:
                detection_with_position = estimate_object_position(detection, fused_data)
                relative_positions.append(detection_with_position)

            # Store detections with timestamps
            objects_with_timestamps = []
            for obj in relative_positions:
                objects_with_timestamps.append({
                    'class_id': obj['class_id'],
                    'class_name': obj['class_name'],
                    'position': obj['position'],
                    'score': obj['score'],
                    'timestamp': obj['timestamp']
                })

            # Combine all objects and timestamps into a data structure
            output_data = {
                'timestamp': image_timestamp,
                'objects': objects_with_timestamps
            }

            # Output the results
            print("Detected Objects at Timestamp {}:".format(image_timestamp))
            for obj in objects_with_timestamps:
                print(f"Class ID: {obj['class_id']}, Class Name: {obj['class_name']}, "
                      f"Position: {obj['position']}, Score: {obj['score']}, Timestamp: {obj['timestamp']}")

            # Sleep to maintain 40 FPS
            time.sleep(1.0 / 40)

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        # Signal the LiDAR thread to stop
        global lidar_running
        lidar_running = False
        lidar_thread.join()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()