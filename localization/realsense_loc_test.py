import pyrealsense2 as rs
import numpy as np
import localizer
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

localiser = localizer.Localizer("../calibration/calibration-realsense-1920-1080.yml")

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        field_map = localiser.estimateLocation(color_image, True)
        field_map.showWindow()
        cv2.imshow("Original", color_image)

        if cv2.waitKey(1) != -1:
            break
    cv2.destroyAllWindows()
finally:

    # Stop streaming
    pipeline.stop()