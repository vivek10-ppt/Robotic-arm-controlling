import pyrealsense2 as rs
import numpy as np
import cv2

class DepthCamera:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image

    def capture_still_photo(self, save_path):
        # Get a frame
        ret, depth, color = self.get_frame()
        if not ret:
            print("Failed to capture frame")
            return
        
        # Convert BGR to RGB for OpenCV compatibility
        color_rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)

        # Save the image
        cv2.imwrite(save_path, color_rgb)
        print(f"Saved photo to {save_path}")

    def release(self):
        self.pipeline.stop()


# Example usage:
if __name__ == "__main__":
    dc = DepthCamera()

    # Capture a still photo and save it
    save_path = "captured_photo.jpg"
    dc.capture_still_photo(save_path)

    dc.release()
