import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.interpolate import splprep, splev


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

    def release(self):
        self.pipeline.stop()


# List to store the points of the curved path
curve_points = []


def show_distance(event, x, y, flags, param):
    global curve_points

    if event == cv2.EVENT_LBUTTONDOWN:
        # Add the point to the list
        curve_points.append((x, y))


def draw_curve(img, points):
    if len(points) < 2:
        return img, []

    points = np.array(points)
    k = min(3, len(points) - 1)  # Ensure degree of spline is less than number of points
    tck, u = splprep([points[:, 0], points[:, 1]], s=0, k=k)
    u_new = np.linspace(u.min(), u.max(), 1000)
    x_new, y_new = splev(u_new, tck)

    for i in range(len(x_new) - 1):
        cv2.line(img, (int(x_new[i]), int(y_new[i])), (int(x_new[i + 1]), int(y_new[i + 1])), (0, 255, 0), 2)

    detailed_curve_points = [(float(x), float(y)) for x, y in zip(x_new, y_new)]
    return img, detailed_curve_points


dc = DepthCamera()
cv2.namedWindow("frame")
cv2.setMouseCallback("frame", show_distance)

while True:
    ret, depth, colour = dc.get_frame()
    if not ret:
        continue

    # Draw the curve on the image
    colour, detailed_curve_points = draw_curve(colour, curve_points)

    cv2.imshow("frame", colour)
    cv2.imshow("depth", depth)
    key = cv2.waitKey(1)
    if key == 27:
        break

dc.release()
cv2.destroyAllWindows()

# Print the detailed curve points
print("Detailed Curve Points:")
print(detailed_curve_points)
