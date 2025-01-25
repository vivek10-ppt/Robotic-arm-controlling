import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.interpolate import splprep, splev
import serial
import time
from scipy.optimize import fsolve

# Configure the serial port
serial_port = serial.Serial('COM6', 115200, timeout=1)  # Adjust COM port as per your setup
time.sleep(2)  # Allow time for serial communication to establish
# for sending data to Arduino, these angle values are used to rotate motors in Arduino
def move_motors(steps1, steps2, steps3):
    command = f"<{steps1},{steps2},{steps3}>\n"
    serial_port.write(command.encode()) 
    print(f"Sent command: {command.strip()}")

    response = serial_port.readline().decode().strip()
    print(f"Arduino response: {response}")

# inverse kinematics equation
def ik(x, y, z):
    # Define the equations based on the inverse kinematics equations
    def equations(vars, P, Q, X, Z):
        A, B = vars
        eq1 = P * np.cos(A) + Q * np.sin(B) - X
        eq2 = P * np.sin(A) - Q * np.cos(B) - Z
        return [eq1, eq2]

    # Initialize the known values
    P = 80  # length of arm
    Q = 60  # length of arm2
    d = np.sqrt(x**2 + y**2)  # Replace with actual value

    # Initial guesses for A and B
    initial_guesses = [1.0, 1.0]  # Non-zero initial guesses to avoid trivial solutions

    # Solve the equations
    solution = fsolve(equations, initial_guesses, args=(P, Q, d, z))
    A, B = solution
    A = ((A * 180) / np.pi)//(360/1600)
    B = ((B * 180) / np.pi)//(360/1600)
    C = ((np.arctan(y / x) * 180) / np.pi)//(360/1600)
    return A, B, C

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

        # Create align object
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        if not depth_frame or not color_frame:
            return False, None, None

        return True, depth_image, color_image

    def release(self):
        self.pipeline.stop()

# List to store the points of the curved path and corners of the workbench
curve_points = []
corner_points = []
drawing_curve = False

# Coordinates to click for calibration (real-world coordinates)
real_world_points = [(0,100), (60, 100), (60, 0), (0, 0)]  # Add Z-coordinates if known

def show_distance(event, x, y, flags, param):
    global curve_points, corner_points, drawing_curve

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(corner_points) < 4:
            # Add the corner point to the list
            corner_points.append((x, y))
            print(f"Corner Points: {corner_points}")
        elif not drawing_curve:
            drawing_curve = True
            print("Start drawing curve inside the workbench")
        elif drawing_curve:
            # Check if the point is inside the workbench
            if cv2.pointPolygonTest(np.array(corner_points), (x, y), False) >= 0:
                curve_points.append((x, y))

def draw_curve(img, points):
    if len(points) < 2:
        return img, []

    points = np.array(points)
    k = min(3, len(points) - 1)  # Ensure degree of spline is less than number of points
    tck, u = splprep([points[:, 0], points[:, 1]], s=0, k=k)
    u_new = np.linspace(u.min(), u.max(), 25)
    x_new, y_new = splev(u_new, tck)

    for i in range(len(x_new) - 1):
        cv2.line(img, (int(x_new[i]), int(y_new[i])), (int(x_new[i + 1]), int(y_new[i + 1])), (0, 255, 0), 2)

    detailed_curve_points = [(float(x), float(y)) for x, y in zip(x_new, y_new)]
    return img, detailed_curve_points


def convert_coordinates_to_real_world(point, H):
    # Convert the point to a homogeneous coordinate
    homogeneous_point = np.array([point[0], point[1], 1.0], dtype=np.float32).reshape((3, 1))

    # Apply the homography matrix
    real_world_point = np.dot(H, homogeneous_point)

    # Normalize to convert from homogeneous to 2D coordinates
    real_world_point /= real_world_point[2]

    return (real_world_point[0][0], real_world_point[1][0])
def calculate_homography(corner_points, real_world_points):
    # Convert lists to numpy arrays and ensure they're 2D arrays of shape (N, 2)
    image_points = np.array(corner_points, dtype=np.float32).reshape(-1, 2)
    world_points = np.array(real_world_points, dtype=np.float32).reshape(-1, 2)

    # Check that you have at least 4 points
    if len(image_points) < 4 or len(world_points) < 4:
        raise ValueError("At least 4 points are required to calculate homography")

    # Calculate the homography matrix
    H, status = cv2.findHomography(image_points, world_points)

    return H

# Use this instead of calibrate_camera

def sent_data(array):
    prevsteps1, prevsteps2, prevsteps3 = 0, 0, 0
    delay = 3.5
    for i in array:
        step1, step2, step3 = ik(i[0], i[1], 0)
        input1 = step1 - prevsteps1
        input2 = step2 - prevsteps2
        input3 = step3 - prevsteps3
        move_motors(input1, input2, input3)
        time.sleep(delay)
        delay = 1
        prevsteps1, prevsteps2, prevsteps3 = step1, step2, step3

dc = DepthCamera()
cv2.namedWindow("frame")
cv2.setMouseCallback("frame", show_distance)

while True:
    ret, depth, colour = dc.get_frame()
    if not ret:
        continue

    # Normalize depth image for better visualization
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)

    # Draw the corner points on the image
    for point in corner_points:
        cv2.circle(colour, point, 5, (0, 0, 255), -1)

    # Draw the workbench boundary if 4 corners are selected
    if len(corner_points) == 4:
        cv2.polylines(colour, [np.array(corner_points)], isClosed=True, color=(255, 0, 0), thickness=2)

    # Draw the curve on the image if the drawing has started
    if drawing_curve:
        colour, detailed_curve_points = draw_curve(colour, curve_points)

    cv2.imshow("frame", colour)
    # cv2.imshow("depth", depth_colormap)

    key = cv2.waitKey(1)
    if key == 27:  # Exit on pressing 'Esc' key
        break

dc.release()
cv2.destroyAllWindows()

# Calibration process using selected corner points and known world coordinates
if len(corner_points) == 4:
    #camera_matrix, dist_coeffs, rvecs, tvecs = calibrate_camera(corner_points, real_world_points)
    H = calculate_homography(corner_points, real_world_points)
    # Convert detailed curve points to real-world coordinates using the camera calibration
    real_world_coordinates = [convert_coordinates_to_real_world(point, H) for point in detailed_curve_points]

    # Format the coordinates for easier manipulation and transfer to Arduino
    formatted_coordinates = [(float(coord[0]), float(coord[1])) for coord in real_world_coordinates]

    # Print the real-world coordinates in the desired format
    print("Real-world Coordinates of Detailed Curve Points:")
    print(formatted_coordinates)
    
    sent_data(real_world_coordinates)

# Close the serial port when done
serial_port.close()