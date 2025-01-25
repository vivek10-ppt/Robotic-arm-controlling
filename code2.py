import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.interpolate import splprep, splev
import serial
import time
from scipy.optimize import fsolve

# Configure the serial port
#serial_port = serial.Serial('COM3', 9600, timeout=1)  # Adjust COM port as per your setup
time.sleep(2)  # Allow time for serial communication to establish
# for senting data to arduino ,these angles values are used to rotate motors in arduino
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
    P = 40  # length of arm
    Q = 30  # length of arm2
    d = np.sqrt(x**2 + y**2)  # Replace with actual value

    # Initial guesses for A and B
    initial_guesses = [0.1, 0.1]  # Non-zero initial guesses to avoid trivial solutions

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

# Example lengths in millimeters
lengths = [965, 715,945, 720]

def pixel_to_real_world(corners, lengths):
    def euclidean_distance(p1, p2):
        return np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
    
    # Calculate pixel distances for each side
    D1_pixels = euclidean_distance(corners[0], corners[1])
    D2_pixels = euclidean_distance(corners[1], corners[2])
    D3_pixels = euclidean_distance(corners[2], corners[3])
    D4_pixels = euclidean_distance(corners[3], corners[0])
    
    # Extract actual lengths from the input
    L1, L2, L3, L4 = lengths
    
    # Calculate conversion factors
    conv_factor1 = L1 / D1_pixels
    conv_factor2 = L2 / D2_pixels
    conv_factor3 = L3 / D3_pixels
    conv_factor4 = L4 / D4_pixels
    
    return [conv_factor1, conv_factor2, conv_factor3, conv_factor4], corners[3]  # corners[3] is the bottom-left corner

def convert_coordinates_to_real_world(pixel_point, origin, conversion_factors):
    x_pixel, y_pixel = pixel_point
    x0, y0 = origin
    conv_factor1, conv_factor2, conv_factor3, conv_factor4 = conversion_factors
    
    # Calculate the pixel distance from the origin
    dx_pixel = x_pixel - x0
    #y_pixel = y_pixel
    dy_pixel =  y0-y_pixel
    
    
    # Convert pixel distances to real-world distances using the conversion factors
    x_real = dx_pixel * conv_factor1
    y_real = dy_pixel * conv_factor4  # y_real uses conv_factor4 for vertical scaling
    
    return float(x_real), float(y_real)

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
def sent_data(array):
    prevsteps1,prevsteps2,prevsteps3=0,0,0
    delay=3500
    for i in array:
        step1,step2,step3=ik(i[0],i[1],0)
        input1=step1-prevsteps1
        input2=step2-prevsteps2
        input3=step3-prevsteps3
        move_motors(input1,input2,input3)
        time.sleep(delay)
        delay=1000
        prevsteps1,prevsteps2,prevsteps3=step1,step2,step3
def print_data(array):
    prevsteps1,prevsteps2,prevsteps3=0,0,0
    delay=3.5
    for i in array:
        step1,step2,step3=ik(i[0],i[1],0)
        input1=step1-prevsteps1
        input2=step2-prevsteps2
        input3=step3-prevsteps3
        print(input1,input2,input3)
        time.sleep(delay)
        delay=1
        prevsteps1,prevsteps2,prevsteps3=step1,step2,step3

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
    #cv2.imshow("depth", depth_colormap)
    
    key = cv2.waitKey(1)
    if key == 27:  # Exit on pressing 'Esc' key
        break

dc.release()
cv2.destroyAllWindows()

# Calculate conversion factors after the corner points are selected
#print(detailed_curve_points)
if len(corner_points) == 4:
    conversion_factors, origin = pixel_to_real_world(corner_points, lengths)
    #print("Conversion Factors:", conversion_factors)
    print("Origin:", origin)

    # Convert detailed curve points to real-world coordinates
    real_world_coordinates = [convert_coordinates_to_real_world(point, origin, conversion_factors) for point in detailed_curve_points]
    

    # Print the real-world coordinates
   # print("Real-world Coordinates of Detailed Curve Points:")
   # for coord in real_world_coordinates:
       # print(coord)
    #sent_data(real_world_coordinates) 
    print_data(real_world_coordinates)

serial_port.close()  # Close the serial port when done