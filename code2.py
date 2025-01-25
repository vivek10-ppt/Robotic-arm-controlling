import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

cv2.namedWindow("RealSense", cv2.WINDOW_AUTOSIZE)

drawing = False
ix, iy = -1, -1

def draw_line(event, x, y, flags, param):
    global ix, iy, drawing

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            img = param.copy()
            cv2.line(img, (ix, iy), (x, y), (255, 0, 0), 5)
            cv2.imshow("RealSense", img)
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        cv2.line(param, (ix, iy), (x, y), (255, 0, 0), 5)
        cv2.imshow("RealSense", param)
        extract_depth_along_line((ix, iy), (x, y), param)

def extract_depth_along_line(pt1, pt2, img):
    global depth_frame

    x0, y0 = pt1
    x1, y1 = pt2
    line_coords = np.linspace((x0, y0), (x1, y1), 100).astype(np.int32)

    depths = []
    for x, y in line_coords:
        depth = depth_frame.get_distance(x, y)
        depths.append(depth)

    depths = np.array(depths)
    print("Depth values along the line:", depths)

    import matplotlib.pyplot as plt
    plt.plot(depths)
    plt.xlabel('Point Index')
    plt.ylabel('Depth (meters)')
    plt.title('Depth along the drawn line')
    plt.show()

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        cv2.imshow("RealSense", depth_colormap)
        cv2.setMouseCallback("RealSense", draw_line, param=depth_colormap)

        key = cv2.waitKey(1)
        if key == 27:
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()