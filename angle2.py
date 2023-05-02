from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import cv2
import os
import sys, getopt
import signal
from edge_impulse_linux.image import ImageImpulseRunner

connection_string = "udp:127.0.0.1:14550"
vehicle = connect(connection_string, wait_ready=True,timeout=60)

runner = None
show_camera = True
if (sys.platform == 'linux' and not os.environ.get('DISPLAY')):
    show_camera = False

centroid_x = None
centroid_y = None
e
w= 640
h= 480

drone_height = 30
fov_degrees = 62.2

model = os.path.expanduser("~/dronekit-vision-integration/examples/image/86.eim")

dir_path = os.path.dirname(os.path.realpath(__file__))
modelfile = os.path.join(dir_path, model)

def now():
    return round(time.time() * 1000)

def sigint_handler(sig, frame):
    print('Interrupted')
    if (runner):
        runner.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)

def arm_and_takeoff(target_altitude):
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(1)

    vehicle.simple_takeoff(target_altitude)

    while True:
        altitude = vehicle.location.global_relative_frame.alt
        if altitude >= target_altitude * 0.95:
            break
        time.sleep(1)

def land():
    vehicle.mode = VehicleMode("LAND")

def send_ned_velocity(vx, vy, vz, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)

    for _ in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def go_forward(distance, velocity):
    duration = int(abs(distance) / velocity)
    yaw = vehicle.attitude.yaw
    vx = math.cos(yaw) * velocity
    vy = math.sin(yaw) * velocity
    send_ned_velocity(vx, vy, 0, duration)
    time.sleep(duration)

def go_backward(distance, velocity):
    duration = int(abs(distance) / velocity)
    yaw = vehicle.attitude.yaw
    vx = -math.cos(yaw) * velocity
    vy = -math.sin(yaw) * velocity
    send_ned_velocity(vx, vy, 0, duration)
    time.sleep(duration)

def go_right(distance, velocity):
    duration = int(abs(distance) / velocity)
    yaw = vehicle.attitude.yaw
    vx = -math.sin(yaw) * velocity
    vy = math.cos(yaw) * velocity
    send_ned_velocity(vx, vy, 0, duration)
    time.sleep(duration)

def go_left(distance, velocity):
    duration = int(abs(distance) / velocity)
    yaw = vehicle.attitude.yaw
    vx = math.sin(yaw) * velocity
    vy = -math.cos(yaw) * velocity
    send_ned_velocity(vx, vy, 0, duration)
    time.sleep(duration)

def set_yaw_angle(angle, relative=False):
    if relative:
        angle = vehicle.attitude.yaw + math.radians(angle)

    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        math.degrees(angle),  # Target angle in degrees
        0,  # Yaw speed (not set)
        1 if relative else 0,  # 1 for relative angle, 0 for absolute angle
        0, 0, 0, 0)  # Added missing 0 for param7
    
    vehicle.send_mavlink(msg)
    vehicle.flush()

def observe():
    global centroid_x, centroid_y
    home_directory = os.path.expanduser("~")  # Get the home directory

    start_time = time.time()  # Add a start time for the timer
    with ImageImpulseRunner(modelfile) as runner:
        try:
            model_info = runner.init()
            labels = model_info['model_parameters']['labels']
            videoCaptureDeviceId = int(0)  # Set to 0 to use the default webcam
            camera = cv2.VideoCapture(videoCaptureDeviceId)
            ret = camera.read()[0]
            if ret:
                backendName = camera.getBackendName()
                w = camera.get(3)
                h = camera.get(4)
                camera.release()
            else:
                raise Exception("Couldn't initialize selected camera.")

            next_frame = 0
            for res, img in runner.classifier(videoCaptureDeviceId):
                if (next_frame > now()):
                    time.sleep((next_frame - now()) / 1000)

                # Break the loop after 8 seconds
                if time.time() - start_time > 8:
                    break

                if "bounding_boxes" in res["result"].keys():
                    for bb in res["result"]["bounding_boxes"]:
                        centroid_x = bb['x'] + bb['width'] // 2 - w // 2
                        centroid_y = bb['y'] + bb['height'] // 2 - h // 2
                        print('Centroid of', bb['label'], ': x =', centroid_x, ', y =', centroid_y)
                        
                        with open(os.path.join(home_directory, "dronex.txt"), "a") as dronex_file, \
                            open(os.path.join(home_directory, "droney.txt"), "a") as droney_file:
                            dronex_file.write(f"{centroid_x}\n")
                            droney_file.write(f"{centroid_y}\n")

                        img = cv2.rectangle(img, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 1)

                if (show_camera):
                    cv2.imshow('edgeimpulse', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                    if cv2.waitKey(1) == ord('q'):
                        break

                next_frame = now() + 1
        finally:
            if (runner):
                runner.stop()


def calculate_ground_distance_x(centroid_x, w, drone_height, fov_degrees):
    # Calculate degrees per pixel for the horizontal field of view
    degrees_per_pixel_x = fov_degrees / w

    # Calculate the pixel distance from the center of the screen (X-axis)
    pixel_distance_x = centroid_x - (w // 2)

    # Convert the pixel distance to an angle in degrees
    angle_to_point_x = degrees_per_pixel_x * pixel_distance_x

    # Convert the angle in degrees to radians
    angle_to_point_radians_x = math.radians(angle_to_point_x)

    # Calculate the ground distance using trigonometry (tangent function)
    ground_distance_x = math.tan(angle_to_point_radians_x) * drone_height

    return ground_distance_x

def calculate_ground_distance_y(centroid_y, h, drone_height, fov_degrees):
    # Calculate degrees per pixel for the vertical field of view
    degrees_per_pixel_y = fov_degrees * h / (640 * 480)  # assuming 4:3 aspect ratio

    # Calculate the pixel distance from the center of the screen (Y-axis)
    pixel_distance_y = centroid_y - (h // 2)

    # Convert the pixel distance to an angle in degrees
    angle_to_point_y = degrees_per_pixel_y * pixel_distance_y

    # Convert the angle in degrees to radians
    angle_to_point_radians_y = math.radians(angle_to_point_y)

    # Calculate the ground distance using trigonometry (tangent function)
    ground_distance_y = math.tan(angle_to_point_radians_y) * drone_height

    return ground_distance_y

def cartesian_to_polar(x, y):
    r = math.sqrt(x**2 + y**2)
    theta = math.atan2(y, x)
    theta_degrees = math.degrees(theta)  # Convert the angle to degrees
    return r, theta_degrees


def read_last_coordinate(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        if lines:
            return float(lines[-1].strip())
        else:
            return 0


try:



    distx = 0
    disty = 0

    arm_and_takeoff(5)

    observe()

    dronex_file = os.path.expanduser('~/dronex.txt')
    droney_file = os.path.expanduser('~/droney.txt')

    centroid_y = read_last_coordinate(dronex_file)
    centroid_x = read_last_coordinate(droney_file)

    distx = calculate_ground_distance_x(centroid_x, w, drone_height, fov_degrees)
    disty = calculate_ground_distance_y(centroid_y, h, drone_height, fov_degrees)

    distd = math.sqrt((distx * distx) + (disty * disty))

    r, theta_degrees = cartesian_to_polar(centroid_x, centroid_y)

    angle = 90 - theta_degrees

    set_yaw_angle(angle, relative=True)
    time.sleep(3)

    go_forward(distd, 5)

    time.sleep(2)

    land()



finally:                   
    vehicle.close()
