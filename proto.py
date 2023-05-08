import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import os
import sys, getopt
import signal
import time
from edge_impulse_linux.image import ImageImpulseRunner
import math

connection_string = "udp:127.0.0.1:14550"
vehicle = connect(connection_string, wait_ready=True,timeout=60)

runner = None
show_camera = True
if (sys.platform == 'linux' and not os.environ.get('DISPLAY')):
    show_camera = False

def now():
    return round(time.time() * 1000)

def sigint_handler(sig, frame):
    print('Interrupted')
    if (runner):
        runner.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)

def observe():
    global runner

    model = os.path.expanduser("~/dronekit-vision-integration/examples/image/86.eim")
    dir_path = os.path.dirname(os.path.realpath(__file__))
    modelfile = os.path.join(dir_path, model)
    home_directory = os.path.expanduser("~")

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
            start_time = now()
            with open(os.path.join(home_directory, "dronex.txt"), "w") as dronex_file, \
                open(os.path.join(home_directory, "droney.txt"), "w") as droney_file:
                for res, img in runner.classifier(videoCaptureDeviceId):
                    if (next_frame > now()):
                        time.sleep((next_frame - now()) / 1000)

                    if "bounding_boxes" in res["result"].keys():
                        for bb in res["result"]["bounding_boxes"]:
                            centroid_x = bb['x'] + bb['width'] // 2 - 120
                            centroid_y = (bb['y'] + bb['height'] // 2 - 140) * -1

                            print('Centroid of', bb['label'], ': x =', centroid_x, ', y =', centroid_y)
                            dronex_file.write(str(centroid_x) + "\n")
                            droney_file.write(str(centroid_y) + "\n")
                            img = cv2.rectangle(img, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 1)

                    if (show_camera):
                        cv2.imshow('edgeimpulse', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                        if cv2.waitKey(1) == ord('q'):
                            break

                    next_frame = now() + 1
                    if (now() - start_time) > 15000:  # Run for 15 seconds (15000 ms)
                        break
        finally:
            if (runner):
                runner.stop()
            if (show_camera):
                cv2.destroyAllWindows()

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
        is_relative = 1
    else:
        is_relative = 0

    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        angle,
        0, 0, 0, 0, 0,
        is_relative)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def calculate_horizontal_vertical_fov(diagonal_fov_degrees, aspect_ratio):
    
    diagonal_fov_radians = math.radians(diagonal_fov_degrees)
    horizontal_fov_radians = 2 * math.atan(math.sqrt(1 / (1 + aspect_ratio ** 2)) * math.tan(diagonal_fov_radians / 2))
    vertical_fov_radians = 2 * math.atan(math.sqrt(1 / (1 + 1 / (aspect_ratio ** 2))) * math.tan(diagonal_fov_radians / 2))
    horizontal_fov_degrees = math.degrees(horizontal_fov_radians)
    vertical_fov_degrees = math.degrees(vertical_fov_radians)

    return horizontal_fov_degrees, vertical_fov_degrees

def calculate_ground_distance_y(centroid_y,gsdy):

    ground_distance_y = centroid_y*gsdy

    return ground_distance_y

def calculate_ground_distance_x(centroid_x,gsdx):

    ground_distance_x = centroid_x*gsdx

    return ground_distance_x

def cartesian_to_polar(x, y):
    r = math.sqrt(x**2 + y**2)
    theta = math.atan2(y, x)
    theta_degrees = math.degrees(theta)
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
    distd = 0

    height = 30
    dfov = 160
    aspect_ratio = 0.75  # For a 480x640 image

    hfov, vfov = calculate_horizontal_vertical_fov(dfov, aspect_ratio)

    gsdy = (2*height*math.tan(hfov/2))/480
    gsdx = (2*height*math.tan(vfov/2))/640

    arm_and_takeoff(10)
    time.sleep(3)

    centroid_x = 45
    centroid_y = 45

    #observe()
    '''
    with open(os.path.expanduser('~/dronex.txt'), "r") as file:
        lines = file.readlines()
        last_line = lines[-1]
        centroid_x = int(last_line.strip())

    with open(os.path.expanduser('~/droney.txt'), "r") as file:
        lines = file.readlines()
        last_line = lines[-1]
        centroid_y = int(last_line.strip())
    '''

    r, theta_degrees = cartesian_to_polar(centroid_x, centroid_y)
    angle = (90 - theta_degrees)//1
    


    distx = calculate_ground_distance_x(centroid_x,gsdx)
    disty = calculate_ground_distance_y(centroid_y,gsdy)
    distd = math.sqrt((distx * distx) + (disty * disty))
    
    angle = int(angle)
    print(angle)

    time.sleep(3)

    vehicle.mode = VehicleMode("GUIDED")
    set_yaw_angle(angle, relative=False)

    time.sleep(3)

    go_forward(distd,2)

    time.sleep(5)

    land()

finally:                   
    vehicle.close()
