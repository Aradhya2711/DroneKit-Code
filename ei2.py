import cv2
import os
import sys, getopt
import signal
import time
from edge_impulse_linux.image import ImageImpulseRunner

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
                        centroid_x = bb['x'] + bb['width'] // 2
                        centroid_y = bb['y'] + bb['height'] // 2
                        print('Centroid of', bb['label'], ': x =', centroid_x, ', y =', centroid_y)
                        dronex_file.write(str(centroid_x) + "\n")
                        droney_file.write(str(centroid_y) + "\n")
                        img = cv2.rectangle(img, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 1)

                if (show_camera):
                    cv2.imshow('edgeimpulse', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                    if cv2.waitKey(1) == ord('q'):
                        break

                next_frame = now() + 1
                if (now() - start_time) > 8000:
                    break
    finally:
        if (runner):
            runner.stop()
        if (show_camera):
            cv2.destroyAllWindows()

            
