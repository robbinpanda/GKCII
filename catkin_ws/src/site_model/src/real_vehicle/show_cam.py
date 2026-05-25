from decouple import config
import time
from ._car_control_v3 import Client
from .core import car_cam
import cv2

window_name = "car_camera"
cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)

client = Client(password=config("car_PASSWORD"), ip=config("LOCAL_IP"))

camera_feed_one = car_cam.carCam(client)
camera_feed_one.start()
time.sleep(1)
i = 0
while True:
    image = camera_feed_one.get_image(timeout=1)
    if image is not None:
        cv2.imshow(window_name, image)
    else:
        print("waiting", i)
        i = i + 1
    if cv2.waitKey(1) == 27:
        cv2.destroyAllWindows()
        exit(0)
