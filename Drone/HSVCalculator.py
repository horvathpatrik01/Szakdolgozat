# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
frameWidth = 480
frameHeight = 368
camera.resolution = (frameWidth, frameHeight)
camera.framerate = 30
camera.vflip=True
camera.rotation= 90
camera.hflip=True
rawCapture = PiRGBArray(camera, camera.resolution)


def func(a):
    pass

cv2.namedWindow("HSV")

cv2.resizeWindow("HSV", 640, 240)

cv2.createTrackbar("HUE Minimum", "HSV", 0, 179, func)
cv2.createTrackbar("HUE Maximum", "HSV", 179, 179, func)
cv2.createTrackbar("SAT Minimum", "HSV", 0, 255, func)
cv2.createTrackbar("SAT Maximum", "HSV", 255, 255, func)
cv2.createTrackbar("VALUE Minimum", "HSV", 0, 255, func)
cv2.createTrackbar("VALUE Maximum","HSV", 255, 255, func)

leave=False
for frame in camera.capture_continuous(rawCapture,format="bgr", use_video_port=True):

    img = frame.array

    img = cv2.resize(img, (frameWidth, frameHeight))

    rawCapture.truncate(0)

    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("HUE Minimum", "HSV")
    h_max = cv2.getTrackbarPos("HUE Maximum", "HSV")
    s_min = cv2.getTrackbarPos("SAT Minimum","HSV")
    s_max = cv2.getTrackbarPos("SAT Maximum", "HSV")
    v_min = cv2.getTrackbarPos("VALUE Minimum", "HSV")
    v_max = cv2.getTrackbarPos("VALUE Maximum", "HSV")

    lower = np.array([h_min, s_min, v_min])

    upper = np.array([h_max, s_max, v_max])

    mask = cv2.inRange(imgHsv, lower, upper)

    result = cv2.bitwise_and(img, img, mask=mask)

    print(f'[{h_min},{s_min},{v_min},{h_max},{s_max},{v_max}]')

    #mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    #hStack = np.hstack([img, mask, result])
    hStack = np.hstack([img, result])

    cv2.imshow('Horizontal Stacking', hStack)

    if (cv2.waitKey(1) & 0xFF == ord('q')) or leave:
        break
    while not leave:
        k=cv2.waitKey(1)
        if k & 0xFF == ord('q'):
            leave=True
        if k & 0xFF == ord('n'):
            break 
cv2.destroyAllWindows()
