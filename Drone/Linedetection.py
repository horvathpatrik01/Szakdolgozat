# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import time
import Drone


### Constant Parameters
# variable for exiting the code
leave=False
# Image size
frameWidth = 480
frameHeight = 368
# hue, saturation values from colorpicker
HSV = [0,105,170,179,255,255]

# Split camera img into n equal section
sections = 3
# Threshold for the amount of white pixels needed in one section
threshold = 0.2

 # lower value equals higher sensitivity
sens = 3

# yaw control values
yawvalues = [-25, -15, 0, 15, 25]

# Masks the incoming rgb image based on the given HSV values
def masking(img):
    # convert rgb into hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    hsvlowervalues = np.array([HSV[0], HSV[1], HSV[2]])

    hsvuppervalues = np.array([HSV[3], HSV[4], HSV[5]])
    #mask the hsv image between the given lower and upper values
    mask = cv2.inRange(hsv, hsvlowervalues, hsvuppervalues)
    return mask

# Find the contours of the line
def getContours(imgThres, img):
    # center of the line on the x axes
    centerx = 0

    contours, hieracrhy = cv2.findContours(imgThres, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) != 0:
        # get the contour with the biggest area
        biggest = max(contours, key=cv2.contourArea)
        # To get the center of the contours simplify the area with a rectangle
        x, y, w, h = cv2.boundingRect(biggest)

        centerx = x + w // 2
        # center of the line on the y axes
        centery = y + h // 2
        # Drawings for testing purposes
        # draw line contours on the img
        cv2.drawContours(img, biggest, -1, (0, 255, 255), 7)
        # draw the center of the line
        cv2.circle(img, (centerx,centery), 10, (0, 255, 0), cv2.FILLED)

    return centerx

# Determines the line position under the drone
def getLinePosition(imgThres, sections):
    # horizontally split the thresholded image
    imgs = np.hsplit(imgThres, sections)
    # total pixelcount of one section
    totalPixels = (img.shape[1] // sections) * img.shape[0]

    lineposArray = []

    for x, im in enumerate(imgs):
        # Counts the how many pixels are white on the splitted image
        pixelCount = cv2.countNonZero(im)

        if pixelCount > threshold * totalPixels:
            # if there are more white pixels
            lineposArray.append(1)

        else:

            lineposArray.append(0)

    print(lineposArray)

    return lineposArray

# Calculates and sends the vy and yaw values to the drone
def getErrorValues(lineposArray, centerx):
    global leave
    yawerror=0
    # calculate the error
    error = (centerx - frameWidth // 2) // sens

    error = int(np.clip(error, -10, 10))

    if (2 > error and error > -2) or centerx == 0:
        error = 0

    # set yawerror based on GetLinePosition return value

    if   lineposArray == [1, 0, 0]:
        yawerror = yawvalues[0]

    elif lineposArray == [1, 1, 0]:
        yawerror = yawvalues[1]

    elif lineposArray == [0, 1, 0]:
        yawerror = yawvalues[2]

    elif lineposArray == [0, 1, 1]:
        yawerror = yawvalues[3]

    elif lineposArray == [0, 0, 1]:
        yawerror = yawvalues[4]

    elif lineposArray == [1, 1, 1]:
        yawerror = yawvalues[2]

    elif lineposArray == [0, 0, 0]:
        yawerror = yawvalues[2]
        leave=True

    elif lineposArray == [1, 0, 1]:
        yawerror = yawvalues[2]
        leave=True
    print(error,yawerror)
    return error,yawerror

### MAIN ###
if __name__=="__main__":
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (frameWidth, frameHeight)
    camera.framerate = 25
    # Correct camera angle
    camera.vflip=True
    camera.rotation= 90
    camera.hflip= True
    rawCapture = PiRGBArray(camera, camera.resolution)

    #connect to the copter
    vehicle=Drone.connectMyCopter()
    try:
        #Drone.arm_and_takeoff(vehicle,1.5)

        for frame in camera.capture_continuous(rawCapture,format="bgr", use_video_port=True):
            if leave:
                break
            # Get the image from the frame
            img=frame.array
            #Resize the image if needed
            #img = cv2.resize(img, (frameWidth, frameHeight))
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)

            imgThres = masking(img)

            centerx = getContours(imgThres, img)

            lineposArray = getLinePosition(imgThres, sections)

            error,yawerror=getErrorValues(lineposArray, centerx)
            Drone.setvelocitywithyawangle(vehicle,0.1,error,yawerror)
            cv2.line(img,(int(frameWidth/2)-80,0),(int(frameWidth/2)-80,frameHeight),(255,255,0),3)
            cv2.line(img,(int(frameWidth/2)+80,0),(int(frameWidth/2)+80,frameHeight),(255,255,0),3)
            cv2.imshow("Output", img)

            cv2.imshow("Path", imgThres)
            # Leave if Q is pressed
            if (cv2.waitKey(1) & 0xFF == ord('q')):
                break
            while not leave:
                k=cv2.waitKey(1)
                if k & 0xFF == ord('q'):
                    leave=True
                if k & 0xFF == ord('n'):
                    break
    finally:
        #Drone.land()
        vehicle.close()
        cv2.destroyAllWindows()
