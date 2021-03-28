import numpy as np
import cv2

cv2.namedWindow("Ring Demo")
cv2.moveWindow("Ring Demo", 200, 200)

cv2.namedWindow("Result")
cv2.namedWindow("HSV")
cv2.moveWindow("Result", 500, 200)

def empty(a):
    pass

cv2.createTrackbar("H_MIN", "HSV", 98, 255, empty)
cv2.createTrackbar("H_MAX", "HSV", 109, 255, empty)
cv2.createTrackbar("S_MIN", "HSV", 196, 255, empty)
cv2.createTrackbar("S_MAX", "HSV", 255, 255, empty)
cv2.createTrackbar("V_MIN", "HSV", 181, 255, empty)
cv2.createTrackbar("V_MAX", "HSV", 255, 255, empty)

#video = cv2.VideoCapture('img/first.mp4')
video = cv2.VideoCapture(0)

frameCount = 0

#IGNORE THE COMMENTED OUT CODE!!!!

while True:

    ret,frame = video.read()
    #frame = cv2.imread("C:\\Users\\Gamer\\Desktop\\robot code\\openCVTest\\img\\goodTestImage.png")
    frameCount += 1

    # if video is over, replay the video
    #if frameCount == video.get(cv2.CAP_PROP_FRAME_COUNT):
    #    frameCount = 0  # Or whatever as long as it is the same as next line
    #    video.set(cv2.CAP_PROP_POS_FRAMES, 0)

    frame = cv2.resize(frame, (700, 700))

    #convert frame to hsv and apply a blur
    imgHsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    imgHsv = cv2.GaussianBlur(imgHsv,(5,5),0)

    #create window with trackbar
    h_min = cv2.getTrackbarPos("H_MIN", "HSV")
    h_max = cv2.getTrackbarPos("H_MAX", "HSV")
    s_min = cv2.getTrackbarPos("S_MIN", "HSV")
    s_max = cv2.getTrackbarPos("S_MAX", "HSV")
    v_min = cv2.getTrackbarPos("V_MIN", "HSV")
    v_max = cv2.getTrackbarPos("V_MAX", "HSV")

    #create mask and apply it to the result
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    mask = cv2.inRange(imgHsv, lower, upper)
    contourMat = cv2.bitwise_and(imgHsv, imgHsv, mask = mask)

    contourMat = cv2.cvtColor(contourMat, cv2.COLOR_BGR2GRAY);
    kernel =  np.ones((5,5), np.uint8)
    contourMat = cv2.dilate(contourMat,kernel, iterations = 2)
    #'''
    contours,_ = cv2.findContours(contourMat, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(frame,contours, -1, (0,255,0), 3)

    largestX = 0
    largestY = 0
    largestArea = 0

    for contour in contours:
        (x, y, w, h) = cv2.boundingRect(contour)
    
        if cv2.contourArea(contour) > 15_000:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        if w * h > largestArea:
            largestX = x
            largestY = y
            largestArea = w * h

     #   '''
    print("x ",largestX,"y ",largestY)

    cv2.imshow("Ring Demo", contourMat)
    cv2.imshow("Result", frame)
    cv2.imshow("HSV", imgHsv)

    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break

video.release()
cv2.destroyAllWindows()