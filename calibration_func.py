import cv2 as cv
import numpy as np

def calibration(filename):

    CHECKERBOARD = (6, 9)
    criteria = (cv.TERM_CRITERIA_EPS +
                cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # Vector for 3D points
    threedpoints = []
    # Vector for 2D points
    twodpoints = []
    #  3D points real world coordinates
    objectp3d = np.zeros((1, CHECKERBOARD[0]
                        * CHECKERBOARD[1],
                        3), np.float32)
    objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0],
                                0:CHECKERBOARD[1]].T.reshape(-1, 2)
    prev_img_shape = None
    
    capvideo = cv.VideoCapture(filename)
    frame_list=[]
    while capvideo.isOpened():
        ret, frame = capvideo.read()
        if not ret:
            break
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        frame_list.append(gray)
 
    interval = 30

    for i in range(0,len(frame_list),interval):
        frame = frame_list[i]
        ret, corners = cv.findChessboardCorners(
                        frame, CHECKERBOARD,
                        cv.CALIB_CB_ADAPTIVE_THRESH
                        + cv.CALIB_CB_FAST_CHECK +
                        cv.CALIB_CB_NORMALIZE_IMAGE)
        if ret == True:
            threedpoints.append(objectp3d)
            corners2 = cv.cornerSubPix(
                frame, corners, (11, 11), (-1, -1), criteria)
            twodpoints.append(corners2)
 
    
    # h, w = frame_list[0].shape[:2]
    ret, matrix, distortion, _, _ = cv.calibrateCamera(
        threedpoints, twodpoints, frame_list[0].shape[::-1], None, None)
    return(matrix, distortion)