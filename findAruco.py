import cv2.aruco as aruco
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

green =(0,255,0)
red = (0,0,255)

def findAruco(img,size=4,total=1000):
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    key = getattr(aruco,f'DICT_{size}X{size}_{total}')

    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
    parameters =  cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(dictionary, parameters)
    (corners, ids, rejected) = detector.detectMarkers(img)
    # arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
    # arucoParams = aruco.DetectorParameters_create()
    # (corners, ids, rejected)  = aruco.detectMarkers(img, arucoDict,parameters=arucoParams)
    return corners,ids,rejected
    
def findAruco_Dict_Id_Center(img,size=4,total=1000):
    corners,ids,rej = findAruco(img,size,total)
    res = {}
    for i in range(ids.size):
        box = corners[i][0]
        x=0
        y=0
        for pt in box:
            x += pt[0]
            y += pt[1]
        x = x/4
        y = y/4
        res[int(ids[i][0])]=(x,y)
    return res



