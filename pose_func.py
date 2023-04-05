import cv2 as cv
import numpy as np
from findAruco import findAruco_Dict_Id_Center
import math

bg_marker_pos = {}
id = 0
a4w = 210
a4h = -297
disrow = 105
discol = -99
page_pos=[[0,0,0],[a4w,0,0],[0,a4h,0],[a4w,a4h,0]]
for page in range(4):
    for i in range(3):
        for j in range(2):
            bg_marker_pos[id] = [np.float32(page_pos[page][0]+j*disrow),np.float32(page_pos[page][1]+i*discol),np.float32(page_pos[page][2])]
            id += 1


def rotationMatrixToEulerAngles(R) :
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0]) 
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

def calculate_rt_vec(img,K,distortion):
    Id_dict_img1= findAruco_Dict_Id_Center(img)
    points_2D = []
    points_3D = []
    for i in Id_dict_img1.keys():
        if i in range(24):
            points_2D.append (Id_dict_img1[i])
            points_3D.append (bg_marker_pos[i])
        
    points_2D = np.array(points_2D,dtype=np.float32)
    points_3D = np.array(points_3D,dtype=np.float32)

    ret, rvec, tvec = cv.solvePnP(points_3D,points_2D, K, distortion)
    return(rvec,tvec)

def calculate_relative_rt(rvec1,tvec1,rvec2,tvec2):
    rmatrix1, _ = cv.Rodrigues(rvec1)
    rmatrix2, _ = cv.Rodrigues(rvec2)
    rmatrix2_inv = np.linalg.inv(rmatrix2)
    rmatrix = np.matmul(rmatrix1,rmatrix2_inv)
    tvec = tvec1 - tvec2
    return(rmatrix,tvec)

def calculate__rt_for_rectify(rvec1,tvec1,rvec2,tvec2):
    rmatrix1, _ = cv.Rodrigues(rvec1)
    rmatrix2, _ = cv.Rodrigues(rvec2)
    rmatrix1_inv = np.linalg.inv(rmatrix1)
    rmatrix = np.matmul(rmatrix2,rmatrix1_inv)
    tvec = tvec2 - np.matmul(rmatrix,tvec1)
    return(rmatrix,tvec)

def change_ax (obj,rvec,tvec):
    point_camera =np.array([obj[0],obj[1],obj[2]])
    rmatrix, _ = cv.Rodrigues(rvec)
    rmatrix_inv = np.linalg.inv(rmatrix)
    t_inv = -np.matmul(rmatrix_inv, tvec)
    point_world = np.matmul(rmatrix_inv, point_camera) + t_inv
    return point_world

def rectify (img1,img2,K,dist,R,T):
    height, width = img1.shape[:2]
    R1,R2,P1,P2,_,_,_ = cv.stereoRectify(K, dist, K, dist, (width, height), R, T, alpha=-1)
    map1x, map1y = cv.initUndistortRectifyMap(K, dist, R1, P1, (width, height), cv.CV_32FC1)
    map2x, map2y = cv.initUndistortRectifyMap(K, dist, R2, P2, (width, height), cv.CV_32FC1)

    Rect_img1 = cv.remap(img1, map1x, map1y, cv.INTER_LINEAR)
    Rect_img2 = cv.remap(img2, map2x, map2y, cv.INTER_LINEAR)

    return(Rect_img1,Rect_img2)


def stereo_error(img1,img2,K,rot,tran,base):
    Id_dict_img1= findAruco_Dict_Id_Center(img1)
    Id_dict_img2 = findAruco_Dict_Id_Center(img2)

    ox = K[0][2]
    oy = K[1][2]
    fx = K[0][0]
    fy = K[1][1]

    available_key = []
    dict_obj_mk ={}
    for i in Id_dict_img1.keys():
        if not(i in Id_dict_img2.keys()):
            continue
        if not(i in bg_marker_pos.keys()):
            continue
        obj_xyz = np.array([0,0,0])
        (u1,v1) = Id_dict_img1[i] 
        (u2,v2) = Id_dict_img2[i] 
        u1 = u1-ox
        u2 = u2-ox
        v1 = v1-oy
        v2 = v2-oy
        try:
            z = (base * fx)/(u1-u2)
            y = (z*v1) / fy
            x =(z*u1) / fx
            obj_xyz = np.array([x,y,z]) 
            obj_xyz = change_ax(obj_xyz,rot,tran)
            dict_obj_mk[i] = obj_xyz
            available_key.append(i)
        except:
            continue

    sum_error = 0
    for id in available_key:
        try:
            expected = bg_marker_pos[id]
            obj = dict_obj_mk[id]
            real = obj.ravel()
            error_vec = real - expected
            error = np.sqrt(error_vec[0]**2+error_vec[1]**2+error_vec[2]**2)
            sum_error += abs(error)
        except:
            print("error in stereo error",id)
    avg_error = sum_error/len(available_key)
    return avg_error
