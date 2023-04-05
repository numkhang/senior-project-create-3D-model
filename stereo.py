import cv2 as cv
import numpy as np
from tkinter import messagebox
from pose_func import rotationMatrixToEulerAngles,calculate_rt_vec,calculate_relative_rt,calculate__rt_for_rectify,change_ax,rectify,stereo_error
from matching_func import sift_pair 
import concurrent.futures

frame_list = [] 
frame_count = 30
# loop_count = 200

# def stereo_thread(i,K,distortion):
#     length = len(frame_list)
#     if length < i+frame_count:
#         end = length
#     else:
#         end = i+frame_count
#     ox = K[0][2]
#     oy = K[1][2]
#     fx = K[0][0]
#     fy = K[1][1]

#     limitx = [0,350]
#     limity = [-400,0]
#     limitz = 0


#     list_obj = []
#     for j in range(i+5,end,5):
#         try:
#             rvec1,tvec1=calculate_rt_vec(frame_list[i],K,distortion)
#             rvec2,tvec2=calculate_rt_vec(frame_list[j],K,distortion)
#             rmatrix_relative,tvec_relative = calculate__rt_for_rectify(rvec1,tvec1,rvec2,tvec2)
#             rect1,rect2 = rectify(frame_list[i],frame_list[j],K,distortion,rmatrix_relative,tvec_relative)
#             rvec1,tvec1=calculate_rt_vec(rect1,K,distortion)
#             rvec2,tvec2=calculate_rt_vec(rect2,K,distortion)
#             rmatrix_relative,tvec_relative = calculate_relative_rt(rvec1,tvec1,rvec2,tvec2)
#             angle = rotationMatrixToEulerAngles(rmatrix_relative)
#             angle = angle * (180/np.pi)
#             if abs(angle[0]) < 1 and abs(angle[1]) < 1 and abs(angle[2]) < 1 and abs(tvec_relative[0]) > 10 and abs(tvec_relative[1]) < 5 and abs(tvec_relative[2]) < 5:
#                 rot,tran = rvec1,tvec1
#                 base = tvec_relative[0]
#                 error = stereo_error(rect1,rect2,K,rot,tran,base)
#                 if error > 15:
#                     continue
#                 # print (error)
#                 kp_pair = sift_pair(rect1,rect2)
#                 for pt in kp_pair:  
#                     obj_xyz = np.array([0,0,0])
#                     (u1,v1) = pt[0] 
#                     (u2,v2) = pt[1]
#                     color = rect1[int(v1)][int(u1)]
#                     u1 = u1-ox
#                     u2 = u2-ox
#                     v1 = v1-oy
#                     v2 = v2-oy
#                     try:
#                         z = (base * fx )/(u1-u2)
#                         y = (z * v1) / fy
#                         x = (z * u1 ) / fx
#                         obj_xyz = np.array([x,y,z]) 
#                         obj_xyz = change_ax(obj_xyz,rot,tran)
#                         if obj_xyz[0] > limitx[0] and obj_xyz[0] < limitx[1]:
#                             if obj_xyz[1] > limity[0] and obj_xyz[1] < limity[1]:
#                                 if obj_xyz[2] > limitz:
#                                     obj_color = np.concatenate((obj_xyz.ravel(),color))
#                                     list_obj.append(obj_color)
#                     except:
#                         continue
#         except:
#             continue
#     return (list_obj)


def stereo_calculation (filename,K,distortion,loop_count):
    try:
        capvideo = cv.VideoCapture(filename)
        frame_list.clear()
        while capvideo.isOpened():
            ret, frame = capvideo.read()
            if not ret:
                break
            frame_list.append(frame)
        length = len(frame_list)

        result_list =[]

        step = int(length/loop_count)
        if step < 1 :
            step = 1

        # with concurrent.futures.ThreadPoolExecutor() as executor:
        #     response_process = []
        #     for i in range(0,length-frame_count,step):
        #         response_process.append(executor.submit(stereo_thread,i=i,K=K,distortion=distortion))
            
        # for f in response_process:
        #     result_list.append(f.result())
        # executor.shutdown()

        # list_obj =[]
        # for lst in result_list:
        #     list_obj.extend(lst)
        # return list_obj

        list_obj =[]

        ox = K[0][2]
        oy = K[1][2]
        fx = K[0][0]
        fy = K[1][1]

        limitx = [0,350]
        limity = [-400,0]
        limitz = 0

        for i in range(0,length-frame_count,step):
            length = len(frame_list)
            if length < i+frame_count:
                end = length
            else:
                end = i+frame_count
            for j in range(i+1,end,5):
                try:
                    rvec1,tvec1=calculate_rt_vec(frame_list[i],K,distortion)
                    rvec2,tvec2=calculate_rt_vec(frame_list[j],K,distortion)
                    rmatrix_relative,tvec_relative = calculate__rt_for_rectify(rvec1,tvec1,rvec2,tvec2)
                    rect1,rect2 = rectify(frame_list[i],frame_list[j],K,distortion,rmatrix_relative,tvec_relative)
                    # rvec1,tvec1=calculate_rt_vec(rect1,K,distortion)
                    # rvec2,tvec2=calculate_rt_vec(rect2,K,distortion)
                    rvec1,tvec1=calculate_rt_vec(rect1,K,None)
                    rvec2,tvec2=calculate_rt_vec(rect2,K,None)
                    rmatrix_relative,tvec_relative = calculate_relative_rt(rvec1,tvec1,rvec2,tvec2)
                    angle = rotationMatrixToEulerAngles(rmatrix_relative)
                    angle = angle * (180/np.pi)
                    if abs(angle[0]) < 1 and abs(angle[1]) < 1 and abs(angle[2]) < 1 and abs(tvec_relative[0]) > 10 and abs(tvec_relative[1]) < 5 and abs(tvec_relative[2]) < 5:
                        rot,tran = rvec1,tvec1
                        base = tvec_relative[0]
                        error = stereo_error(rect1,rect2,K,rot,tran,base)
                        if error > 20:
                            continue
                        kp_pair = sift_pair(rect1,rect2)
                        for pt in kp_pair:  
                            obj_xyz = np.array([0,0,0])
                            (u1,v1) = pt[0] 
                            (u2,v2) = pt[1]
                            color = rect1[int(v1)][int(u1)]
                            u1 = u1-ox
                            u2 = u2-ox
                            v1 = v1-oy
                            v2 = v2-oy
                            try:
                                z = (base * fx )/(u1-u2)
                                y = (z * v1) / fy
                                x = (z * u1 ) / fx
                                obj_xyz = np.array([x,y,z]) 
                                obj_xyz = change_ax(obj_xyz,rot,tran)
                                if obj_xyz[0] > limitx[0] and obj_xyz[0] < limitx[1]:
                                    if obj_xyz[1] > limity[0] and obj_xyz[1] < limity[1]:
                                        if obj_xyz[2] > limitz:
                                            obj_color = np.concatenate((obj_xyz.ravel(),color))
                                            list_obj.append(obj_color)
                            except Exception as e:
                                # print(e)
                                continue
                except Exception as e:
                    # print(e)
                    continue
        return list_obj
    except Exception as e:
        print(e)
        return ("error")
        
        