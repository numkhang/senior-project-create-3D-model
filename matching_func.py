import cv2 as cv
import numpy as np

def sift_pair(img1,img2):
    sift = cv.SIFT_create()
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=50)   # or pass empty dictionary
    flann = cv.FlannBasedMatcher(index_params,search_params)
    matches = flann.knnMatch(des1,des2,k=2)

    matchesMask = [[0,0] for i in range(len(matches))]
    good =[]
    
    # ratio test as per Lowe's paper
    for i,(m,n) in enumerate(matches):
        if m.distance < 0.5*n.distance:
            good.append(m)
            matchesMask[i]=[1,0]

    mkp1 =[]
    mkp2 =[]
    for mat in good:
        img1_idx = mat.queryIdx
        img2_idx = mat.trainIdx
        mkp1.append(kp1[img1_idx].pt)
        mkp2.append(kp2[img2_idx].pt)

    mkp2 = np.float32(np.array(mkp2))
    mkp1 = np.float32(np.array(mkp1))
    F, mask = cv.findFundamentalMat(mkp1,mkp2,cv.FM_RANSAC)
    mkp1 = mkp1[mask.ravel()==1]
    mkp2 = mkp2[mask.ravel()==1]

    kp_pair = list(zip(mkp1,mkp2))

    return kp_pair
