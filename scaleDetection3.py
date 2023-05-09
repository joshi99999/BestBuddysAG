import cv2
import numpy as np
from math import cos, sin, pi, atan

org = cv2.imread('image1.jpg')
org = cv2.GaussianBlur(src=org, ksize=(5,5), sigmaX=0)

a = 0.1
b = 50

ratio = org.shape[1]/org.shape[0]
gamma = atan(1/ratio)

for alpha in np.linspace(-pi/2, pi/2, int(pi/a)):
    img = org.copy()
    v = np.array([cos(alpha), sin(alpha)], dtype=np.float32)
    vo = v[::-1] * [1,-1]
    n = int(np.hypot(img.shape[0], img.shape[1]) * cos(abs(alpha)-gamma) / b)
    for i in range(1, n+1, 1):
        p = i * b * v
        if alpha < 0:
            p = p + [0, img.shape[0]]

        j = 0
        line = [(0,0),(0,0)]

        if vo[1] != 0:
            s = (p[0]*vo[1]-p[1]*vo[0])/vo[1]
            if 0 <= s < img.shape[1]:
                line[j] = (int(s), 0)
                j += 1

        if vo[0] != 0:
            s = (p[1]*vo[0]+(img.shape[1]-p[0])*vo[1])/vo[0]
            if 0 <= s < img.shape[0]:
                line[j] = (img.shape[1], int(s))
                j += 1

        if j < 2 and vo[1] != 0:
            s = (p[0]*vo[1]-(p[1]-img.shape[0])*vo[0])/vo[1]
            if 0 < s <= img.shape[1]:
                line[j] = (int(s), img.shape[0])
                j += 1

        if j < 2 and vo[0] != 0:
            s = (p[1]*vo[0]-p[0]*vo[1])/vo[0]
            if 0 < s <= img.shape[0]:
                line[j] = (0, int(s))

        cv2.circle(img=img, center=line[0], radius=3, color=(255,0,0), thickness=-1)
        cv2.circle(img=img, center=line[1], radius=3, color=(255,0,0), thickness=-1)
        cv2.circle(img=img, center=p.astype(np.int32), radius=3, color=(0,255,0), thickness=-1)

        cv2.line(img=img, pt1=line[0], pt2=line[1], color=(0,0,255), thickness=1)
        cv2.imshow('Image', img)
        cv2.waitKey(0)