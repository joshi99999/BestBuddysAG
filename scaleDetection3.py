import cv2
import numpy as np
from math import cos, sin, pi, atan

org = cv2.imread(filename='Repository/Images/image1.jpg', flags=cv2.IMREAD_GRAYSCALE)
org = cv2.resize(src=org, dsize=(480,270))
org = cv2.GaussianBlur(src=org, ksize=(5,5), sigmaX=0)

a = 0.05
b = 5

shape = (org.shape[0]-1, org.shape[1]-1)

ratio = shape[1]/shape[0]
gamma = atan(1/ratio)

for alpha in np.linspace(-pi/2, pi/2, int(pi/a)):
    img = org.copy()
    img = cv2.cvtColor(src=img, code=cv2.COLOR_GRAY2BGR)
    v = np.array([cos(alpha), sin(alpha)], dtype=np.float32)
    vo = v[::-1] * [1,-1]
    n = int(np.hypot(shape[0], shape[1]) * cos(abs(alpha)-gamma) / b)
    for i in range(1, n+1, 1):
        p = i * b * v
        if alpha < 0:
            p = p + [0, shape[0]]

        j = 0
        ends = np.zeros(shape=(2,2), dtype=np.int16)

        if vo[1] != 0:
            s = (p[0]*vo[1]-p[1]*vo[0])/vo[1]
            if 0 <= s < shape[1]:
                ends[j] = [int(s), 0]
                j += 1

        if vo[0] != 0:
            s = (p[1]*vo[0]+(shape[1]-p[0])*vo[1])/vo[0]
            if 0 <= s < shape[0]:
                ends[j] = [shape[1], int(s)]
                j += 1

        if j < 2 and vo[1] != 0:
            s = (p[0]*vo[1]-(p[1]-shape[0])*vo[0])/vo[1]
            if 0 < s <= shape[1]:
                ends[j] = [int(s), shape[0]]
                j += 1

        if j < 2 and vo[0] != 0:
            s = (p[1]*vo[0]-p[0]*vo[1])/vo[0]
            if 0 < s <= shape[0]:
                ends[j] = [0, int(s)]

        cv2.circle(img=img, center=ends[0], radius=3, color=(255,0,0), thickness=-1)
        cv2.circle(img=img, center=ends[1], radius=3, color=(255,0,0), thickness=-1)
        cv2.circle(img=img, center=p.astype(np.int32), radius=3, color=(0,255,0), thickness=-1)

        #cv2.line(img=img, pt1=ends[0], pt2=ends[1], color=(0,0,255), thickness=1)

        line_vector = ends[0]-ends[1]
        angle = np.arctan2(line_vector[1],line_vector[0])
        length = np.ceil(np.linalg.norm(line_vector))
        indices = ([cos(angle), sin(angle)] * np.reshape(np.arange(0, length+1, 1, dtype=np.int16), (-1,1)) + ends[1]).astype(np.uint16)
        otsu = cv2.threshold(src=org, thresh=150, maxval=255, type=cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
        line = otsu[indices[:,1], indices[:,0]]

        cv2.line(img=img, pt1=ends[0], pt2=ends[1], color=(0,0,255), thickness=1)

        edges = np.arange(1, line.shape[0], 1)[line[1:] != line[0:-1]]
        if 20 < edges.shape[0]:
            distances = edges[1:] - edges[0:-1]
            ratios = distances[1:] / distances[0:-1]
            areas = np.all([0.5 < ratios, ratios < 1.5, 5 < distances[1:]], axis=0)
            areas = np.concatenate(([False], areas, [False]))
            borders = np.arange(1, areas.shape[0], 1)[areas[1:] != areas[0:-1]]
            if 1 < borders.shape[0]:
                lengths = borders[1:] - borders[0:-1]
                if 20 < max(lengths):
                    index = borders[np.argmax(lengths)]
                    cv2.line(img=img, pt1=indices[edges[index]], pt2=indices[edges[index+max(lengths)]], color=(0,255,0), thickness=1)

        cv2.imshow('Otsu', otsu)
        cv2.imshow('Image', img)
        cv2.imshow('Line', np.tile(line, (50,1)))
        cv2.waitKey(1)