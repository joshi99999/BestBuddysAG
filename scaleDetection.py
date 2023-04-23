import cv2
import numpy as np

org = cv2.imread(filename="Repository/image.jpg")
org = cv2.resize(src=org, dsize=(960, 540))
gray = cv2.cvtColor(src=org, code=cv2.COLOR_BGR2GRAY)
img = cv2.GaussianBlur(src=gray, ksize=(5, 5), sigmaX=0)
lsd = cv2.createLineSegmentDetector()

lines = np.reshape(lsd.detect(img)[0], (-1, 2, 2))
lengths = np.hypot(lines[:,0,0] - lines[:,1,0], lines[:,0,1] - lines[:,1,1])
centers = lines[:,0] + (lines[:,1] - lines[:,0]) / 2

indexes = np.arange(0, len(lines), 1)


for i in range(len(lines)):
    color = org.copy()
    for j in range(len(lines)):
        line = lines[j]
        center = (int(centers[j][0]), int(centers[j][1]))
        color = cv2.line(img=color, pt1=(int(line[0,0]), int(line[0,1])), pt2=(int(line[1,0]), int(line[1,1])), color=(0, 0, 255), thickness=1)
        color = cv2.circle(img=color, center=center, radius=1, color=(255,0,0), thickness=-1)
    color = cv2.circle(img=color, center=(int(centers[i][0]), int(centers[i][1])), radius=int(lengths[i]), color=(0,255,0), thickness=1)

    vectors = centers - centers[i]
    
    near = indexes[np.hypot(vectors[:,0], vectors[:,1]) < lengths[i]]

    for n in near:

        vector1 = lines[i,1] - lines[i,0]
        vector2 = lines[n,1] - lines[n,0]

        vector3 = vector1[[1,0]] * [1,-1]


        parallel = (lines[i] + vector3) if 0 < np.dot(centers[n] - centers[i], vector3) else (lines[i] - vector3)

        M = cv2.getPerspectiveTransform(np.concatenate((lines[i], lines[n] if 0 < np.dot(vector1, vector2) else lines[n,[1,0]]), dtype=np.float32), np.concatenate((lines[i], parallel), dtype=np.float32))

        print(M)

        color = cv2.circle(img=color, center=(int(centers[n][0]),int(centers[n][1])), radius=2, color=(0,0,255), thickness=-1)
        color = cv2.line(img=color, pt1=(int(lines[i,0,0]), int(lines[i,0,1])), pt2=(int(parallel[0,0]), int(parallel[0,1])), color=(0, 255, 255), thickness=1)
        color = cv2.line(img=color, pt1=(int(lines[i,1,0]), int(lines[i,1,1])), pt2=(int(parallel[1,0]), int(parallel[1,1])), color=(0, 255, 255), thickness=1)
        color = cv2.line(img=color, pt1=(int(parallel[0,0]), int(parallel[0,1])), pt2=(int(parallel[1,0]), int(parallel[1,1])), color=(0, 255, 255), thickness=1)
        cv2.imshow("Bearbeitet", color)
        cv2.waitKey(1)