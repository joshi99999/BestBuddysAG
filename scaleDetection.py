import cv2
import numpy as np

org = cv2.imread(filename="Repository/image.jpg", flags=cv2.IMREAD_GRAYSCALE)
org = cv2.resize(src=org, dsize=(960, 540))
img = cv2.GaussianBlur(src=org, ksize=(5, 5), sigmaX=0)
lsd = cv2.createLineSegmentDetector()
lines = np.reshape(lsd.detect(img)[0], (-1, 4))
length = np.hypot(lines[:,0] - lines[:,2], lines[:, 1] - lines[:, 3])
color = cv2.cvtColor(src=org, code=cv2.COLOR_GRAY2BGR)

#linesSortedByCenterX =
#linesSortedByCenterY = 

for i in range(len(lines)):
    line = lines[i]
    pt1 = (int(line[0]), int(line[1]))
    pt2 = (int(line[2]), int(line[3]))
    center = ((pt2[0] - pt1[0]) // 2 + pt1[0], (pt2[1] - pt1[1]) // 2 + pt1[1])
    color = cv2.line(img=color, pt1=pt1, pt2=pt2, color=(0, 0, 255), thickness=1)
    color = cv2.circle(img=color, center=center, radius=1, color=(255,0,0), thickness=-1)
    color = cv2.circle(img=color, center=center, radius=int(length[i]), color=(0,255,0), thickness=1)

cv2.imshow("Bearbeitet", color)
cv2.waitKey(0)