from skimage.filters.rank import entropy
from skimage.morphology import disk
import numpy as np
import cv2
from time import time
from math import cos, sin

for image in range(11):

    gray = cv2.imread("image" + str(image + 1) + ".jpg", flags=cv2.IMREAD_GRAYSCALE)

    previous = time()
    ent = entropy(image=cv2.equalizeHist(cv2.GaussianBlur(src=gray, ksize=(29, 29), sigmaX=5, sigmaY=5)), footprint=disk(1))
    print(time()-previous)

    ent = cv2.threshold(src=ent, thresh=0, maxval=255, type=cv2.THRESH_BINARY_INV)[1].astype(np.uint8)
    ent = cv2.threshold(src=gray, thresh=100, maxval=255, type=cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1].astype(np.uint8) // 255 * ent

    cv2.imshow("Image", gray)
    cv2.imshow("Entropie", ent)
    cv2.waitKey(0)


    kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3,3))

    ent = cv2.dilate(src=ent, kernel=kernel, iterations=1)
    ent = cv2.erode(src=ent, kernel=kernel, iterations=8)
    ent = cv2.dilate(src=ent, kernel=kernel, iterations=10)


    ent = cv2.threshold(src=ent, thresh=0, maxval=255, type=cv2.THRESH_BINARY)[1].astype(np.uint8)
    cv2.imshow("Image", gray)
    cv2.imshow("Entropie", ent)
    cv2.waitKey(0)


    contours = cv2.findContours(image=ent, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)[0]

    img = cv2.cvtColor(src=gray, code=cv2.COLOR_GRAY2BGR)

    centers = np.zeros_like(ent)

    for contour in contours:
        M = cv2.moments(contour)
        try:
            cv2.circle(img=centers, center=(int(M['m10']/M['m00']), int(M['m01']/M['m00'])), radius=10, thickness=1, color=255)
        finally:
            continue

    lines = cv2.HoughLinesP(image=centers, rho=1, theta=np.pi/180, threshold=20, lines=None, minLineLength=400, maxLineGap=200)[:,0]

    if lines is not None:
        line = max(lines, key=lambda x: np.hypot(x[2]-x[0], x[3]-x[1]))
        cv2.line(centers, (line[0], line[1]), (line[2], line[3]), 255, 1, cv2.LINE_AA)
        
        line_vector = np.array([line[2]-line[0], line[3]-line[1]], dtype=np.float32)
        unit_vector =  line_vector / np.hypot(line_vector[0], line_vector[1])
        
        gray = cv2.GaussianBlur(src=gray, ksize=(9, 9), sigmaX=0)
        lsd_lines = np.reshape(cv2.createLineSegmentDetector().detect(gray)[0], (-1, 2, 2))
        line_centers = lsd_lines[:,0] + (lsd_lines[:,1] - lsd_lines[:,0]) / 2
  
        indices = np.arange(0, len(line_centers))

        projection = (line_centers - [line[0], line[1]]) @ unit_vector

        indices = indices[np.logical_and(0 < projection, projection < np.hypot(line_vector[0], line_vector[1]))]

        normal_vectors = line_centers[indices] - np.reshape(projection[indices], (-1,1)) * unit_vector - [line[0], line[1]]

        distances = np.hypot(normal_vectors[:,0], normal_vectors[:,1])

        indices = indices[distances < 10]

        for l in lsd_lines[indices].astype(np.int32):
            cv2.line(img=img, pt1=l[0], pt2=l[1], color=(0,0,255), thickness=2)



    cv2.imshow("Centers", centers)
    #cv2.imshow("Entropie", ent)
    cv2.imshow("Image", img)
    #cv2.imshow("Gray", gray)
    cv2.waitKey(0)
    cv2.destroyAllWindows()