import cv2
import numpy as np
from math import cos, sin, pi, atan
from time import time

class ScaleDetector:

    def __init__(self):
        self.__radius = 0           #Distance between the ankor and the coordinate origin
        self.__delta = 20           #Radius step width
        self.__maxRadius = 0

        self.__alpha = -pi/2        #Angle between the x-axis and a line through the coordinate origin and the ankor
        self.__phi = 0.01           #Angle step width

        self.__ankorDirectionVector = np.array([0,1], dtype=np.float32)             #Unity vector facing from the origin to the ankor
        self.__lineDirectionVector = np.array([1,0], dtype=np.float32)              #Unity vector orgthogonal to the ankorDirectionVector

    def __nextLine(self, shape):
        self.__radius += self.__delta
        if self.__maxRadius < self.__radius:
            self.__radius = self.__delta
            self.__alpha = self.__phi - self.__alpha if 0 < self.__alpha else -self.__alpha
            self.__ankorDirectionVector = np.array([cos(self.__alpha), sin(self.__alpha)], dtype=np.float32)
            self.__lineDirectionVector = self.__ankorDirectionVector[::-1] * [1,-1]
            self.__maxRadius = np.hypot(shape[0], shape[1]) * cos(abs(self.__alpha) - atan(shape[0]/shape[1]))

    @staticmethod
    def scanArea(img, ankor, alpha, delta, phi):
        angles = np.linspace(alpha-phi, alpha+phi, 5)
        unityVectors = np.stack((np.cos(angles), np.sin(angles)), axis=-1)
        vectors = unityVectors[:,::-1]*[1,-1]
        ankors = ankor + np.reshape(np.outer(np.linspace(-delta, delta, 10), unityVectors), (-1,2))
        ends = ScaleDetector.calculateLineEnds(shape=(img.shape[0]-1,img.shape[1]-1), anchor=ankors, vector=np.tile(vectors, (10,1)))
        line = None
        for e in ends:
            indices = ScaleDetector.calculateLineIndices(ends=e)
            edges = ScaleDetector.analyseLine(line=img[indices[:,1], indices[:,0]])
            if edges is not None and (line is None or min(indices[edges][:,0]) < min(line[:,0])):
                line = indices[edges]
        return line[[0,20]] if line[0,0] < line[-1,0] else line[[-1,-21]]
    
    def findCorners(img, snap, line):
        angle = np.arctan2(line[0,1]-line[1,1], line[0,0]-line[1,0])
        anchors = line if line[0,0] < line[1,0] else line[::-1]
        corners = np.array([line[1], line[1], line[0], line[0]], dtype=np.uint16)
        for i in range(4):
            phi = 0
            while phi < pi/2:
                phi = phi+pi/720 if i%2 == 0 else phi-pi/720
                vector = np.array([[cos(angle+phi), sin(angle+phi)]], dtype=np.float32)
                ends = ScaleDetector.calculateLineEnds(shape=(img.shape[0]-1,img.shape[1]-1), anchor=anchors[0:1] if i<2 else anchors[1:2], vector=vector)[0]
                indices = ScaleDetector.calculateLineIndices(ends)
                edges = ScaleDetector.analyseLine(line=img[indices[:,1], indices[:,0]])
                if edges is not None:
                    edges = indices[edges]
                    edges = edges[anchors[0,0] <= edges[:,0] if i < 2 else edges[:,0] <= anchors[1,0]]
                    if 20 < edges.shape[0]:
                        corners[i] = edges[20 if i<2 else 0] if edges[0,0] < edges[-1,0] else edges[-21 if i<2 else -1]
                        continue
                break
        return corners
    
    @staticmethod
    def calculateLineEnds(shape, anchor, vector):
        j = np.zeros(anchor.shape[0], dtype=np.uint8)
        ends = np.zeros(shape=(anchor.shape[0],2,2), dtype=np.int16)

        i = np.arange(0,vector.shape[0],1)[vector[:,1] != 0]
        if 0 < i.shape[0]:
            s = (anchor[i,0]*vector[i,1]-anchor[i,1]*vector[i,0])/vector[i,1]
            i = i[np.logical_and(0<=s, s<shape[1])]
            if 0 < i.shape[0]:
                ends[i,j[i],0] = s[i].astype(np.uint16)
                ends[i,j[i],1] = 0
                j[i] += 1

        i = np.arange(0,vector.shape[0],1)[vector[:,0] != 0]
        if 0 < i.shape[0]:
            s = (anchor[i,1]*vector[i,0]+(shape[1]-anchor[i,0])*vector[i,1])/vector[i,0]
            i = i[np.logical_and(0<=s, s<shape[0])]
            if 0 < i.shape[0]:
                ends[i,j[i],0] = shape[1]
                ends[i,j[i],1] = s[i].astype(np.uint16)
                j[i] += 1

        i = np.arange(0,vector.shape[0],1)[np.logical_and(vector[:,1] != 0, j < 2)]
        if 0 < i.shape[0]:
            s = (anchor[i,0]*vector[i,1]-(anchor[i,1]-shape[0])*vector[i,0])/vector[i,1]
            i = i[np.logical_and(0<s, s<=shape[1])]
            if 0 < i.shape[0]:
                ends[i,j[i],0] = s[i].astype(np.uint16)
                ends[i,j[i],1] = shape[0]
                j[i] += 1

        i = np.arange(0,vector.shape[0],1)[np.logical_and(vector[:,0] != 0, j < 2)]
        if 0 < i.shape[0]:
            s = (anchor[i,1]*vector[i,0]-anchor[i,0]*vector[i,1])/vector[i,0]
            i = i[np.logical_and(0<s, s<=shape[0])]
            if 0 < i.shape[0]:
                ends[i,j[i],0] = 0
                ends[i,j[i],1] = s[i].astype(np.uint16)
        
        return ends

    @staticmethod
    def calculateLineIndices(ends):
        lineVector = ends[0]-ends[1]
        angle = np.arctan2(lineVector[1],lineVector[0])
        length = np.ceil(np.linalg.norm(lineVector))
        return ([cos(angle), sin(angle)] * np.reshape(np.arange(0, length+1, 1, dtype=np.int16), (-1,1)) + ends[1]).astype(np.uint16)

    @staticmethod
    def analyseLine(line):
        edges = np.arange(0, line.shape[0]-1, 1)[line[1:] != line[0:-1]]
        if 20 < edges.shape[0]:
            distances = edges[1:] - edges[0:-1]
            ratios = distances[1:] / distances[0:-1]
            areas = np.all([0.5 < ratios, ratios < 1.5, 5 < distances[1:]], axis=0)
            areas = np.concatenate(([False], areas, [False]))
            borders = np.arange(0, areas.shape[0]-1, 1)[areas[1:] != areas[0:-1]]
            if 1 < borders.shape[0]:
                lengths = (borders[1:] - borders[0:-1])[areas[1:][borders][0:-1]]
                if 20 < max(lengths):
                    index = borders[np.argmax(lengths)]+1
                    return edges[index:index+max(lengths)]

    def detectScale(self, img):
        otsu = cv2.threshold(src=img, thresh=150, maxval=255, type=cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
        img = cv2.cvtColor(src=img, code=cv2.COLOR_GRAY2BGR)

        while self.__phi < abs(self.__alpha):
            snap = img.copy()
            self.__nextLine(shape=snap.shape)
            ankor = self.__radius * self.__ankorDirectionVector
            if self.__alpha < 0:
                ankor = ankor + [0, snap.shape[0]]
            ends = self.calculateLineEnds(shape=(snap.shape[0]-1,snap.shape[1]-1), anchor=np.array([ankor]), vector=np.array([self.__lineDirectionVector]))[0]
            indices = self.calculateLineIndices(ends=ends)
            line = self.analyseLine(line=otsu[indices[:,1], indices[:,0]])
            #cv2.line(img=snap, pt1=ends[0], pt2=ends[1], color=(0,0,255), thickness=1)
            #cv2.circle(img=snap, center=ends[0], radius=3, color=(255,0,0), thickness=-1)
            #cv2.circle(img=snap, center=ends[1], radius=3, color=(255,0,0), thickness=-1)
            #cv2.circle(img=snap, center=ankor.astype(np.int32), radius=3, color=(0,255,0), thickness=-1)
            if line is not None:
                left = ScaleDetector.scanArea(img=otsu, ankor=ends[0]+(ends[1]-ends[0])//2, alpha=self.__alpha, delta=10, phi=pi/50)
                if left is not None:
                    #cv2.line(img=snap, pt1=left[0], pt2=left[1], color=(255,0,0), thickness=1)
                    corners = ScaleDetector.findCorners(img=otsu, snap=snap, line=left.astype(np.int16))
                    cv2.line(img=snap, pt1=corners[0], pt2=corners[1], color=(0,0,255), thickness=2)
                    cv2.line(img=snap, pt1=corners[1], pt2=corners[2], color=(0,0,255), thickness=2)
                    cv2.line(img=snap, pt1=corners[2], pt2=corners[3], color=(0,0,255), thickness=2)
                    cv2.line(img=snap, pt1=corners[3], pt2=corners[0], color=(0,0,255), thickness=2)
                    cv2.imshow("Image", snap)
                    #cv2.imshow("Otsu", otsu)
                    #cv2.imshow("Line", np.tile(otsu[indices[:,1], indices[:,0]], (50,1)))
                    cv2.waitKey(0)

            #cv2.imshow("Image", snap)
            #cv2.imshow("Otsu", otsu)
            #cv2.imshow("Line", np.tile(otsu[indices[:,1], indices[:,0]], (50,1)))
            #cv2.waitKey(1)

for n in range(1,13,1):
    img = cv2.imread(filename='Repository/Images/image' + str(n) + '.jpg', flags=cv2.IMREAD_GRAYSCALE)
    img = cv2.resize(src=img, dsize=(480,270)) 
    #img = cv2.GaussianBlur(src=img, ksize=(5,5), sigmaX=0)
    detector = ScaleDetector()
    previous = time()
    detector.detectScale(img=img)
    print(time()-previous)