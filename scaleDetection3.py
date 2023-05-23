import cv2
import numpy as np
from math import cos, sin, pi, atan

class ScaleDetector:

    def __init__(self):
        self.__radius = 0           #Distance between the ankor and the coordinate origin
        self.__delta = 10           #Radius step width
        self.__maxRadius = 0

        self.__alpha = -pi/2        #Angle between the x-axis and a line through the coordinate origin and the ankor
        self.__phi = 0.01           #Angle step width

        self.__ankorDirectionVector = np.array([0,1], dtype=np.float32)             #Unity vector facing from the origin to the ankor
        self.__lineDirectionVector = self.__ankorDirectionVector[::-1] * [1,-1]     #Unity vector orgthogonal to the ankorDirectionVector

    def __nextLine(self, shape):
        self.__radius += self.__delta
        if self.__maxRadius < self.__radius:
            self.__radius = self.__delta
            self.__alpha += self.__phi
            self.__ankorDirectionVector = np.array([cos(self.__alpha), sin(self.__alpha)], dtype=np.float32)
            self.__lineDirectionVector = self.__ankorDirectionVector[::-1] * [1,-1]
            self.__maxRadius = np.hypot(shape[0], shape[1]) * cos(abs(self.__alpha) - atan(shape[0]/shape[1]))

    @staticmethod        
    def calculateLineEnds(shape, ankor, vector):
        j = 0
        ends = np.zeros(shape=(2,2), dtype=np.int16)

        if vector[1] != 0:
            s = (ankor[0]*vector[1]-ankor[1]*vector[0])/vector[1]
            if 0 <= s < shape[1]:
                ends[j] = [int(s), 0]
                j += 1

        if vector[0] != 0:
            s = (ankor[1]*vector[0]+(shape[1]-ankor[0])*vector[1])/vector[0]
            if 0 <= s < shape[0]:
                ends[j] = [shape[1], int(s)]
                j += 1

        if j < 2 and vector[1] != 0:
            s = (ankor[0]*vector[1]-(ankor[1]-shape[0])*vector[0])/vector[1]
            if 0 < s <= shape[1]:
                ends[j] = [int(s), shape[0]]
                j += 1

        if j < 2 and vector[0] != 0:
            s = (ankor[1]*vector[0]-ankor[0]*vector[1])/vector[0]
            if 0 < s <= shape[0]:
                ends[j] = [0, int(s)]
        
        return ends

    @staticmethod
    def calculateLineIndices(ends):
        lineVector = ends[0]-ends[1]
        angle = np.arctan2(lineVector[1],lineVector[0])
        length = np.ceil(np.linalg.norm(lineVector))
        return ([cos(angle), sin(angle)] * np.reshape(np.arange(0, length+1, 1, dtype=np.int16), (-1,1)) + ends[1]).astype(np.uint16)

    @staticmethod
    def analyseLine(line):
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
                    return edges[index], edges[index+max(lengths)]

    def __show(self):
        self.__snap = self.__img.copy()
        cv2.circle(img=self.__snap, center=self.__ends[0], radius=3, color=(255,0,0), thickness=-1)
        cv2.circle(img=self.__snap, center=self.__ends[1], radius=3, color=(255,0,0), thickness=-1)
        cv2.circle(img=self.__snap, center=self.__p.astype(np.int32), radius=3, color=(0,255,0), thickness=-1)

        cv2.line(img=self.__snap, pt1=self.__ends[0], pt2=self.__ends[1], color=(0,0,255), thickness=1)

        cv2.imshow('Image', self.__snap)
        cv2.imshow('Line', np.tile(self.__line, (50,1)))
        cv2.waitKey(1)

    def detectScale(self, img):
        snap = img.copy()
        #img = cv2.cvtColor(src=img, code=cv2.COLOR_GRAY2BGR)
        while self.__alpha < pi/2:
            #snap = img.copy()
            self.__nextLine(shape=snap.shape)
            ankor = self.__radius * self.__ankorDirectionVector
            if self.__alpha < 0:
                ankor = ankor + [0, snap.shape[0]]
            ends = self.calculateLineEnds(shape=(snap.shape[0]-1,snap.shape[1]-1), ankor=ankor, vector=self.__lineDirectionVector)
            indices = self.calculateLineIndices(ends=ends)
            otsu = cv2.threshold(src=snap, thresh=150, maxval=255, type=cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
            line = self.analyseLine(line=otsu[indices[:,1], indices[:,0]])
            if line != None:
                cv2.line(img=snap, pt1=indices[line[0]], pt2=indices[line[1]], color=(0,255,0), thickness=1)
            #cv2.line(img=snap, pt1=ends[0], pt2=ends[1], color=(0,0,255), thickness=1)
            cv2.imshow("Image", snap)
            cv2.waitKey(0)

img = cv2.imread(filename='Repository/Images/image1.jpg', flags=cv2.IMREAD_GRAYSCALE)
img = cv2.resize(src=img, dsize=(480,270))
img = cv2.GaussianBlur(src=img, ksize=(5,5), sigmaX=0)
detector = ScaleDetector()

detector.detectScale(img=img)
