import cv2
import numpy as np
from math import cos, sin, pi, atan

class ScaleDetector:

    def __init__(self, img, a, b):
        self.__img = img
        self.__shape = (self.__img.shape[0]-1, self.__img.shape[1]-1)
        self.__ratio = self.__shape[1]/self.__shape[0]
        self.__gamma = atan(1/self.__ratio)
        self.__a = a
        self.__b = b
        self.__ends = np.zeros(shape=(2,2), dtype=np.int16)
        self.__alpha = -pi/2
        self.__v = np.array([cos(self.__alpha), sin(self.__alpha)], dtype=np.float32)
        self.__vo = self.__v[::-1] * [1,-1]
        self.__n = int(np.hypot(self.__shape[0], self.__shape[1]) * cos(abs(self.__alpha)-self.__gamma) / self.__b)
        self.__i = 0
        self.__p = 0

    def calculateLineEnds(self):
        j = 0

        if self.__vo[1] != 0:
            s = (self.__p[0]*self.__vo[1]-self.__p[1]*self.__vo[0])/self.__vo[1]
            if 0 <= s < self.__shape[1]:
                self.__ends[j] = [int(s), 0]
                j += 1

        if self.__vo[0] != 0:
            s = (self.__p[1]*self.__vo[0]+(self.__shape[1]-self.__p[0])*self.__vo[1])/self.__vo[0]
            if 0 <= s < self.__shape[0]:
                self.__ends[j] = [self.__shape[1], int(s)]
                j += 1

        if j < 2 and self.__vo[1] != 0:
            s = (self.__p[0]*self.__vo[1]-(self.__p[1]-self.__shape[0])*self.__vo[0])/self.__vo[1]
            if 0 < s <= self.__shape[1]:
                self.__ends[j] = [int(s), self.__shape[0]]
                j += 1

        if j < 2 and self.__vo[0] != 0:
            s = (self.__p[1]*self.__vo[0]-self.__p[0]*self.__vo[1])/self.__vo[0]
            if 0 < s <= self.__shape[0]:
                self.__ends[j] = [0, int(s)]
    
    def nextLine(self):
        if self.__i == self.__n:
            self.__i = 1
            self.__alpha += self.__a
            self.__v = np.array([cos(self.__alpha), sin(self.__alpha)], dtype=np.float32)
            self.__vo = self.__v[::-1] * [1,-1]
            self.__n = int(np.hypot(self.__shape[0], self.__shape[1]) * cos(abs(self.__alpha)-self.__gamma) / self.__b)
        else:
            self.__i += 1
        
        self.__p = self.__i * self.__b * self.__v
        if self.__alpha < 0:
            self.__p = self.__p + [0, self.__shape[0]]

    def show(self):
        snap = self.__img.copy()
        cv2.circle(img=snap, center=self.__ends[0], radius=3, color=(255,0,0), thickness=-1)
        cv2.circle(img=snap, center=self.__ends[1], radius=3, color=(255,0,0), thickness=-1)
        cv2.circle(img=snap, center=self.__p.astype(np.int32), radius=3, color=(0,255,0), thickness=-1)

        cv2.line(img=snap, pt1=self.__ends[0], pt2=self.__ends[1], color=(0,0,255), thickness=1)

        line_vector = self.__ends[0]-self.__ends[1]
        angle = np.arctan2(line_vector[1],line_vector[0])
        length = np.ceil(np.linalg.norm(line_vector))
        indices = ([cos(angle), sin(angle)] * np.reshape(np.arange(0, length+1, 1, dtype=np.int16), (-1,1)) + self.__ends[1]).astype(np.uint16)
        otsu = cv2.threshold(src=org, thresh=150, maxval=255, type=cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
        line = otsu[indices[:,1], indices[:,0]]

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
                    cv2.line(img=snap, pt1=indices[edges[index]], pt2=indices[edges[index+max(lengths)]], color=(0,255,0), thickness=1)


        cv2.imshow('Otsu', otsu)
        cv2.imshow('Image', snap)
        cv2.imshow('Line', np.tile(line, (50,1)))
        cv2.waitKey(1)


org = cv2.imread(filename='Repository/Images/image1.jpg', flags=cv2.IMREAD_GRAYSCALE)
org = cv2.resize(src=org, dsize=(480,270))
org = cv2.GaussianBlur(src=org, ksize=(5,5), sigmaX=0)
detector = ScaleDetector(img=org, a=0.05, b=20)

while True:
    detector.nextLine()
    detector.calculateLineEnds()
    detector.show()
