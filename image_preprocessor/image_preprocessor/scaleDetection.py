import cv2
import numpy as np
from math import cos, sin, pi, atan
from time import time

## A ScaleDetector instance is used to find a scale, consisting of n alternating black and white squares, in an image.
#
#  To work properly the following conditions must be met.
#  + The beginning of the scale points to the left.
#  + One square of the scale in the image is wider than one fortieth of the image width.
#  + More than 20 squares, starting from the beginning of the scale are fully visible, without a gap.

class ScaleDetector:

    ## Initializes a new ScaleDetector instance.
    #  @param delta1 Radius step width while the whole image is scanned.
    #  @param delta2 Orthogonal step width while the nearby area of a valid line is scanned.
    #  @param n Number of orthogonal steps done in each direction while scanning the nearby area.
    #  @param phi1 Angle step width while the whole image is scanned.
    #  @param phi2 Angle step width while the nearby area of a valid line is scanned.
    #  @param sigma1 Start angle step witdh for the border adaption.
    #  @param sigma2 Minimal angle step witdh for the border adaption.
    #  @param m Number of angle steps done in each direction while scanning the nearby area.
    #  @param count Number of gray value edges with minimum absolute distance and low varity in distance ratio in a row, 
    #               the line must go trough so it is assumed to go through the scale.
    #  @param accuracy The maximum value the ratio between the distances from an edge to the previous edge and the next edge can deviate from one.
    #  @param min The minimum distance between two edges.

    def __init__(self, delta1, delta2, n, phi1, phi2, sigma1, sigma2, m, count, accuracy):
        self.__delta1 = delta1
        self.__delta2 = delta2
        self.__n = n
        self.__phi1 = phi1
        self.__phi2 = phi2
        self.__sigma1 = sigma1
        self.__sigma2 = sigma2
        self.__m = m
        self.__count = count
        self.__accuracy = accuracy
        self.__min = 0

        self.__radius = 0                   #Distance between the anchor and the coordinate origin
        self.__maxRadius = 0
        self.__alpha = -pi/2                #Angle between the x-axis and a line through the coordinate origin and the anchor

        self.__img = None

    def __nextLine(self):
        self.__radius += self.__delta1
        if self.__maxRadius < self.__radius:
            self.__radius = self.__delta1
            self.__alpha = self.__phi1 - self.__alpha if 0 < self.__alpha else -self.__alpha
            if abs(self.__alpha) < self.__phi1:
                self.__alpha = -pi/2
                self.__maxRadius = 0
                return False  
            self.__maxRadius = np.hypot(self.__img.shape[0], self.__img.shape[1]) * cos(abs(self.__alpha) - atan(self.__img.shape[0]/self.__img.shape[1]))
        return True

    ## Calculates the intersection points between some lines and the border of an image.
    ## Each line is defined by an anchor point on the line and a vector of length one, giving the direction of the line.
    #  @param shape The shape of the image.
    #  @param anchors Array of anchor points.
    #  @param vectors Array of line vectors.
    #  @return Array of intersection points.
    @staticmethod
    def calculateLineEnds(shape, anchors, vectors):
        j = np.zeros(anchors.shape[0], dtype=np.uint8)
        ends = np.zeros(shape=(anchors.shape[0],2,2), dtype=np.float32)

        i = np.arange(0,vectors.shape[0],1)[vectors[:,1] != 0]
        if 0 < i.shape[0]:
            s = (anchors[i,0]*vectors[i,1]-anchors[i,1]*vectors[i,0])/vectors[i,1]
            x = np.logical_and(0<=s, s<shape[1]-1)
            i = i[x]
            if 0 < i.shape[0]:
                k = np.arange(0, s.shape[0], 1)[x]
                ends[i,j[i],0] = s[k]
                ends[i,j[i],1] = 0
                j[i] += 1

        i = np.arange(0,vectors.shape[0],1)[vectors[:,0] != 0]
        if 0 < i.shape[0]:
            s = (anchors[i,1]*vectors[i,0]+(shape[1]-1-anchors[i,0])*vectors[i,1])/vectors[i,0]
            x = np.logical_and(0<=s, s<shape[0]-1)
            i = i[x]
            if 0 < i.shape[0]:
                ends[i,j[i],0] = shape[1]-1
                k = np.arange(0, s.shape[0], 1)[x]
                ends[i,j[i],1] = s[k]
                j[i] += 1

        i = np.arange(0,vectors.shape[0],1)[np.logical_and(vectors[:,1] != 0, j < 2)]
        if 0 < i.shape[0]:
            s = (anchors[i,0]*vectors[i,1]-(anchors[i,1]-shape[0]+1)*vectors[i,0])/vectors[i,1]
            x = np.logical_and(0<s, s<=shape[1]-1)
            i = i[x]
            if 0 < i.shape[0]:
                k = np.arange(0, s.shape[0], 1)[x]
                ends[i,j[i],0] = s[k]
                ends[i,j[i],1] = shape[0]-1
                j[i] += 1

        i = np.arange(0,vectors.shape[0],1)[np.logical_and(vectors[:,0] != 0, j < 2)]
        if 0 < i.shape[0]:
            s = (anchors[i,1]*vectors[i,0]-anchors[i,0]*vectors[i,1])/vectors[i,0]
            x = np.logical_and(0<s, s<=shape[0]-1)
            i = i[x]
            if 0 < i.shape[0]:
                ends[i,j[i],0] = 0
                k = np.arange(0, s.shape[0], 1)[x]
                ends[i,j[i],1] = s[k]
        
        return ends

    ## Calculates the indices of all pixels on a line between two end pixels.
    #  @param ends The end pixels of the line.
    #  @return Array of pixel indices.
    @staticmethod
    def calculateLine(ends, anchor):
        lineVector = ends[0]-ends[1]
        length = np.linalg.norm(lineVector)
        if length == 0:
            return ends[0:1]
        lineVector = lineVector / length
        distances = (ends - anchor) @ lineVector
        roundedDistances = np.rint(distances)
        return lineVector * np.reshape(np.arange(0, roundedDistances[0]-roundedDistances[1] + 1, 1, dtype=np.float32), (-1,1)) + ends[1] + lineVector * (roundedDistances[1] - distances[1])

    ## Calculates the positions of the edges between black and white in the binary gray values of a line and 
    ## checks on basis of the distances between them, if a line is likely to go through <count> squares of the scale.
    #  @param ends The end points of the line.
    #  @return If the check is positive, the positions where it was assumed that the line intersects the edges between white and black squares.
    def analyseLine(self, ends, anchor):
        points = ScaleDetector.calculateLine(ends=ends, anchor=anchor)
        indices = np.rint(points).astype(np.int16)
        line = self.__img[indices[:,1], indices[:,0]]
        edges = np.arange(0, line.shape[0]-1, 1)[line[1:] != line[0:-1]]
        if self.__count < edges.shape[0]:
            distances = edges[1:] - edges[0:-1]
            ratios = distances[1:] / distances[0:-1]
            areas = np.all([1-self.__accuracy < ratios, ratios < 1+self.__accuracy, self.__min < distances[1:]], axis=0)
            areas = np.concatenate(([False], areas, [False]))
            borders = np.arange(0, areas.shape[0]-1, 1)[areas[1:] != areas[0:-1]]
            if 1 < borders.shape[0]:
                lengths = (borders[1:] - borders[0:-1])[areas[1:][borders][0:-1]]
                if self.__count < max(lengths):
                    index = borders[np.argmax(lengths)]+1
                    return points[edges[index:index+max(lengths)]]

    ## Checks the nearby area of a valid line for another valid line, that is assumed to intersect 
    ## an edge between a black and a white scale square further left in the image.
    #  @return The positions where it was assumed that the line with the most left intersection, intersects the edges between white and black squares.
    def scanArea(self, center):
        angles = np.linspace(self.__alpha-self.__m*self.__phi2, self.__alpha+self.__m*self.__phi2, 2*self.__m+1)
        anchorVectors = np.stack((np.cos(angles), np.sin(angles)), axis=-1)
        lineVectors = anchorVectors[:,::-1]*[1,-1]
        anchors = center + np.reshape(np.outer(np.linspace(-self.__n*self.__delta2, self.__n*self.__delta2, 2*self.__n+1), anchorVectors), (-1,2))
        ends = ScaleDetector.calculateLineEnds(shape=self.__img.shape, anchors=anchors, vectors=np.tile(lineVectors, (2*self.__n+1,1)))
        line = None
        for i in range(ends.shape[0]):
            edges = self.analyseLine(ends=ends[i], anchor=anchors[i])
            if edges is not None and (line is None or min(edges[:,0]) < min(line[:,0])):
                line = edges
        return line

    def adaptBorders(self, left, right):
        angle = np.arctan2(right[1]-left[1], right[0]-left[0])
        angles = np.array([angle, angle], dtype=np.float32)
        borders = np.zeros((2,self.__count+1,2), dtype=np.float32)
        borders[:,0], borders[:,-1] = left, right
        stepWidths = np.array([self.__sigma1, -self.__sigma1, -self.__sigma1, self.__sigma1], dtype=np.float32)
        i = -1
        while 0 < np.sum(abs(stepWidths)):
            i = (i + 1) % 4
            j = [i%2,0 if i<2 else -1]
            if stepWidths[i] == 0:
                continue
            angle = angles[i%2]+stepWidths[i]
            vector = np.array([[cos(angle), sin(angle)]], dtype=np.float32)
            ends = ScaleDetector.calculateLineEnds(shape=self.__img.shape, anchors=borders[j[0],[j[1]]], vectors=vector)[0]
            edges = self.analyseLine(ends=ends, anchor=borders[j[0],j[1]])
            if edges is not None:
                edges = edges[borders[j[0],j[1],0]-1 <= edges[:,0] if i < 2 else edges[:,0] <= borders[j[0],j[1],0]+1]
                if self.__count < edges.shape[0] and (np.linalg.norm(borders[j[0],j[1]] - edges[0]) < 2 or np.linalg.norm(borders[j[0],j[1]] - edges[-1]) < 2):
                    borders[j[0]] = edges[0:self.__count+1] if edges[0,0] < edges[-1,0] else edges[-1:-self.__count-2:-1]
                    angles[j[0]] = angle
                    continue
            if stepWidths[i] == -stepWidths[(i + 2) % 4] or abs(stepWidths[i] / 2) < self.__sigma2:
                stepWidths[i] = 0
            else:
                stepWidths[i] /= 2
                stepWidths[(i + 2) % 4] = -stepWidths[i]
        return borders

    def detectScale(self, img):
        self.__img = img
        self.__min = img.shape[1] / self.__count / 2.5
        while self.__nextLine():
            anchor = self.__radius * np.array([cos(self.__alpha), sin(self.__alpha)], dtype=np.float32)
            if self.__alpha < 0:
                anchor = anchor + [0, img.shape[0]]
            ends = ScaleDetector.calculateLineEnds(shape=img.shape, anchors=np.array([anchor]), vectors=np.array([[sin(self.__alpha), -cos(self.__alpha)]], dtype=np.float32))[0]
            line = self.analyseLine(ends=ends, anchor=anchor)
            if line is not None:
                left = self.scanArea(center=line[0]+(line[self.__count]-line[0])//2 if line[0,0] < line[-1,0] else line[-1]+(line[-self.__count-1]-line[-1])//2)
                if left is not None:
                    borders = self.adaptBorders(left=left[0 if left[0,0] < left[-1,0] else -1], right=left[self.__count if left[0,0] < left[-1,0] else -self.__count-1])
                    return borders
