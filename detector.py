import cv2
import numpy as np
import os


class ObjectDetector:
  def __init__(self):
    self.id_prev = -1
    self.detections = []
    
    
  
  def detectObject(self, img):
    self.detections = []
    contours, _=cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
      # Calculate area and remove small elements
      area = cv2.contourArea(cnt)
      if area > 1000:
        # Rectangle bounding box
        x, y, w, h = cv2.boundingRect(cnt)
        self.detections.append([x, y, w, h])
        
  
  

  def getSample(self, img, x_min, x_max, cx, id, path = "C:/Users/jerem/Desktop/Hochschule/Projekte/Projekt3/Workspace/Repository/Samples", saveSample = False):
    if (cx>=x_min and cx<=x_max) and (id != self.id_prev):
      # Create image of sample
      """ Change parameter """
      sample = img[0:300, cx-100:cx+100]
      cv2.imshow("sample", sample)
      # Save samples in folder 
      if saveSample == True:
        cv2.imwrite(os.path.join(path, "sample_test_"+str(id)+".jpg"), sample)
      cv2.waitKey(1)
      self.id_prev = id
      return sample
    else:
      return
