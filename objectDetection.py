import cv2
import numpy as np
import math
from tracker import*
import os


# Create Sample
saveSample = False
id_prev = -1
def getSampel(frame, cx, id):
  # Create image of sample
  """ Change parameter """
  sampel = frame[0:300, cx-100:cx+100]
  cv2.imshow("sampel", sampel)
  # Save samples in folder 
  if saveSample == True:
    path = "C:/Users/jerem/Desktop/Hochschule/Projekte/Projekt3/Workspace/Repository/Samples"
    cv2.imwrite(os.path.join(path, "sample_test_"+str(id)+".jpg"), sampel)
  print("sampel with id: "+str(id)) 
  cv2.waitKey(1)
  return sampel

# Create tracker object
tracker = EuclideanDistTracker()

# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name
cap = cv2.VideoCapture('Repository/Aufnahmen/MiauTrain.mp4')

 
# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")

# Read until video is completed
while(cap.isOpened()):
  # Capture frame-by-frame
  ret, frame = cap.read()
  if ret == True:
    
    #Bildvorverarbeitung
    #
    #
    #
    frame = cv2.resize(src=frame, dsize=(960, 540))
    """ Change parameter """
    pts1 = np.float32([[231,250], [721,295], [220,400], [710,420]])
    pts2 = np.float32([[0,0],[1200,0],[0,300],[1200,300]])
    H = cv2.getPerspectiveTransform(pts1,pts2)
    warped = cv2.warpPerspective(frame, H,(1200,300))

    roi = warped[88:277, 0:1200]
   
    gray = cv2.cvtColor(src=roi, code=cv2.COLOR_BGR2GRAY)
    gaussianblur = cv2.GaussianBlur(src=gray, ksize=(5, 5), sigmaX=0)
    otsu_threshold, otsu = cv2.threshold(gaussianblur, 200, 255, cv2.THRESH_BINARY)
    #otsu_threshold, otsu = cv2.threshold(gaussianblur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU,)
    #
    #
    #
    #Bildvorverarbeitung


    # 1. Object detection 
    detections = []
    contours, _=cv2.findContours(otsu, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
      # Calculate area and remove small elements
      area = cv2.contourArea(cnt)
      if area > 1000:
        # Draw contours
        cv2.drawContours(roi, [cnt], -1, (255, 0, 255), 2)

        # Rectangle bounding box
        x, y, w, h = cv2.boundingRect(cnt)
        detections.append([x, y, w, h])

    # 2. Object tracking
    boxes_ids = tracker.update(detections)
    for box_id in boxes_ids:
      x, y, w, h, id = box_id
      # Centerpoints
      cx = int((x + x + w)/2)
      cy = int((y + y + h)/2)
      # Create Sampel when object is at certain positon
      """ Change parameter """
      if (cx>=320 and cx<=325) and (id != id_prev):
        sample = getSampel(otsu, cx, id) 
        id_prev = id                

      # Finde die Konturen des binären Bildes
      contours, hierarchy = cv2.findContours(sample, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      for contour in contours:
        # Calculate area and remove small elements
        area = cv2.contourArea(contour)
        if area > 1000:
          # Bestimme die convexHull für jede Kontur und zeichne sie auf das Originalbild
          hull = cv2.convexHull(contour)
          cv2.drawContours(sample, [hull], 0, (0, 255, 0), 2)
          # Ausgabe der Anzahl der Kanten der Convex Hull
          num_edges = len(hull)
          print("Die Convex Hull hat", num_edges, "Kanten.")


          # Bestimme das minAreaRectangle für jede Kontur
          rect = cv2.minAreaRect(contour)
          box = cv2.boxPoints(rect)
          box = np.int0(box)
          print(box)
          #print("höhe: "+str(h)+" weite: "+str(w))
            
          # Zeichne das minAreaRectangle auf das Bild
          cv2.drawContours(sample,[box],0,(0,0,255),2)

      # Draw boundingbox with id
      """ Change parameter """
      cv2.rectangle(roi, (cx-100, 0), (cx + 100, 189), (0, 255, 0), 3)
      cv2.putText(roi, str(id), (x + 5, y + 15), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)
      # Show the calculated speed
      cv2.putText(roi, str(tracker.getsp(id)), (x + 5, y + 40), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 2)
      


    # Speed detection lines
    cv2.line(warped, (120, 0), (120, 540), (0, 0, 255), 1)
    cv2.line(warped, (130, 0), (130, 540), (0, 0, 255), 1)

    cv2.line(warped, (960, 0), (960, 540), (0, 0, 255), 1)
    cv2.line(warped, (970, 0), (970, 540), (0, 0, 255), 1)

             
              

    # Display the resulting frame
    #cv2.imshow("roi", roi)
    cv2.imshow("warped", warped) 
    cv2.imshow("binary", otsu)
    #cv2.imshow("sample", sample)

    
    # Press Q on keyboard to  exit
    if cv2.waitKey(0) & 0xFF == ord('q'):
      break
 
  # Break the loop
  else: 
    break
 
# When everything done, release the video capture object
cap.release()
 
# Closes all the frames
cv2.destroyAllWindows()