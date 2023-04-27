import cv2
import numpy as np
import math
from tracker import*

# Create tracker object
tracker = EuclideanDistTracker()

# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name
cap = cv2.VideoCapture('Repository/movingLong1.mp4')
 
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
    pts1 = np.float32([[230,280], [850,320], [230,375], [850,398]])
    pts2 = np.float32([[0,0],[1200,0],[0,300],[1200,300]])
    H = cv2.getPerspectiveTransform(pts1,pts2)
    warped = cv2.warpPerspective(frame, H,(1200,300))
   
    gray = cv2.cvtColor(src=warped, code=cv2.COLOR_BGR2GRAY)
    gaussianblur = cv2.GaussianBlur(src=gray, ksize=(5, 5), sigmaX=0)
    otsu_threshold, otsu = cv2.threshold(gaussianblur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU,)
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
        cv2.drawContours(warped, [cnt], -1, (255, 0, 255), 2)

        # Draw Rectangle bounding box
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(warped, (x,y), (x + w, y + h), (0, 255, 0), 3)

        detections.append([x, y, w, h])

    # 2. Object tracking
    boxes_ids = tracker.update(detections)
    for box_id in boxes_ids:
      x, y, w, h, id = box_id
      cv2.rectangle(warped, (x,y), (x + w, y + h), (0, 255, 0), 3)
      cv2.putText(warped, str(id), (x + 5, y + 15), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)
      # Draw the calculated speed
      cv2.putText(warped, str(tracker.getsp(id)), (x + 5, y + 40), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 1)
      


    # Speed detection lines
    cv2.line(warped, (150, 0), (150, 540), (0, 0, 255), 2)
    cv2.line(warped, (160, 0), (160, 540), (0, 0, 255), 2)

    cv2.line(warped, (300, 0), (300, 540), (0, 0, 255), 2)
    cv2.line(warped, (310, 0), (310, 540), (0, 0, 255), 2)

             
              

    # Display the resulting frame
    cv2.imshow("warped", warped) 
    cv2.imshow("binary", otsu)

    
    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
 
  # Break the loop
  else: 
    break
 
# When everything done, release the video capture object
cap.release()
 
# Closes all the frames
cv2.destroyAllWindows()