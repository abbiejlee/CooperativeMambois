import numpy as np
import cv2
from ColorSegmentation import cd_color_segmentation


cap = cv2.VideoCapture(0)

while(cap.isOpened()):
    # Capture frame-by-frame
    ret, frame = cap.read()

    [box_pt, ln_color] = cd_color_segmentation(frame)
    cv2.rectangle(frame, box_pt[0], box_pt[1], ln_color,3)
    
    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
