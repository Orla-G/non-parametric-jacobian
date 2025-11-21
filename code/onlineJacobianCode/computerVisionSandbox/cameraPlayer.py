import numpy as np
import cv2

cap = cv2.VideoCapture(-1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640*2)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480*2)
cap.set(cv2.CAP_PROP_FPS, 60)

if not cap.isOpened():
    print "ERROR: Camera failed to open."
    exit()
else:
	print "Video Size: %d x %d  @ %d  fps " % (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT), cap.get(cv2.CAP_PROP_FPS))
	#cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
	#cap.get(cv2.CAP_PROP_FPS)
	 

while(cap.isOpened()):
    # Capture frame-by-frame
    ret, frame = cap.read()
    #cv2.waitKey(1) # wait a little for frame acquisition?
    # Our operations on the frame come here
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if ret == True:
        # Display the resulting frame
        #cv2.imshow('frame',gray)
        cv2.imshow('frame', frame)


    else:
        print "Failed to capture video frame"
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
