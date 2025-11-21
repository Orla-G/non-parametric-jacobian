#import everything we need
import time
import cv2.aruco as A
import cv2
import numpy as np
import json

#setup aruco dictionary
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(5,5,.0475,.0238,dictionary)
img = board.draw((200*3,200*3))

#Dump the calibration board to a file
cv2.imwrite('charuco.png',img)

#Setup image capture
cap = cv2.VideoCapture("front45.avi")
#cap = cv2.VideoCapture(0)#1 is the camera on my computer, can change
allCorners = []
allIds = []
decimator = 0

#check if camera calibration file exists and load variables
try:
	with open('cameraCalibrationData.json', 'r') as f:	
		print "Opening calibration file"
		datastore = json.load(f)
		cameraMatrix = np.array(datastore['cameraMatrix'])
	#need to convert matrix to numpy array so use np.array(data)
	print "Retrieved cameraMatrix"
	distCoeffs = np.array(datastore['distCoeffs'])
	print "Retrieved distCoeffs"
	
	# #I'm not sure if we need any of the following variables
	# retval = np.asarray(datastore['retval'])
	# print "Retrieved retval"
	# rvecs = np.asarray(datastore['rvecs'])
	# print "Retrieved rvecs"
	# tvecs = np.asarray(datastore['tvecs'])
	# print "Retrieved tvecs"
	
	
	
	
#attempt camera calibration if no file exists	
except Exception as e: 
	#the next line prints the error that caused the exception
	#print(e)
	print "Running calibration"

	#check if video file can be opened
	if not cap.isOpened():
		print "ERROR: Video source Failed to open..."
		exit()

	#Camera calibration video
	for i in range(300):

		ret,frame = cap.read()
		if ret != True:
			print "Quiting ..."
			break
		
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		res = cv2.aruco.detectMarkers(gray,dictionary)

		if len(res[0])>0:
			res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)
			if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%3==0:
				allCorners.append(res2[1])
				allIds.append(res2[2])

			cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])

		# Display video
		cv2.imshow('frame',gray)
		
		# end program if q is pressed
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		decimator+=1

	#Calibration fails for lots of reasons. Release the video if we do
	try:
		imsize = gray.shape
		retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)
		print "SUCCESS in calibration..."
		print cameraMatrix
		
		#tries to save the data to a file, if it fails we can still continue
		try: 
			# if I get the below code working I can dump out the info to a file
			#need to convert numpy data to matrix
			#data = {"retval": np.asmatrix(retval),"cameraMatrix": np.asmatrix(cameraMatrix), "distCoeffs": np.asmatrix(distCoeffs), "rvecs": np.asmatrix(rvecs), "tvecs": np.asmatrix(tvecs)}
			data = {"cameraMatrix": cameraMatrix.tolist(), "distCoeffs": distCoeffs.tolist()}
			with open("cameraCalibrationData.json", "w") as outfile: 
				json.dump(data,outfile)	
		
		#except happens if error occurs in saving data to JSON and lets you know the file didn't save
		except Exception as e: 
			print(e)
			#do nothing
			print "Did not save file"
		
	except: #calibration failed, end session
		cap.release()
		print "CALIBRATION FAILED"


#display and track markers
pos_frame = cap.get(cv2.CAP_PROP_POS_FRAMES)
while True:
    # Capture frame-by-frame
        ret, frame = cap.read()

        if ret:
                # Detect Marker
                res = cv2.aruco.detectMarkers(frame,dictionary)
                print(res)

                #if markers are detected track and display axis
                if len(res[0])>0:
                        print("b")
                        res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],frame,board)
                        if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%3==0:
                                allCorners.append(res2[1])
                                allIds.append(res2[2])
                                
                        cv2.aruco.drawDetectedMarkers(frame,res[0],res[1])
                        rvec, tvec,_ = cv2.aruco.estimatePoseSingleMarkers(res[0], 0.0238,cameraMatrix,distCoeffs)
                        
                        #print axis for each aruco code
                        for j in range(len(res[0])):
                                cv2.aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvec[j], tvec[j], 0.03)
                                
                                #print xyz position for first marker, still figuring this out
                                #it might be this? 
                                # cv2.Rodrigues(rvec[j],R)
                                # np.transpose(R)
                                # tvec = -R*tvec[j]
                                # print(tvec)
                                
                                #or this? 
                                print(tvec[j])
                                
                
                # Display the resulting frame
                cv2.imshow('frame', frame)
                pos_frame = cap.get(cv2.CAP_PROP_POS_FRAMES)
        else:
                # The next frame is not ready, so we try to read it again
                cap.set(cv2.CAP_PROP_POS_FRAMES, pos_frame-1)
                print "frame is not ready"
                # It is better to wait for a while for the next frame to be ready
                cv2.waitKey(1000)

        # end program if q is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        if cap.get(cv2.CAP_PROP_POS_FRAMES) == cap.get(cv2.CAP_PROP_FRAME_COUNT):
        # If the number of captured frames is equal to the total number of frames,
        # we stop
                break
	
# release files and close all windows	
cap.release()
cv2.destroyAllWindows()
