############# SETUP STUFF ################

#Import everything we need
import time
import cv2.aruco as A
import cv2
import numpy as np
import json

############ GLOBAL VARIABLES HERE ############

#Setup Aruco Dictionary
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(5,5,.0475,.0238,dictionary)
img = board.draw((200*3,200*3))

#Dump the calibration board to a file
cv2.imwrite('charuco.png',img)

# Setup image capture
cap = cv2.VideoCapture(1) #1 is the camera on my computer, can change
allCorners = []
allIds = []
decimator = 0

############ TYPE PROGRAM HERE ############

def main():
    global cap
    global allCorners
    global allIds
    global decimator
    #Start calling functions
    decideCal()

    print("\nStarting to track marker")
    while(cap.isOpened()):
        trackMark()
        #end program if q is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break  
        
    # release files and close all windows	
    cap.release()
    cv2.destroyAllWindows()
        

############# CREATE FUNCTIONS HERE ###############

#### decide what to do about calibration data
def decideCal():
    print("Please choose from the following:")
    print("\n[1] Load Calibration")
    print("\n[2] Start Calibration")
    while True:
        try:
            data = int(raw_input("\nWhat would you like to do?   "))
        except ValueError:
            print("\nSorry that is not a valid choice")
            continue
        if data == 1 or data == 2:
            break
        else:
            print("\nSorry that is not a valid choice")
            continue
    if data == 1:
        print("\nLoading Calibration data")
        loadCal()
    else:
        print("\nStarting Calibration")
        startCal()

#### Load calibration file
def loadCal():
    try:
        with open('cameraCalibrationData.json', 'r') as f:	
            print "\nOpening calibration file"
            datastore = json.load(f)
            cameraMatrix = np.array(datastore['cameraMatrix'])
	#need to convert matrix to numpy array so use np.array(data)
        print "\nRetrieved cameraMatrix"
        distCoeffs = np.array(datastore['distCoeffs'])
        print "\nRetrieved distCoeffs"
    except:
        print("\nUnable to load calibration data, starting calibration")
        startCal()

#### Start Calibration from scratch
def startCal(): 
    global decimator
    global allCorners
    global allIds
    global cap
    print "\nRunning calibration"
    #check if video file can be opened
    if not cap.isOpened():
            print "\nERROR: Video source Failed to open..."
            exit()

    #Camera calibration video
    for i in range(300):

            ret,frame = cap.read()
            if ret != True:
                    print "\nQuiting ..."
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
        print "\nSUCCESS in calibration..."
            #tries to save the data to a file, if it fails we can still continue
        try: 
            #need to convert numpy data to matrix
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
        print "\nCALIBRATION FAILED"  

### Track Mark
def trackMark():
    global decimator
    global allCorners
    global allIds
    global cap
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    # Detect Marker
    res = cv2.aruco.detectMarkers(frame,dictionary)

    #if markers are detected track and display axis
    if len(res[0])>0:
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

########### RUN PROGRAM ##########
main()
        
