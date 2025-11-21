
############# SETUP STUFF ################

#Import everything we need
import time
import cv2.aruco as A
import cv2
import numpy as np
import json
import serial
import time
import sys

############ VARIABLES HERE, these will be passed around ############

#Setup Aruco Dictionary
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(5,5,.0475,.0238,dictionary)
img = board.draw((200*3,200*3))

#Dump the calibration board to a file
cv2.imwrite('charuco.png',img)

# Setup image capture
cap = cv2.VideoCapture(1) #0, front camera, 1 is webcam or back camera
allCorners = []
allIds = []
decimator = 0

# NOTE the user must ensure that the serial port and baudrate are correct
serPort = "COM6"
baudRate = 9600
ser = serial.Serial(serPort, baudRate)
print "Serial port " + serPort + " opened  Baudrate " + str(baudRate)

startMarker = 60
endMarker = 62

############ TYPE PROGRAM HERE ############

def main(decimator, allCorners, allIds, cap, dictionary,board):
    #Call functions to get calibration data
    cameraMatrix, distCoeffs = decideCal(decimator, allCorners, allIds, cap, dictionary, board)

    #Call functions to track marker
    trackOpt(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary,board)  
        
    # release files and close all windows	
    cap.release()
    cv2.destroyAllWindows()
        

############# CREATE FUNCTIONS HERE ###############

#### decide what to do about calibration data
def decideCal(decimator, allCorners, allIds, cap, dicitionary, board):
    print("Please choose from the following:")
    print("\n[1] Load Calibration")
    print("[2] Start Calibration")
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
        cameraMatrix, distCoeffs = loadCal(decimator, allCorners, allIds, cap)
    else:
        print("\nStarting Calibration")
        cameraMatrix, distCoeffs = startCal(decimator, allCorners, allIds, cap, dictionary, board)
    return cameraMatrix, distCoeffs

#### Load calibration file
def loadCal(decimator, allCorners, allIds, cap):
    try:
        with open('cameraCalibrationData.json', 'r') as f:	
            print "\nOpening calibration file"
            datastore = json.load(f)
            cameraMatrix = np.array(datastore['cameraMatrix'])
	#need to convert matrix to numpy array so use np.array(data)
            print "\nRetrieved cameraMatrix"
            distCoeffs = np.array(datastore['distCoeffs'])
            print "\nRetrieved distCoeffs"
        return (cameraMatrix, distCoeffs)
    except:
        print("\nUnable to load calibration data, starting calibration")
        startCal(decimator, allCorners, allIds, cap)

#### Start Calibration from scratch
def startCal(decimator, allCorners, allIds,cap, dictionary, board): 
    print "\nRunning calibration"
    #check if video file can be opened
    if not cap.isOpened():
            print "\nERROR: Video source Failed to open..."
            sys.exit()

    #Camera calibration video
    for i in range(1000):

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
            return(cameraMatrix, distCoeffs)              
        #except happens if error occurs in saving data to JSON and lets you know the file didn't save
        except Exception as e: 
            print(e)
            #do nothing
            print "Did not save file"
    except: #calibration failed, end session
        cap.release()
        print "\nCALIBRATION FAILED"
        decideCal(decimator, allCorners, allIds, cap, dicitionary, board)

### Choose what to do with tracking
def trackOpt(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board):
    print("***************************************************")
    print("\nPlease choose from the following:")
    print("\n[1] Load Initial Jacobian")
    print("[2] Find Initial Jacobian")
    print("[3] Track Single Marker in 2D")
    print("[4] Track Single Marker in 3D") 
    print("[5] Exit Program")
    while True:
        try:
            data = int(raw_input("\nWhat would you like to do?   "))
        except ValueError:
            print("\nSorry that is not a valid choice")
            continue
        if data > 0 and data < 6:
            break
        else:
            print("\nSorry that is not a valid choice")
            continue
    if data == 1:
        print("\nLoading Initial Jacobian")
        Ji = loadJi(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board)
        decideControl(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board)
    elif data == 2:
        print("\nStarting to Find Initial Jacobian")
        Ji = find2DJi(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board)
        decideControl(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board)
    elif data == 3:
        print("\nTracking single marker in 2D")
        trackMark2D(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board)
    elif data == 4:
        print("\nTracking single marker in 3D")
        trackMark3D(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board)
    else:
        print("Exiting the program, thank you")
        cap.release()
        cv2.destroyAllWindows()
        sys.exit()
    return ()

### decide to control robot or not
def decideControl(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board):
    print("***************************************************")
    print("\nPlease choose from the following:")
    print("\n[1] Control Robot")
    print("[2] Exit")
    while True:
        try:
            data = int(raw_input("\nWhat would you like to do?   "))
        except ValueError:
            print("\nSorry that is not a valid choice")
            continue
        if data > 0 and data < 3:
            break
        else:
            print("\nSorry that is not a valid choice")
            continue
    if data == 1:
        print("\nControlling Robot")
        controlRobot(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board)
    else:
        print("\nThanks for visiting!")
    return ()

### Load Initial Jacobian
def loadJi(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board):
    try:
        with open('initialJacobian.json', 'r') as f:	
            print "\nOpening initial Jacobian file"
            datastore = json.load(f)
            Ji = np.array(datastore['Ji'])
	#need to convert matrix to numpy array so use np.array(data)
            print "\nRetrieved initiall Jacobian it is...."
            print Ji
        return Ji
    except Exception as e:
        print e
        print("\nUnable to load initial Jacobian, please choose again")
        trackOpt(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board)

### Find initial 2D Jacobian
def find2DJi(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board):
    #Place holder while I figure code out
    Ji = np.array([0])

    waitForArduino()

    xdiff = np.array([])
    ydiff = np.array([])
    #the initial xlast and ylast should be set to whatever the first frame is
    xlast = 0
    ylast = 0

    q1diff = []
    q2diff = []
    q3diff = []
    q1last = 180
    q2last = 0
    q3last = 180

    print("\nStarting to find initial Jacobian")
    cap.isOpened() 
    while(cap.isOpened()):

        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # Detect Marker
        res = cv2.aruco.detectMarkers(frame,dictionary)

        #if markers are detected track and display axis
        if len(res[0])>0:
            sendToArduino("a")

            ### GET ARDUINO DATA HERE!
            q1, q2, q3 = getServoPos()
            if q1 == 1000: #arduino sets q1, q2, q3 to 1000 when it is done running through program once
                break
            q1diff = np.append(q1diff, q1-q1last)
            q2diff = np.append(q2diff, q2-q2last)
            q3diff = np.append(q3diff, q3-q3last)
            q1last = q1
            q2last = q2
            q3last = q3
            cv2.aruco.drawDetectedMarkers(frame,res[0],res[1])
            corners = res[0]
            #Get change in XY position in matrix form
            x = (corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]) / 4
            y = (corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]) / 4 
            xappend = x-xlast
            yappend = y-ylast
            xdiff = np.append(xdiff, xappend)
            ydiff = np.append(ydiff, yappend)
            xlast = x
            ylast = y
            
        # Display the resulting frame
        cv2.imshow('frame', frame)
        
        #end program if q is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    ser.close
    cap.release()
    cv2.destroyAllWindows()
    #Matrix of change in XY position and Q's over time
    B = np.vstack((xdiff,ydiff)) #2xN where row 1 is xdiff and row 2 is ydiff
    mX = np.vstack((q1diff, q2diff, q3diff)) #3xN where row 1 is q1diff, row 2 is q2diff, row 3 is q3diff
    
    #Check that some data was found to calculate initial Jacobian with
##    if B.size < 45*5: #20 is product of array dimensions
##        print("\nNo data found to solve for initial Jacobian Matrix")
##        ser.close
##        cap.release()
##        cv2.destroyAllWindows()
##        trackOpt(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board)

    #calculate Jacobian
    #B and mX are numpy arrays that are 3xN
    mXmXT = np.dot(mX,mX.transpose())

    #need to make sure that we have a matrix with non zero rows in order for .inv to work 
    imX = np.linalg.inv(mXmXT)
    inter = np.dot(B,mX.transpose())
    Ji = np.dot(inter,imX)
    print("Ji found to be....")
    print(Ji)
    
    #Try and save Ji to file
    try: 
        #need to convert numpy data to matrix
        data = {"Ji": cameraMatrix.tolist()}
        with open("initialJacobian.json", "w") as outfile: 
                json.dump(data,outfile)
        print("Initial Jacobian Saved to File") 
    #except happens if error occurs in saving data to JSON and lets you know the file didn't save
    except Exception as e: 
        print(e)
        #do nothing
        print "Did not save Ji to file"
    return Ji

### Track Mark
def trackMark2D(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board):
    print("\nStarting to track marker")
    while(cap.isOpened()):

        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # Detect Marker
        res = cv2.aruco.detectMarkers(frame,dictionary)

        #if markers are detected track and display axis
        if len(res[0])>0:
            corners = res[0]              
            x = (corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]) / 4
            y = (corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]) / 4 
            print x,",", y
            # Display the resulting frame
            cv2.aruco.drawDetectedMarkers(frame,res[0],res[1])

        # Display the resulting frame
        cv2.imshow('frame', frame)

        #end program if q is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    #return values
    return ()

def trackMark3D(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board):
    print("\nStarting to track marker in 3D")
    print("CODE DOES NOT ACCURATELY WORK YET")
    while(cap.isOpened()):

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
            
            #looking for just one marker
            cv2.aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvec, tvec, 0.03)
            
            #print xyz position for first marker, still figuring this out
            #it might be this? 
            # cv2.Rodrigues(rvec[j],R)
            # np.transpose(R)
            # tvec = -R*tvec[j]
            # print(tvec)
            
            #or this? 
            print(tvec)
                        
        
        # Display the resulting frame
        cv2.imshow('frame', frame)

        #end program if q is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    #return values
    return ()

### Control Robot Function
def controlRobot(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board):
    print("This currently does not do anything, exiting program")
    return()

########### ARDUINO COMMUNICATION FUNCTIONS #########

###
def sendToArduino(sendStr):
  ser.write(sendStr)

###
def recvFromArduino():
  global startMarker, endMarker
  
  ck = ""
  x = "z" # any value that is not an end- or startMarker
  byteCount = -1 # to allow for the fact that the last increment will be one too many
  
  # wait for the start character
  while  ord(x) != startMarker: 
    x = ser.read()
  
  # save data until the end marker is found
  while ord(x) != endMarker:
    if ord(x) != startMarker:
      ck = ck + x 
      byteCount += 1
    x = ser.read()
  
  return(ck)

###
def waitForArduino():

   # wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
   # it also ensures that any bytes left over from a previous message are discarded
   
    global startMarker, endMarker
    
    msg = ""
    while msg.find("Arduino is ready") == -1:

      while ser.inWaiting() == 0:
        pass
        
      msg = recvFromArduino()

      print msg
      print

### Get servo position
def getServoPos():
    msg = recvFromArduino()
    time, q1, q2, q3 = msg.split(',')
    q1 = int(q1)
    q2 = int(q2)
    q3 = int(q3)
    return(q1, q2, q3)

########### RUN PROGRAM ##########
main(decimator, allCorners, allIds, cap, dictionary, board)
