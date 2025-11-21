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
cap = cv2.VideoCapture(0) #0, front camera, 1 is webcam or back camera
allCorners = []
allIds = []
decimator = 0

# NOTE the user must ensure that the serial port and baudrate are correct
serPort = "COM4"
baudRate = 9600
ser = serial.Serial(serPort, baudRate)
print "Serial port " + serPort + " opened  Baudrate " + str(baudRate)

startMarker = 60
endMarker = 62

#final path destination
x_f = []
y_f = []

############ TYPE PROGRAM HERE ############

def main(decimator, allCorners, allIds, cap, dictionary, board):
    #Call functions to get calibration data
    cameraMatrix, distCoeffs = decideCal(decimator, allCorners, allIds, cap, dictionary, board)

    #Call functions to track marker
    trackOpt(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary,board)  
        
    # release files and close all windows	
    cap.release()
    cv2.destroyAllWindows()
        

############# CREATE FUNCTIONS HERE ###############

#### decide what to do about calibration data
def decideCal(decimator, allCorners, allIds, cap, dictionary, board):
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
        cameraMatrix, distCoeffs = loadCal(decimator, allCorners, allIds, cap, dictionary, board)
    else:
        print("\nStarting Calibration")
        cameraMatrix, distCoeffs = startCal(decimator, allCorners, allIds, cap, dictionary, board)
    return cameraMatrix, distCoeffs

#### Load calibration file
def loadCal(decimator, allCorners, allIds, cap, dictionary, board):
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
        startCal(decimator, allCorners, allIds, cap, dictionary, board)

#### Start Calibration from scratch
def startCal(decimator, allCorners, allIds, cap, dictionary, board): 
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
        decideControl(cap,dictionary, Ji)
    elif data == 2:
        print("\nStarting to Find Initial Jacobian")
        Ji = find2DJi(cap, dictionary)
        decideControl(cap,dictionary, Ji)
    elif data == 3:
        print("\nTracking single marker in 2D")
        trackMark2D(cap,dictionary)
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
def decideControl(cap,dictionary, Ji):
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
        ControlRobot(cap, dictionary, Ji)
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
            print "\nRetrieved initial Jacobian it is...."
            print Ji
        return Ji
    except Exception as e:
        print e
        print("\nUnable to load initial Jacobian, please choose again")
        trackOpt(decimator, allCorners, allIds, cap, cameraMatrix, distCoeffs, dictionary, board)

### Find initial 2D Jacobian
def find2DJi(cap, dictionary):
    #Place holder while I figure code out
    Ji = np.array([0])

    # use a state variabel to have first frame read be differnt than other frames read
    state = 0
    waitForArduino()

    xdiff = np.array([])
    ydiff = np.array([])
    #the initial xlast and ylast should be set to whatever the first frame is
    xlast = 0
    ylast = 0

    # initial q variables
    q1diff = []
    q2diff = []
    q3diff = []
    q1last = 900
    q2last = 900
    q3last = 2300

    print("\nStarting to find initial Jacobian")
    cap.isOpened() 
    while(cap.isOpened()):

        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # Detect Marker
        res = cv2.aruco.detectMarkers(frame,dictionary)

        #if markers are detected track and display axis
        if len(res[0])>0:
            if state == 0:
                corners = res[0]
                #Get change in XY position in matrix form
                xlast, ylast = getXY(res[0])   
                state = 1
            else:
                #Get XY Data here!
                cv2.aruco.drawDetectedMarkers(frame,res[0],res[1])
                corners = res[0]
                #Get change in XY position in matrix form
                x, y = getXY(res[0])
                #print(x,y)
                xdiff = np.append(xdiff, x-xlast)
                ydiff = np.append(ydiff, y-ylast)
                xlast = x
                ylast = y
                
            #send random character to arduino to unpause program
            sendToArduino("a")

            ### GET ARDUINO DATA HERE!
            q1, q2, q3 = getServoPos()
            if q1 == 9999: #arduino sets q1, q2, q3 to 1000 when it is done running through program once
                break
            q1diff = np.append(q1diff, q1-q1last)
            q2diff = np.append(q2diff, q2-q2last)
            q3diff = np.append(q3diff, q3-q3last)
            q1last = q1
            q2last = q2
            q3last = q3
            print(q1,q2,q3)


            
        # Display the resulting frame
        cv2.imshow('frame', frame)
        
        #end program if q is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    #Matrix of change in XY position and Q's over time
    B = np.vstack((xdiff,ydiff)) #2xN where row 1 is xdiff and row 2 is ydiff
    mX = np.vstack((q1diff, q2diff, q3diff)) #3xN where row 1 is q1diff, row 2 is q2diff, row 3 is q3diff

    #calculate Jacobian
    #B = 2xN array that is [x;y] and mX 3xN array that is [q1;q2;q3]
    #realized I could use the pseudo inverse, much easier!
    imX = np.linalg.pinv(mX)
    Ji = np.dot(B,imX)
    print("Ji found to be....")
    print(Ji)
    
    #Try and save Ji to file
    try: 
        #need to convert numpy data to matrix
        data = {"Ji": Ji.tolist(), "Delta_EE": B.tolist(), "Delta_Qs": mX.tolist()}
        with open("initialJacobian.json", "w") as outfile: 
                json.dump(data,outfile)
        print("Initial Jacobian Saved to File") 
    #except happens if error occurs in saving data to JSON and lets you know the file didn't save
    except Exception as e: 
        print(e)
        #do nothing
        print "Did not save Ji to file"
    return Ji

### Get center of Aruco marker
def getXY(corners):
    #Get change in XY position in matrix form
    x = (corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]) / 4
    y = (corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]) / 4
    return(x,y)

### Track Mark
def trackMark2D(cap,dictionary):
    print("\nStarting to track marker")
    while(cap.isOpened()):

        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # Detect Marker
        res = cv2.aruco.detectMarkers(frame,dictionary)

        #if markers are detected track and display axis
        if len(res[0])>0:
            x,y = getXY(res[0])
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
def ControlRobot(cap, dictionary, Ji):
    global x_f
    global y_f
    #set Jacobian
    J = Ji

    #counter
    i = 1 #don't care about first point in trajectory 

    #store some arrays
    x_path = [] #path robot follows in x
    y_path = [] #path robot follows in y
    x_traj = [] #path we want the robot to follow in x
    y_traj = [] #path we want the robot to follow in y 

    #store changes
    delta_xy = np.matrix([[0],[0]])
    delta_q = np.matrix([[0],[0],[0]])

    #set initial q positions
    #hard coded from initial position in arduino code
    q1 = 900 
    q2 = 900
    q3 = 2300
    qqq = np.matrix([[q1],[q2],[q3]])

    x_f = [] #test position for code
    y_f = [] #test position for code
    
    #wait for the Arduino
    waitForArduino()

    while(cap.isOpened()):
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # Detect Marker
        res = cv2.aruco.detectMarkers(frame,dictionary)
        if x_f != []: 
            cv2.circle(frame,(int(x_f),int(y_f)),5, (0,0,255),-1)

        # Get desired position to go to will need some sort of pause maybe?
        # Or we could do this outside of the while loop? 

        ## Mouse click code        
        cv2.imshow('frame', frame)
        #add click line here
        cv2.setMouseCallback('frame',clicky)
        
        #if markers are detected track and display axis
        if (len(res[0])>0 and x_f != []):             

            ##### Position stuff #####
            #cv2.aruco.drawDetectedMarkers(frame,res[0],res[1])
            #get current position
            x_i, y_i = getXY(res[0])
            cv2.circle(frame,(int(x_i),int(y_i)),3, (0,255,0),-1)
             
            #append position to matrix to save data
            x_path.append(x_i)
            y_path.append(y_i)

            #get position in trajectory
            x_d, y_d = pixelpixel(x_i, y_i, x_f, y_f)
            x_traj.append(x_d)
            y_traj.append(y_d)

            #get error between current position and where we want to go
            e_x = x_d - x_i
            e_y = y_d - y_i
            delta_xy = np.hstack((delta_xy,np.matrix([[e_x],[e_y]])))

            ##### calculate new Jacobian ####
            #wait 10 steps then start calculating new Jacobian each step
            if len(delta_xy) > 10:                  
                    #B = 2xN array that is [x;y] and mX 3xN array that is [q1;q2;q3]
                    B = delta_xy[:,length-10:length]
                    mX = delta_q[:,length-10:length]
                    imX = np.linalg.pinv(mX)
                    J = np.dot(B,imX) #new Jacobian!                 


            ##### Q stuff #####
            #need to make sure that we have a matrix with non zero rows in order for .inv to work 
            #solve for delta q1, q2, q3 needed
            iJ = np.linalg.pinv(J)
            q_d = np.dot(iJ, np.matrix([[e_x],[e_y]]))
            q1_new = int(round(q1 + np.asscalar(q_d[0])))
            q2_new = int(round(q2 + np.asscalar(q_d[1])))
            q3_new = int(round(q3 + np.asscalar(q_d[2])))
            #print(q1_new,q2_new,q3_new)

            #do some error checking to make sure 900 < q < 2300
            #if not q stays at the last value
            if q1_new > 900 and q1_new < 2300:
                    q1 = q1_new
            else:
                    q_d[0] = 0
            if q2_new > 900 and q2_new < 2300:
                    q2 = q2_new
            else:
                    q_d[1] = 0
            if q3_new > 900 and q3_new < 2300:
                    q3 = q3_new
            else:
                    q_d[2] = 0

            delta_q = np.hstack((delta_q,q_d))                
            qqq = np.hstack((qqq,np.matrix([[q1],[q2],[q3]])))

            #send data to arduino
            sendStr = "*"+str(q1)+","+str(q2)+","+str(q3)+"*"
            sendToArduino(sendStr)

            #print servo position from board to double check values
            #print(getServoPos())


        # Display the resulting frame
        cv2.imshow('frame', frame)
        
        #end program if q is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    #Try and save data to file
    try: 
        #need to convert numpy data to matrix
        data = {"x_traj": x_traj, "y_traj": y_traj,
                "x_path": x_path, "y_path": y_path,
                "delta_xy": delta_xy.tolist(), 
                "qqq": qqq.tolist(), "delta_q": delta_q.tolist()} 
        with open("control_path_data.json", "w") as outfile: 
                json.dump(data,outfile)
        print("Control Data Saved to File")
        print(x_f, y_f)
    #except happens if error occurs in saving data to JSON and lets you know the file didn't save
    except Exception as e: 
        print(e)
        #do nothing
        print "Did not save control data to file"

    return()

### Click Function
def clicky(click, i,j, flags, param):
    global x_f
    global y_f
    if click == cv2.EVENT_LBUTTONDOWN:
        x_f = i
        y_f = j

### pixel by pixel trajectory, moves controller in direction of x_f, y_f 2 pixels at a time
# Inputs:
#   x_i = current x pixel
#   y_i = current y pixel
#   x_f = desired x pixel
#   y_f = desired y pixel
# Outputs:
#   x_d = desired x position at next step
#   y_d = desired y position at next step
def pixelpixel(x_i, y_i, x_f, y_f): 
    if x_i > x_f:
        x_d = x_i - 2
    elif x_i < x_f:
        x_d = x_i + 2
    else:
        x_d = x_i
    if y_i > y_f:
        y_d = y_i - 2
    elif y_i < y_f:
        y_d = y_i + 2
    else:
        y_d = y_i
        
    return(x_d, y_d) 
    


########### ARDUINO COMMUNICATION FUNCTIONS #########

###
def sendToArduino(sendStr):
    ser.write(sendStr) #data should be of the form <poser1,poser2,poser3>
    return()

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
    q1, q2, q3 = msg.split(',')
    q1 = int(q1)
    q2 = int(q2)
    q3 = int(q3)
    return(q1, q2, q3)

########### RUN PROGRAM ##########
main(decimator, allCorners, allIds, cap, dictionary, board)
