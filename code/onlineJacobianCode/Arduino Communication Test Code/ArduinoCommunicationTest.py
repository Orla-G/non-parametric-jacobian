import serial
import time

print
print

# NOTE the user must ensure that the serial port and baudrate are correct
serPort = "COM5" #"/dev/ttyS80"
baudRate = 9600
ser = serial.Serial(serPort, baudRate)
print "Serial port " + serPort + " opened  Baudrate " + str(baudRate)


startMarker = 60
endMarker = 62


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
    while ser.inWaiting() == 0:
        pass
    
    dataRecvd = recvFromArduino()
    print dataRecvd

    return(dataRecvd)


#### Program
waitForArduino()
sendToArduino("a")
while True:
    print recvFromArduino()
    
