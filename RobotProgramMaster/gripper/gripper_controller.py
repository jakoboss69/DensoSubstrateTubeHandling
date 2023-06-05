import time
import serial

# create a function that sends binary signal to an arduino to open/close the gripper serial


def openSerial(port = 8):
    # Open a serial connection to the Arduino on COM7
    ser = serial.Serial('COM'+str(port), timeout=0)
    # Set the baud rate to 9600
    ser.baudrate = 115200
    # Wait for the serial connection to be established

    time.sleep(2)
    return ser

def status(ser):
    ser.write(str.encode("2")) # string "2" incicates that the arduino should send the status of the hall sensors
    
    print("waiting for status")
    while(True):
        serial_message=ser.readline().decode() #read the serial message
        if serial_message == "": #if the serial message is empty, wait a bit and try again
            time.sleep(0.01)
        else:
            return(serial_message)
            


def eval_status(serial_message):
    #evaluate a csv string into 4 integers and evaluate them according to a goven threshold
    finger_closed_threshold = [760,760,760,760]
    
    
    message_split = serial_message.split(",")
    for i in range(len(message_split)): #evaluates the string into ints and checks the threshold
        message_split[i] = int(message_split[i])
        if message_split[i] > finger_closed_threshold[i]:
            message_split[i] = 1
        else:
            message_split[i] = 0
    return(message_split) #returns a list of 4 integers, 1 if the finger is closed, 0 if it is open
    

def ready_to_grip(ser):
    stat=status(ser)
    stat_eval=eval_status(stat)
    if stat_eval==[1,1,1,1]:
        return True
    else:
        return False
    


def grip(ser, binary_input):
    # Send the binary input as a byte 
    # time.sleep(1)
    ser.write(str.encode(binary_input))
    # print(ser.readline().decode())
    # time.sleep(0.06)
    # ser.write(str.encode(binary_input))
    

def closeSerial(ser):
    # Close the serial connection
    ser.close()





