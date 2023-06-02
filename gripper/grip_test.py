import gripper_controller as gripper
from time import sleep

ser = gripper.openSerial(8)
sleep(20)
gripper.grip(ser, "1")

while True:
    print("opening")
    stat=gripper.status(ser)
    print(stat)
    print(gripper.eval_status(stat))
    print(gripper.ready_to_grip(ser))
    sleep(0.4)
    gripper.grip(ser, "1")
    sleep(1)
    print("closing")    
    stat=gripper.status(ser)
    print(stat)
    print(gripper.eval_status(stat))
    print(gripper.ready_to_grip(ser))
    sleep(0.4)
    gripper.grip(ser, "0")
    sleep(1)
gripper.grip(ser, "1")

sleep(1)

gripper.grip(ser, "0")

gripper.closeSerial(ser)