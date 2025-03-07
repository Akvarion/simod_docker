import socket
import time

from enum import Enum
from robotiq_control.GripperCommon import RobotiqGripperType,Robotiq,RobotiqSocketCmds
''' da usare se non si usa ros'''
# from utils import enforce_cast
# from GripperCommon import RobotiqGripperType,Robotiq,RobotiqSocketCmds


class ActivationStatus(Enum):
    NOT_CONNECTED = -1
    RESET = 0
    ACTIVATION_BUSY = 1
    NOT_USED = 2
    ACTIVATION_DONE = 3

def enforce_cast(func):
    def wrapper(*args, **kwargs):
        # Perform casting for input parameters if necessary
        new_args = [int(arg) if isinstance(arg, (str, float)) else arg for arg in args]


        # Call the original function with the casted parameters
        result = func(*new_args, **kwargs)

        # Perform casting for the output value if necessary
        if not isinstance(result, str):
            result = str(result)

        return result

    return wrapper
        
class GripperSocket(Robotiq):
    
    def __init__(self, robot_ip="192.168.0.102", port=63352, gripper_type=RobotiqGripperType.Hand_E):
        Robotiq.__init__(self, gripper_type)
        try:
            self.skt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.skt.settimeout(0.1)
            print ("Socket successfully created")
        except socket.error as err:
            print ("socket creation failed with error %s" %(err))
        
        self.is_connected = False
        self.skt_port = port
        self.skt_ip = robot_ip
        self.gripper_type = gripper_type
        
    
    def connect(self):
        try:
            self.skt.connect((self.skt_ip, self.skt_port))
            print(f"Connection Estabilished to {self.skt_ip}:{self.skt_port}")
            self.is_connected = True
            time.sleep(0.1)
            return True
        except:
            print(f"Failed To Connect to {self.skt_ip}:{self.skt_port}")
            return False
        
    def connectionClose(self):
        try:
            self.skt.close()
            print("Connection Closed")
            self.is_connected = False
            return True
        except:
            print("Failed To Close Connection")
            return False
        
    def __sendCommand(self, cmd):
        try:
            self.skt.settimeout(0.1)
            self.skt.send(cmd)
            # self.skt.sendall(cmd)
            data = self.skt.recv(2**10)
            
            # print("Command {} Sent With Success".format(cmd.decode("utf-8").replace("\n", "")))
            if 'ack' in data.decode('utf-8'):
                return True, -1
            else:
                return True, int(data.split()[1])
        except socket.timeout as to:
            self.connectionClose()
            print("Timeout Expired : %s" % to)
            return False, -1
    
    
    @enforce_cast
    def __setPos_perc(self, perc:int) -> str:
        return 255 - int(255*perc/100)
    
    def __checkGripperStatus(self):
        result = -1
        success = False
        if self.is_connected:
            success, result = self.__sendCommand(RobotiqSocketCmds.cmd_get_activation_status)
        
        # print(ActivationStatus(int(result)))
        return success, ActivationStatus(int(result))
    
    def setGripperAperture(self, value):
        super().setStroke(value)
        
    def initialize(self):
        if not self.is_connected:
            self.connect()
        
        if self.isSleeping(): #gripper is in reset
            print("Sending Activation Cmd")
            cmd_sent = self.__activate()
            time.sleep(3)   
        
        if self.isReady():
            return True
        
        return False
    
    
    def __activate(self):
        print("ACTIVATE")
        return self.__sendCommand(RobotiqSocketCmds.cmd_activate)
    
    def deactivate(self):
        return self.__sendCommand(RobotiqSocketCmds.cmd_deactivate)
    
    def isReady(self):
        _, activation_status =  self.__checkGripperStatus()
        return activation_status == ActivationStatus.ACTIVATION_DONE
    
    def isSleeping(self):
        _, activation_status =  self.__checkGripperStatus()
        return activation_status == ActivationStatus.RESET
    
    def open(self):
        print("OPEN")
        return self.__sendCommand(RobotiqSocketCmds.cmd_full_open)

    def close(self):
        print("CLOSE")  
        return self.__sendCommand(RobotiqSocketCmds.cmd_full_close)

    def __sendMoveRoutine(self, pos, speed, force):
        print("SEND MOVE ROUTINE ")
        success_pos, _   = self.__sendCommand(pos)
        success_vel, _   = self.__sendCommand(RobotiqSocketCmds.cmd_set_speed + str(speed).encode() + b'\n')
        success_force, _ = self.__sendCommand(RobotiqSocketCmds.cmd_set_force + str(force).encode() + b'\n')

        success_GTO = False
        if success_pos and success_vel and  success_force:
            # sent, echo = self.getTargetPos()
            # if echo in pos.decode("utf-8"): #check se il comando ricevuto dal gripper coincide con quello inviato dall'utente
                success_GTO, _ = self.__sendCommand(RobotiqSocketCmds.cmd_EnableMove)
        print(success_pos, success_vel, success_force, success_GTO)
        return success_GTO

    
    def moveToPos(self, pos, speed, force):
        print("MOVE TO POSE")
        if pos > self.max_stroke:
            pos = self.max_stroke
        elif pos < 0:
            pos = 0
        #devono essere int
        speed = int(speed)
        force = int(force) 
        pos = RobotiqSocketCmds.cmd_set_pos + str(super().getPositionRequest(pos)).encode() + b"\n"
        return self.__sendMoveRoutine(pos, speed, force)
        
    
    @enforce_cast
    def moveToPer(self, pos_perc:int, speed:int, force:int):
        print("MOVE TO PERC")
        if pos_perc > 100:
            pos_perc = 100
        elif pos_perc < 0:
            pos_perc = 0
        # if self.__checkGripperStatus() == ActivationStatus.ACTIVATION_DONE:
        target_pos = RobotiqSocketCmds.cmd_set_pos + self.__setPos_perc(pos_perc).encode() + b'\n'
        self.__sendMoveRoutine(target_pos, speed, force)
    
    def graspDetected(self):
        _ , fdbk = self.__sendCommand(RobotiqSocketCmds.cmd_object_detected)
        return fdbk == 1 or fdbk == 2

    def getFaultId(self):
        _, fdbk = self.__sendCommand(RobotiqSocketCmds.cmd_get_fault)
        return fdbk
         
    def getVelocityEcho(self):
        success, fdbk = self.__sendCommand(RobotiqSocketCmds.cmd_get_speed)
        return fdbk
    
    def getForceEcho(self):
        success, fdbk = self.__sendCommand(RobotiqSocketCmds.cmd_get_force)
        return fdbk

    def isInPosition(self):
        return self.getTargetPos() == self.getActualPos()
    
    def getRequestedPosition(self):
        success, fdbk = self.__sendCommand(RobotiqSocketCmds.cmd_get_echo)
        # print(fdbk)
        return super().byteToPosition(fdbk)
    
    def getActualPos(self):
        success, fdbk = self.__sendCommand(RobotiqSocketCmds.cmd_get_pos)
        # print(fdbk)
        return super().byteToPosition(fdbk)

    def getCurrent(self):
        _, fdbk = self.__sendCommand(RobotiqSocketCmds.cmd_get_current)
        return fdbk


# gripper = GripperSocket()
# gripper.setGripperAperture(0.038)
# gripper.setGripperAperture(0.001)
# gripper.initialize()
# gripper.moveToPer(20, 10, 10)
# time.sleep(2)
# gripper.moveToPos(0.9, 10, 10.1)

# gripper.connectionClose()



'''
ACT: Activation bit
0 - Gripper not activated
1 - Gripper activated
GTO: 1 if the gripper is set to move to requested position 0 if gripper is set to stay at the same place
PRE: Position request eco. Should be same a the requested position if
the gripper successfully received the requested position.
POS: Current position of the gripper
SPE: Speed eco. Should be same as requested speed.
FOR: Force parameter of the gripper
OBJ: Object grippings status
0 - Fingers are inmotion towards requested position.No object detected.
1 - Fingers have stopped due to a contact while opening before requested position.Object detected opening.
2 - Fingers have stopped due to a contact while closing before requested position.Object detected closing.
3 - Fingers are at requested position.No object detected or object has been loss / dropped.
STA: Gripper status, returns the current status & motion of theGripper fingers.
0 -Gripper is in reset ( or automatic release )state. See Fault Status if Gripper is activated.
1 - Activation in progress.
2 - Not used.
3 - Activation is completed.
MOD: ...
FLT: Fault status returns general errormessages that are useful for troubleshooting. Fault LED (red) is present on theGripper chassis,
LED can be blue, red or both and be solid or blinking.
0 - No fault (LED is blue)
Priority faults (LED is blue)
5 - Action delayed, activation (reactivation)must be completed prior to performing the action.
7 - The activation bit must be set prior to action.
Minor faults (LED continuous red)
8 -Maximum operating temperature exceeded,wait for cool-down.
9 No communication during at least 1 second.
Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit rACT needed).
10 - Underminimum operating voltage.
11- Automatic release in progress.
12- Internal fault; contact support@robotiq.com.
13 - Activation fault, verify that no interference or other error occurred.
14-Overcurrent triggered.
15- Automatic release completed.
MSC: Gripper maximym current.
COU: Gripper current.
NCY: Number of cycles performed by the gripper
DST: Gripper driver state
0 - Gripper Driver State : RQ_STATE_INIT
1 - Gripper Driver State : RQ_STATE_LISTEN
2 - Gripper Driver State : Q_STATE_READ_INFO
3 - Gripper Driver State : RQ_STATE_ACTIVATION
Other - Gripper Driver State : RQ_STATE_RUN
PCO: Gripper connection state
0 - Gripper Connection State : No connection problem detected
Other - Gripper Connection State : Connection problem detected



It is possible to select which gripper to send the command to by using the command sid followed by the gripper id.
ex

sid2 (will send commands to gripper with ID 2

Note that ID1 in polyscope represent ID9 in socket communication(default)

Here's the full list 
drivergripper SET/GET commands SET commands : 

 ACT activateRequest 
 MOD gripperMode 
 GTO goto 
 ATR automaticReleaseRoutine 
 ARD autoreleaseDirection 
 MSC maxPeakSupplyCurrent 
 POS positionRequest 
 SPE speedRequest 
 FOR forceRequest 
 SCN_BLOCK scanBlockRequest 
 SCN scanRequest 
 NID updateGripperSlaveId 
 SID socketSlaveId 

 GET commands : 
 ACT activateRequest 
 MOD gripperMode 
 GTO goto 
 STA status 
 VST vacuumStatus 
 OBJ objectDetected 
 FLT fault 
 MSC maxPeakSupplyCurrent 
 PRE positionRequestEcho 
 POS positionRequest 
 COU motorCurrent 
 SNU serialNumber 
 PYE productionYear 
 NCY numberOfCycles 
 PON numberOfSecondsPumpIsOn 
 NPA numberOfPumpActivations 
 FWV firmwareVersion 
 VER driverVersion 
 SPE speedRequest
 FOR forceRequest 
 DRI printableState
 SID socketSlaveId
 
'''