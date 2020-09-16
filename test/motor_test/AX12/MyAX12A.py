import os

MOTOR_MAX_ANGLE = 750
MOTOR_MIN_ANGLE = 550

MIDDLE_MOTOR_ID = 13
INDEX_MOTOR_ID  = 14
THUMB_MOTOR_ID  = 15
PALM_MOTOR_ID   = 17

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import time

def dynamixel_torque(packetHandler, portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE) :
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL_ID)

def allocate_goal_position(dxl_goal_position, DXL_ID, groupSyncWrite) :
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID, param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID)
        quit()

def read_dynamixel(packetHandler, portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION) :
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    return dxl_present_position

def disable_torque(packetHandler, portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE) :
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

class AX12A_Handler:
    def __init__(self, DXL_IDs , DEVICENAME):
        # Control table address
        self.ADDR_MX_TORQUE_ENABLE      = 24               # Control C
        self.ADDR_MX_GOAL_POSITION      = 30
        self.ADDR_MX_PRESENT_POSITION   = 36

        # Data Byte Length
        self.LEN_MX_GOAL_POSITION       = 4
        self.LEN_MX_PRESENT_POSITION    = 4

        # Protocol version
        self.PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

        # Default setting
        self.DXL_IDs                      = DXL_IDs                 # Dynamixel ID : 1
        self.BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
        self.DEVICENAME                  = DEVICENAME    # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold


        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_MX_GOAL_POSITION, self.LEN_MX_GOAL_POSITION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()


        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        for DXL_ID in self.DXL_IDs:
            print("Enableing Torque of {}".format(DXL_ID))
            # Enable Dynamixel#1 Torque
            #print(self.packetHandler)
            dynamixel_torque(packetHandler = self.packetHandler, portHandler = self.portHandler, DXL_ID = DXL_ID, ADDR_MX_TORQUE_ENABLE = self.ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE = self.TORQUE_ENABLE)

    # goalDict = {3: 155, 11: 0, ...}
    def ControlPos(self, goalDict):
        isGoal = dict()
        for DXL_ID, dxl_goal_position in goalDict.items():
            if dxl_goal_position > MOTOR_MAX_ANGLE:
                dxl_goal_position = MOTOR_MAX_ANGLE
                goalDict[DXL_ID] = dxl_goal_position
            elif dxl_goal_position < MOTOR_MIN_ANGLE:
                dxl_goal_position = MOTOR_MIN_ANGLE
                goalDict[DXL_ID] = dxl_goal_position
            allocate_goal_position(dxl_goal_position = dxl_goal_position, DXL_ID = DXL_ID, groupSyncWrite = self.groupSyncWrite)
            isGoal.update({DXL_ID : 0})
        
        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

        while True:
            for DXL_ID, dxl_goal_position in goalDict.items():
                if dxl_goal_position == -1:
                    continue
                dxl_present_position = read_dynamixel(packetHandler = self.packetHandler, portHandler = self.portHandler, DXL_ID = DXL_ID, ADDR_MX_PRESENT_POSITION = self.ADDR_MX_PRESENT_POSITION)
                #print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position, dxl_present_position))
                
                if not ((abs(dxl_goal_position - dxl_present_position) > self.DXL_MOVING_STATUS_THRESHOLD)):
                    isGoal[DXL_ID] = 1
            if 0 not in isGoal.values():
                break

    def Close(self):
        for DXL_ID in self.DXL_IDs:
            disable_torque(packetHandler = self.packetHandler, portHandler = self.portHandler, DXL_ID = DXL_ID, ADDR_MX_TORQUE_ENABLE = self.ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE = self.TORQUE_DISABLE)
        # Close port
        self.portHandler.closePort()

# Test code
if __name__ == "__main__":
    DXL_IDs = [13, 14, 15, 17]
    ax12 = AX12A_Handler(DXL_IDs=DXL_IDs, DEVICENAME= "/dev/ttyUSB0")
    angle = 550
    for i in range(100):
        goalDict = {13: angle, 14: angle, 15: angle, 17: angle}
        ax12.ControlPos(goalDict)
        angle += 10
        time.sleep(0.0001)
    ax12.Close()