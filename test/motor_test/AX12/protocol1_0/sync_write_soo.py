import os

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

#################### my code #########################
def dynamixel_torque(packetHandler, portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE) :
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL_ID)
        
def allocate_goal_position(dxl_goal_position, DXL_ID, groupSyncWrite, index) :
    param_goal_position2 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]))]
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID, param_goal_position2)
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
#################### my code #########################



def move_soo( D1_ID, D2_ID, D3_ID, D4_ID, DEVICENAME_tmp, D1_min, D1_max, D2_min, D2_max, D3_min, D3_max, D4_min, D4_max) :

    #def move_soo( DXL_ID_tmp , DEVICENAME_tmp, DXL_MINIMUM_POSITION_VALUE_tmp, DXL_MAXIMUM_POSITION_VALUE_tmp):
    # Control table address
    ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
    ADDR_MX_GOAL_POSITION      = 30
    ADDR_MX_PRESENT_POSITION   = 36

    # Data Byte Length
    LEN_MX_GOAL_POSITION       = 4
    LEN_MX_PRESENT_POSITION    = 4

    # Protocol version
    PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

    # Default setting
    DXL1_ID                     = D1_ID                 # Dynamixel#1 ID : 1
    DXL2_ID                     = D2_ID                # Dynamixel#1 ID : 2
    DXL3_ID                     = D3_ID                 # Dynamixel#1 ID : 1
    DXL4_ID                     = D4_ID                # Dynamixel#1 ID : 2
    
    BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
    DEVICENAME                  = DEVICENAME_tmp           # Check which port is being used on your controller
                                                    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque
    DXL_MINIMUM_POSITION_VALUE  = D1_min           # Dynamixel will rotate between this value
    DXL_MAXIMUM_POSITION_VALUE = D1_max            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
    DXL_MINIMUM_POSITION_VALUE2  = D2_min           # Dynamixel will rotate between this value
    DXL_MAXIMUM_POSITION_VALUE2  = D2_max
    DXL_MINIMUM_POSITION_VALUE3  = D3_min           # Dynamixel will rotate between this value
    DXL_MAXIMUM_POSITION_VALUE3  = D3_max            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
    DXL_MINIMUM_POSITION_VALUE4  = D4_min           # Dynamixel will rotate between this value
    DXL_MAXIMUM_POSITION_VALUE4  = D4_max
    
    DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold

    index = 0
    dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         
    dxl_goal_position2 = [DXL_MINIMUM_POSITION_VALUE2, DXL_MAXIMUM_POSITION_VALUE2] 
    dxl_goal_position3 = [DXL_MINIMUM_POSITION_VALUE3, DXL_MAXIMUM_POSITION_VALUE3]         
    dxl_goal_position4 = [DXL_MINIMUM_POSITION_VALUE4, DXL_MAXIMUM_POSITION_VALUE4] 
    
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # Enable Dynamixel#1 Torque
    dynamixel_torque(packetHandler = packetHandler, portHandler = portHandler, DXL_ID = DXL1_ID, ADDR_MX_TORQUE_ENABLE = ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE = TORQUE_ENABLE)
    # Enable Dynamixel#2 Torque
    dynamixel_torque(packetHandler = packetHandler, portHandler = portHandler, DXL_ID = DXL2_ID, ADDR_MX_TORQUE_ENABLE = ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE = TORQUE_ENABLE)
    # Enable Dynamixel#3 Torque
    dynamixel_torque(packetHandler = packetHandler, portHandler = portHandler, DXL_ID = DXL3_ID, ADDR_MX_TORQUE_ENABLE = ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE = TORQUE_ENABLE)
    # Enable Dynamixel#4 Torque
    dynamixel_torque(packetHandler = packetHandler, portHandler = portHandler, DXL_ID = DXL4_ID, ADDR_MX_TORQUE_ENABLE = ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE = TORQUE_ENABLE)
    while 1:
        ######## wait code  ########
        '''
        print("Press any key to continue! (or press ESC to quit!)")
        if getch() == chr(0x1b):
            break
        '''
        ######## wait code  ########
        
        allocate_goal_position(dxl_goal_position = dxl_goal_position, DXL_ID = DXL1_ID, groupSyncWrite = groupSyncWrite, index = index)
        allocate_goal_position(dxl_goal_position = dxl_goal_position2, DXL_ID = DXL2_ID, groupSyncWrite = groupSyncWrite, index = index)
        allocate_goal_position(dxl_goal_position = dxl_goal_position3, DXL_ID = DXL3_ID, groupSyncWrite = groupSyncWrite, index = index)
        allocate_goal_position(dxl_goal_position = dxl_goal_position4, DXL_ID = DXL4_ID, groupSyncWrite = groupSyncWrite, index = index)
        
        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

        while 1:
            dxl1_present_position = read_dynamixel(packetHandler = packetHandler, portHandler = portHandler, DXL_ID = DXL1_ID, ADDR_MX_PRESENT_POSITION = ADDR_MX_PRESENT_POSITION)
            dxl2_present_position = read_dynamixel(packetHandler = packetHandler, portHandler = portHandler, DXL_ID = DXL2_ID, ADDR_MX_PRESENT_POSITION = ADDR_MX_PRESENT_POSITION)
            dxl3_present_position = read_dynamixel(packetHandler = packetHandler, portHandler = portHandler, DXL_ID = DXL3_ID, ADDR_MX_PRESENT_POSITION = ADDR_MX_PRESENT_POSITION)
            dxl4_present_position = read_dynamixel(packetHandler = packetHandler, portHandler = portHandler, DXL_ID = DXL4_ID, ADDR_MX_PRESENT_POSITION = ADDR_MX_PRESENT_POSITION)
            print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, dxl_goal_position[index], dxl1_present_position, DXL2_ID, dxl_goal_position2[index], dxl2_present_position))
            print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL3_ID, dxl_goal_position3[index], dxl3_present_position, DXL4_ID, dxl_goal_position4[index], dxl4_present_position))
            if not ((abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl_goal_position2[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD) \
                and (abs(dxl_goal_position3[index] - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl_goal_position4[index] - dxl4_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
                break

        # Change goal position
        if index == 0:
            index = 1
        else:
            index = 0

    # Disable Dynamixel#1 Torque
    disable_torque(packetHandler = packetHandler, portHandler = portHandler, DXL_ID = DXL1_ID, ADDR_MX_TORQUE_ENABLE = ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE = TORQUE_DISABLE)
    disable_torque(packetHandler = packetHandler, portHandler = portHandler, DXL_ID = DXL2_ID, ADDR_MX_TORQUE_ENABLE = ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE = TORQUE_DISABLE)
    disable_torque(packetHandler = packetHandler, portHandler = portHandler, DXL_ID = DXL3_ID, ADDR_MX_TORQUE_ENABLE = ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE = TORQUE_DISABLE)
    disable_torque(packetHandler = packetHandler, portHandler = portHandler, DXL_ID = DXL4_ID, ADDR_MX_TORQUE_ENABLE = ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE = TORQUE_DISABLE)
    
    # Close port
    portHandler.closePort()




move_soo(D1_ID = 13, D2_ID = 14, D3_ID = 15, D4_ID = 17, DEVICENAME_tmp = '/dev/ttyUSB0',\
         D1_min = 550, D1_max = 650, D2_min = 550, D2_max = 650, D3_min = 550, D3_max = 650, D4_min = 550, D4_max = 650)