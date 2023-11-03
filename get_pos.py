import os
from time import time

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
from config import *


portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

position_groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)
position_groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
position_groupBulkRead = GroupBulkRead(portHandler, packetHandler)


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

#------------------------------SETTING END------------------------------#

#Torque On
def torque_on():
    for id in motors:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    # print('\033[32m' + 'Torque On' + '\033[0m')

#Torque Off
def torque_off():
    for id in motors:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    # print('\033[31m' + 'Torque Off' + '\033[0m')


def decimal_to_bit(position):
    return [DXL_LOBYTE(DXL_LOWORD(position)), 
                        DXL_HIBYTE(DXL_LOWORD(position)), 
                        DXL_LOBYTE(DXL_HIWORD(position)), 
                        DXL_HIBYTE(DXL_HIWORD(position))]

def set_speed(speed_val): # SPEED -> 0~100
    global step
    global speed
    if speed > 100:
        speed = 100
    elif speed < 0:
        speed = 0
    # speed 1 -> step = 1000
    # speed 100 -> step = 30
    if speed == 0:
        step = 0
    else:
        step = int(-(MAX_MOTOR_STEP-MIN_MOTOR_STEP)/100*speed_val + MAX_MOTOR_STEP)
        speed = speed_val
        print(step)    

def speed_joint_move_to(target_point, speed_val):
    global speed
    temp = speed
    set_speed(speed_val)
    joint_move_to(target_point)
    set_speed(temp)
    pass
    
def get_joint_pos():
    positions = []
    for DX_id in motors:
        dxl_addparam_result = position_groupBulkRead.addParam(DX_id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkRead position addparam failed" % DXL10)
            quit()

    dxl_comm_result = position_groupBulkRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    dxl_getdata_result = position_groupBulkRead.isAvailable(DX_id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupBulkRead getdata failed" % DX_id)
        quit()

    for DX_id in motors:    
        # dxl_present_position = position_groupBulkRead.getData(DX_id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
        dxl_present_position,_,_ = packetHandler.read4ByteTxRx(portHandler,DX_id, ADDR_MX_PRESENT_POSITION)
        deg = decimal_to_deg(dxl_present_position)
        if deg > 360 or deg < 0:
            '\033[31m' + 'Torque Off' + '\033[0m'
            print('\033[31m' + f'deg break out : {deg}', end=" ")
            deg = deg % 360
            deg = round(deg, 2)
            print(f'updated deg = {deg}' + '\033[0m')
            
        positions.append(deg)
    position_groupBulkRead.clearParam()
    #print(positions)
    return positions

def joint_move_to(target_point):
    # global step
    print('step = ', step)
    if step == 0:
        print('set speed = 0')
        return
    step_diff = [0]*6
    now_pos = get_joint_pos()
    for i in range(6):
        step_diff[i] = (target_point[i] - now_pos[i])/step
    print(f'target point = {target_point}')
    print(f'now pos = {now_pos}')
    for i in range(step):
        #print(i)
        position1 = decimal_to_bit(deg_to_decimal(now_pos[0]+step_diff[0]*i))
        position2 = decimal_to_bit(deg_to_decimal(now_pos[1]+step_diff[1]*i))
        position3 = decimal_to_bit(deg_to_decimal(now_pos[2]+step_diff[2]*i))
        position4 = decimal_to_bit(deg_to_decimal(now_pos[3]+step_diff[3]*i))
        position5 = decimal_to_bit(deg_to_decimal(now_pos[4]+step_diff[4]*i))
        position6 = decimal_to_bit(deg_to_decimal(now_pos[5]+step_diff[5]*i))
        position = [position1, position2, position3, position4, position5, position6]
        for i, id in enumerate(motors):
            dxl_addparam_result = position_groupSyncWrite.addParam(id, position[i])
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkRead position addparam failed" % DXL10)
                quit()
        dxl_comm_result = position_groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        #time.sleep(0.001)
        position_groupSyncWrite.clearParam()
        time.sleep(0.001)
    for DX_id in motors:
        while True:
            dxl_ismoving,_,_ = packetHandler.read4ByteTxRx(portHandler, DX_id, ADDR_MX_MOVING)
            if dxl_ismoving == 0:
                #print(dxl_ismoving)
                break

#------------------------------Functions------------------------------#            
    

if __name__ == '__main__':
    
    while True:
        a = int(input())
        #print(a)
        if a == 1:
            torque_on()
            print(f'joint_move_to({get_joint_pos()})')
            torque_off()
        elif a == 2:
            print('end')
            break
