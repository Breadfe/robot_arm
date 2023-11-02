# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_MOVING_SPEED          = 32
ADDR_MX_TORQUE_LIMIT          = 34
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_PRESENT_SPEED      = 38
ADDR_MX_MOVING             = 46

# Data Byte Length
LEN_MX_GOAL_POSITION       = 4
LEN_MX_PRESENT_POSITION    = 4
LEN_MX_PRESENT_SPEED       = 4
LEN_MX_TORQUE_LIMIT        = 4
LEN_MX_MOVING              = 1

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL10                       = 10                 
DXL21                       = 21
DXL22                       = 22
DXL23                       = 23
DXL24                       = 24
DXL25                       = 25


BAUDRATE                    = 1000000            # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM6'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 40             # Dynamixel moving status threshold

MOTOR_SPEED = 300

motors = [DXL10, DXL21, DXL22, DXL23, DXL24, DXL25]


speed = 70 # defalut speed
MAX_MOTOR_STEP = 100
MIN_MOTOR_STEP = 30 # 30 recommended
step = step = int(-(MAX_MOTOR_STEP-MIN_MOTOR_STEP)/100*speed + MAX_MOTOR_STEP)  # defalut step

def deg_to_decimal(deg):
    # 0 4095 <- 0 360
    a:float = 4095/359.91
    decimal:int = int(deg * a)
    # print(decimal)
    return decimal

def decimal_to_deg(decimal):
    a:float = 359.91/4095
    deg = round(float(a*decimal), 2)
    # print(deg)
    return deg

