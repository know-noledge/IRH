import gpiod
import spidev
import asyncio
import time
import datetime
import os
import numpy as np
from dynamixel_sdk import *


# -- UART Dynamixel servo config --
# Control table address (for AX-12A)
ADDR_TORQUE_ENABLE      = 24
ADDR_GOAL_POSITION      = 30
ADDR_SPEED              = 32
ADDR_CURRENT_POSITION   = 36

PROTOCOL_VERSION = 1.0

# Default setting
DXL1_ID = 1                 # Dynamixel motor 1
DXL2_ID = 2                 # Dynamixel motor 2
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyTHS0'  # Pin 8 (uart-TX)

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# Min, Max = 0, 1023
DXL1_START_POSITION_VALUE = 200 # Open values
DXL2_START_POSITION_VALUE = 325
DXL1_END_POSITION_VALUE = 825   # Closed values
DXL2_END_POSITION_VALUE = 950

DXL_SPEED = 200

# -- Open tx port with protocol version
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

# -- Enable torque
packetHandler.write1ByteTxOnly(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
packetHandler.write1ByteTxOnly(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
# --- Set speed ---
packetHandler.write2ByteTxOnly(portHandler, DXL1_ID, ADDR_SPEED, DXL_SPEED)
packetHandler.write2ByteTxOnly(portHandler, DXL2_ID, ADDR_SPEED, DXL_SPEED)
# -- Set start position
packetHandler.write2ByteTxOnly(portHandler, DXL1_ID, ADDR_GOAL_POSITION, DXL1_START_POSITION_VALUE) #200 startpoint
packetHandler.write2ByteTxOnly(portHandler, DXL2_ID, ADDR_GOAL_POSITION, DXL2_START_POSITION_VALUE) #325 startpoint

# --PWM configs-- 
PWM_Channel1 = "0"
PWM_Channel2 = "2"
SERVO_PERIOD_NS = 20000000     # 20ms
SERVO_LEFT_DUTY_NS = 500000    # 0.5ms
SERVO_CENTER_DUTY_NS = 1500000 # 1.5ms
SERVO_RIGHT_DUTY_NS = 2500000  # 2.5ms

# --GPIO configs--
GPIO_CHIP = "/dev/gpiochip0"
MOTION_LINE_OFFSET = 106
SPI_BUS = 0
SPI_DEVICE = 0
POLL_INTERVAL_SEC = 0.1

# -- Setup PWM --
def write(path, value):
    with open(path, 'w') as f:
        f.write(str(value))

pwm_base = f"/sys/class/pwm/pwmchip{PWM_Channel1}"
pwm_base2 = f"/sys/class/pwm/pwmchip{PWM_Channel2}"

if not os.path.exists(f"{pwm_base}/pwm0"):
    write(f"{pwm_base}/export", "0")
    time.sleep(0.1)
write(f"{pwm_base}/pwm0/period", SERVO_PERIOD_NS)
#write(f"{pwm_base}/pwm0/duty_cycle", SERVO_CENTER_DUTY_NS)
write(f"{pwm_base}/pwm0/enable", 1)

if not os.path.exists(f"{pwm_base2}/pwm0"):
    write(f"{pwm_base2}/export", "0")
    time.sleep(0.1)
write(f"{pwm_base2}/pwm0/period", SERVO_PERIOD_NS)
#write(f"{pwm_base2}/pwm0/duty_cycle", SERVO_CENTER_DUTY_NS)
write(f"{pwm_base2}/pwm0/enable", 1)

# -- Setup SPI --
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = 2000000
spi.mode = 3

# -- Setup GPIO --
chip = gpiod.chip(GPIO_CHIP)
motion_line = chip.get_line(MOTION_LINE_OFFSET)
config = gpiod.line_request()
config.consumer = "motion_detect"
config.request_type = gpiod.line_request.EVENT_FALLING_EDGE
motion_line.request(config)

# -- Shared variables --
motion_detected = False
counter = 0
current_angle = 90 # OBS!!! 90 deg = CLOSED

# -- Servo functions --
def set_servo(angle_deg):
    angle_deg = max(0, min(180, angle_deg))
    duty_ns = int(SERVO_LEFT_DUTY_NS + (angle_deg / 180) * (SERVO_RIGHT_DUTY_NS - SERVO_LEFT_DUTY_NS))
    write(f"{pwm_base}/pwm0/duty_cycle", duty_ns)
    write(f"{pwm_base2}/pwm0/duty_cycle", duty_ns)
    print(angle_deg)
    return angle_deg

def close_hand():	# we need to find this_angle then add angles to squeeze the object. (don't use LEFT_DUTY_NS)
    #write(f"{pwm_base}/pwm0/duty_cycle", SERVO_LEFT_DUTY_NS) # RIGHT == MAX CLOSED but find the closed value
    #write(f"{pwm_base2}/pwm0/duty_cycle", SERVO_LEFT_DUTY_NS)
    set_servo(90)
    packetHandler.write2ByteTxOnly(portHandler, DXL1_ID, ADDR_GOAL_POSITION, DXL1_END_POSITION_VALUE)
    packetHandler.write2ByteTxOnly(portHandler, DXL2_ID, ADDR_GOAL_POSITION, DXL2_END_POSITION_VALUE)
    
    return False

def open_hand():	# always opens maximum when opening hand
    write(f"{pwm_base}/pwm0/duty_cycle", SERVO_LEFT_DUTY_NS)
    write(f"{pwm_base2}/pwm0/duty_cycle", SERVO_LEFT_DUTY_NS)
    packetHandler.write2ByteTxOnly(portHandler, DXL1_ID, ADDR_GOAL_POSITION, DXL1_START_POSITION_VALUE) #200 startpoint
    packetHandler.write2ByteTxOnly(portHandler, DXL2_ID, ADDR_GOAL_POSITION, DXL2_START_POSITION_VALUE) #325 startpoint
    
    return True

# slip motion function for the pwm motors
async def slow_servo(start_angle, end_angle):
    step_size = 0.25
    step_delay = 0.15
    step = step_size
    
    for angle in np.arange(start_angle, end_angle + step, step):
        set_servo(angle)
        await asyncio.sleep(step_delay)
    return set_servo(end_angle)

# -- Motion read --
def read_motion():
    response = spi.xfer2([0x02, 0x00])  # Read motion register
    time.sleep(0.000035)  # 35us
    motion = response[1]
    return motion

async def sensor_task():
    global motion_detected
    global counter
    while True:
        data = read_motion()
        if data & 0x80:  # Check motion bit
            motion_detected = True
            counter += 1
            print(f"Motion detected!, counter: {counter}")
        await asyncio.sleep(POLL_INTERVAL_SEC)

async def motor_task():
    global motion_detected
    global counter
    global current_angle
    counter = 0
    isOpen = True
    
    while True:
        
        if motion_detected:
            motion_detected = False
            #counter += 1
            print(f"Motion registered! Closing hand at {counter} movements")
            if counter > 2 and isOpen:
                isOpen = close_hand()
                counter = 0
                await asyncio.sleep(0.5)	# This sleep limits the reaction time of the servo relative to the sensor 
                current_angle = 90		
                
        else:
            #if not isOpen:
                #counter = 0
            if current_angle > 10:
                next_angle = current_angle - 5
            else:
                next_angle = current_angle
            await slow_servo(current_angle, next_angle)
            isOpen = True
            current_angle = next_angle

        await asyncio.sleep(0.01)

# -- Main runner --
async def main():
    open_hand()
    time.sleep(5)
    close_hand()
    time.sleep(5)
    print("Starting async tasks...")
    await asyncio.gather(
        sensor_task(),
        motor_task()
    )

# -- Start program --
try:
    asyncio.run(main())

except KeyboardInterrupt:
    print("Stopping...")
    
finally:
    write(f"{pwm_base}/pwm0/enable", 0)
    write(f"{pwm_base2}/pwm0/enable", 0)
    spi.close()
    motion_line.release()
    packetHandler.write1ByteTxOnly(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxOnly(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()
