from machine import Pin, PWM, UART
import time
import sys
import select

uart = UART(1, 9600)                         # init with given baudrate
uart.init(9600, bits=8, parity=None, stop=1) # init with given parameters

# IR array enable pin
ir_enable = Pin(1, Pin.OUT)
ir_enable.value(1)  # Turn on the IR array

# Sensor pins (left → right)
sensor_pins = [
    Pin(2, Pin.IN),
    Pin(3, Pin.IN),
    Pin(4, Pin.IN),
    Pin(5, Pin.IN),
    Pin(6, Pin.IN),
    Pin(7, Pin.IN),
    Pin(8, Pin.IN),
    Pin(9, Pin.IN)
]

# Motor pins
AIN1 = PWM(Pin(18), freq=1000)  # Right motor forward
AIN2 = PWM(Pin(19), freq=1000)  # Right motor backward
BIN1 = PWM(Pin(21), freq=1000)  # Left motor forward
BIN2 = PWM(Pin(20), freq=1000)  # Left motor backward
SLEEP = Pin(22, Pin.OUT)        # Sleep pin

LED = Pin(25, Pin.OUT)

# Button setup
button = Pin(10, Pin.IN, Pin.PULL_UP)  # Use PULL_UP if button connects to GND

# General variables
baseSpeed = 40   # 0-255
debugMode = False
pos = -1
error = 0

#PID variables
Kp = 1
Kd = 10

def read_line(prev_pos):
    values = ['#' if pin.value() == 1 else '_' for pin in sensor_pins]
    c = 0
    s = 0
    for i in range(8):
        if values[i] == '#':
            c += 1
            s += i*10

    if c == 8:
        SLEEP.value(0)  # Stop motors
        return values, -1
    elif c == 0:
        return values, prev_pos
    else:
        return values, s/c
    
def calc_PID(pos, prev_error):
    error = pos - 35
    P = Kp * error
    I = 0
    D = error - prev_error
    
    PID = P + I + D
    
    prev_error = error
    return error, PID

def motor_drive(PIDvalue):
    # Calculate motor speeds
    right = constrain(baseSpeed + PIDvalue, -255, 255)
    left  = constrain(baseSpeed - PIDvalue, -255, 255)
    
    # Right motor
    if right > 0:
        AIN1.duty_u16(int(right * 257))  # Scale 0-255 → 0-65535
        AIN2.duty_u16(0)
    else:
        AIN1.duty_u16(0)
        AIN2.duty_u16(int(abs(right) * 257))
    
    # Left motor
    if left > 0:
        BIN1.duty_u16(int(left * 257))
        BIN2.duty_u16(0)
    else:
        BIN1.duty_u16(0)
        BIN2.duty_u16(int(abs(left) * 257))

def constrain(value, min_val, max_val):
    return max(min_val, min(max_val, value))

def read_button():
    if button.value() == 0:  # Button pressed (active LOW)
        while button.value() == 0:
            time.sleep(0.1)  # Debounce delay
        time.sleep(0.5)  # Additional delay to prevent multiple triggers
        SLEEP.value(1)  # Wake up motors

def read_command():
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        line = sys.stdin.readline().strip()
        
        if line:
            # Remove spaces (so "d 1" → "d1")
            line = line.replace(" ", "")
            
            # Split command and numeric value
            cmd = ""
            val = ""
            
            for c in line:
                if c.isalpha():
                    cmd += c
                elif c.isdigit() or c == '-':
                    val += c
            
            val = int(val) if val else None
            
            process_command(cmd, val)

def process_command(cmd, val):
    global Kp, Kd, baseSpeed, debugMode
    if cmd == 'p':
        #if val is not None else set Kp
        Kp = val if val is not None else Kp
        print(f"Kp is {Kp}")
    elif cmd == 'd':
        Kd = val if val is not None else Kd
        print(f"Kd is {Kd}")
    elif cmd == 's':
        baseSpeed = val if val is not None else baseSpeed
        print(f"Base speed is {baseSpeed}")
    elif cmd == 'g':
        debugMode = not debugMode
        print(f"Debug mode {'enabled' if debugMode else 'disabled'}")
    elif cmd == 'h':
        print("p <value> - Set Kp\n")
        print("d <value> - Set Kd\n")
        print("s <value> - Set base speed\n")
        print("g - Toggle debug mode\n")
        print("h - Show this help message\n")
    else:
        print(f"Received: {cmd}{val if val is not None else ''}")



while True:
    # ledbuilt-in LED on pin 25 can be used for debugging
    LED.value(SLEEP.value())  # Turn on LED when motors are active
    read_command()
    read_button()
    values, pos = read_line(pos)
    error, PIDvalue = calc_PID(pos, error)
    motor_drive(PIDvalue)

    if debugMode:
        print(f"Values: {values}, Error: {error}, PID: {PIDvalue}")

    time.sleep(0.05)  # Adjust as needed