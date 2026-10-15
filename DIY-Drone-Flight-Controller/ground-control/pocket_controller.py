import pygame
import serial
import time

# ---------- SERIAL ----------
ser = serial.Serial('COM4', 115200, timeout=1)
time.sleep(2)  # wait for Nano reset

# ---------- PYGAME ----------
pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()
    
print("Using joystick:", joystick.get_name())

# ---------- FUNCTIONS ----------
def axis_to_arduino(val):
    """Convert -1.0 .. 1.0 → 0 .. 1023 for joysticks"""
    return int((val + 1.0) * 511.5)

def axis_to_3pos(val, deadzone=0.3):
    """
    Snap analog axis to -1, 0, 1
    deadzone avoids noise around center
    """
    if val > deadzone:
        return 1
    elif val < -deadzone:
        return -1
    else:
        return 0

# ---------- MAIN LOOP ----------
while True:
    pygame.event.pump()

    # Axes
    joy1x = axis_to_arduino(joystick.get_axis(0))
    joy1y = axis_to_arduino(joystick.get_axis(2))
    joy2x = axis_to_arduino(joystick.get_axis(1))
    joy2y = axis_to_arduino(joystick.get_axis(3))

    # Buttons / switches
    btn1 = joystick.get_button(0)
    btn2 = axis_to_3pos(joystick.get_axis(4))
    btn3 = axis_to_3pos(joystick.get_axis(5))
    btn4 = axis_to_3pos(joystick.get_axis(6))
    btn5 = axis_to_3pos(joystick.get_axis(7))

    # Send all 9 channels
    data = f"{joy1x},{joy1y},{joy2x},{joy2y},{btn1},{btn2},{btn3},{btn4},{btn5}\n"
    ser.write(data.encode())

    # Debug print
    print(data.strip())

    time.sleep(0.02)  # 50 Hz
