import time
from MotoPi import MotoPi
from Robotarm import Robotarm

# MotoPi channels connected to the six servos.
# Adjust these numbers to match your wiring.
# values are sort from bottom to top
servo_channels = [0, 1, 2, 3, 4, 5]

# open communication with MotoPi (RB-Moto3, I2C address 0x41, enable pin GPIO 7)
with MotoPi(address=0x41, enable_pin=7) as motopi:
    # set up robot arm with MotoPi and used channels
    robotarm = Robotarm(motopi, servo_channels)

    # begin communication with MotoPi
    if not robotarm.begin():
        raise SystemExit("MotoPi initialization failed.")

    # set all motors to home position
    robotarm.reset_all_positions()

    try:
        while True:
            robotarm.set_angle_of_servo(0, 20)
            time.sleep(1)

            robotarm.set_angle_of_servo(0, 160)
            time.sleep(1)
    except KeyboardInterrupt:
        # move all motors back to home position before exiting
        robotarm.reset_all_positions()