import signal
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

    # begin communication with MotoPi (also sets the PWM frequency to 50 Hz)
    if not robotarm.begin():
        raise SystemExit("MotoPi initialization failed.")

    # set all motors (except the claw) to neutral position
    for channel in servo_channels[:-1]:
        motopi.write_microseconds(channel, 1500)

    # set claw to a slightly different position
    motopi.write_microseconds(servo_channels[-1], 1600)

    # keep the signals active while mounting, leaving the with block switches all outputs off
    print("Servos are in mounting position. Press Ctrl+C to exit.")
    try:
        signal.pause()
    except KeyboardInterrupt:
        pass