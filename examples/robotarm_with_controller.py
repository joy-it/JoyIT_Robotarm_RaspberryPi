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

    # set pulse width of all used servo motors
    robotarm.set_pulse_width_of_all_servos(500, 2500)

    # set all motors to home position
    robotarm.reset_all_positions()

    # activate use with controller (SBC-WLGamepad with default pins ATT=17, CLK=18, CMD=27, DATA=22)
    if not robotarm.use_external_controller(True):
        print("Controller not ready yet, check receiver and analog mode.")

    # variable to slow output down
    last_print = 0.0

    try:
        while True:
            # poll the gamepad and update servo movements
            robotarm.update()

            # output position of motors every second
            if time.monotonic() - last_print >= 1.0:
                last_print = time.monotonic()
                positions = []
                for i, angle in enumerate(robotarm.get_position_of_all_servo()):
                    # if motor position is still unknown
                    if angle is None:
                        positions.append(f"Servo {i+1}: unknown")
                    else:
                        positions.append(f"Servo {i+1}: {angle:.0f} deg")
                print(" | ".join(positions))

            # short break to limit CPU load, the controller is polled every 20 ms
            time.sleep(0.005)
    except KeyboardInterrupt:
        # disable controller and move all motors back to home position before exiting
        robotarm.release()