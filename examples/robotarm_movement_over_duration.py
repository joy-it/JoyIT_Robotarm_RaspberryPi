import time
from MotoPi import MotoPi
from Robotarm import Robotarm

# MotoPi channels connected to the six servos.
# Adjust these numbers to match your wiring.
# values are sort from bottom to top
servo_channels = [0, 1, 2, 3, 4, 5]

# constant for duration of movement and duration of pauses in between
MOVEMENT_DURATION_MS = 2500
PAUSE_DURATION_S = 2.0

# Angle offsets relative to the configured home positions.
# These are example movements; adjust them to your robot arm.
pose_offsets = [
    [ 30, -30,  30, -30,  30, -30],
    [-30,  20, -20,  30, -30,  20],
    [  0,   0,   0,   0,   0,   0],
]

# open communication with MotoPi (RB-Moto3, I2C address 0x41, enable pin GPIO 7)
with MotoPi(address=0x41, enable_pin=7) as motopi:
    # set up robot arm with MotoPi and used channels
    robotarm = Robotarm(motopi, servo_channels)

    # begin communication with MotoPi
    if not robotarm.begin():
        raise SystemExit("MotoPi initialization failed.")

    # set all motors to home position
    robotarm.reset_all_positions()

    # save positions of robot arm into desired list
    home_angles = robotarm.get_position_of_all_servo()

    # counter for poses
    next_pose = 0

    # variables to control duration of pauses and movement
    waiting = True
    pause_start = time.monotonic()

    try:
        while True:
            # keep timed movements running
            robotarm.update()

            # check if movement of robot is finished
            if waiting:
                # check if time of pause is passed
                if time.monotonic() - pause_start >= PAUSE_DURATION_S:
                    # set target angles to the offset values with the home position
                    target_angles = [max(0, min(180, home + offset)) for home, offset in zip(home_angles, pose_offsets[next_pose])]

                    # set movement of all motors and select next pose
                    if robotarm.move_all_to(target_angles, MOVEMENT_DURATION_MS):
                        next_pose = (next_pose + 1) % len(pose_offsets)
                        waiting = False
            # if movement is finished start the next pause
            elif not robotarm.is_moving():
                pause_start = time.monotonic()
                waiting = True

            # short break to limit CPU load, update() is still called often enough
            time.sleep(0.01)
    except KeyboardInterrupt:
        # move all motors back to home position before exiting
        robotarm.release()
