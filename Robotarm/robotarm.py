import time

class Robotarm():
    """
    Control the Joy-IT Robot02 robot arm with six servos connected to the RB-Moto3 (MotoPi).

    The servos are driven by the PCA9685 of the MotoPi with a 50 Hz PWM signal. Angles from
    0 to 180 degrees are mapped linearly onto a calibrated pulse width range (default 500 to 2500 us).
    Servo indices 0 to 5 are ordered from the bottom of the arm to the top.
    
    Timed movements and the optional SBC-WLGamepad are processed in update(), which must be
    called frequently from the main loop.
    """

    SERVO_COUNT = 6
    DEFAULT_HOME = (80, 100, 55, 85, 25, 140)
    DEFAULT_SERVO_MIN_US = 500
    DEFAULT_SERVO_MAX_US = 2500
    CONTROLLER_POLL_SECONDS = 0.020

    def __init__(self, motopi: "MotoPi", servo = range(6)):
        """
        Create the robot arm. No hardware is accessed until begin() is called.

        :param MotoPi motopi: opened MotoPi object that drives the servos
        :param servo: six distinct PWM channels (0 to 15) of the MotoPi, ordered from bottom to top (default: 0 to 5)

        :raises ValueError: if not exactly six distinct channels from 0 to 15 are given
        """
        servo = list(servo)
        if len(servo) != self.SERVO_COUNT or len(set(servo)) != self.SERVO_COUNT or any(channel < 0 or channel > 15 for channel in servo):
            raise ValueError("Provide six distinct PWM channels from 0 to 15")
        self._motopi = motopi
        self._channels = servo
        self._servo_pulse_width = self.SERVO_COUNT * [(self.DEFAULT_SERVO_MIN_US, self.DEFAULT_SERVO_MAX_US)]
        self._home_position = list(self.DEFAULT_HOME)
        self._position = self.SERVO_COUNT * [None]
        self._movement = [None] * self.SERVO_COUNT
        self._controller = None
        self._controller_enabled = False
        self._controller_ready = False
        self._initialized = False
        self._last_poll = None
        self._stick_speed = 45.0    # must be positive
        self._button_speed = 60.0   # must be positive
        self._deadzone = 24         # from 0 to 127

    @staticmethod
    def _check_servo(servo_index):
        """
        Validate a servo index. Return this servo index.

        :param int servo_index: servo index from 0 to 5

        :raises ValueError: if the index is out of range
        """
        if not 0 <= servo_index < Robotarm.SERVO_COUNT:
            raise ValueError("Servo index must be from 0 to 5")
        return servo_index

    @staticmethod
    def _check_angle(angle):
        """
        Return angle within the valid range from 0 to 180 degrees.

        :param float angle: requested angle in degrees
        """

        return max(0, min(180, angle))

    @staticmethod
    def _check_pulse_range(min, max):
        """
        Validate a pulse width range for a servo. Return tuple of (min, max).

        :param int min: pulse width for 0 degrees in microseconds
        :param int max: pulse width for 180 degrees in microseconds

        :raises ValueError: if the range does not satisfy 0 < min < max < 20000 us
        """
        if not 0 < min < max < 20_000:
            raise ValueError("Pulse width must satisfy 0 < min < max < 20.000 us")
        return min, max

    @staticmethod
    def _check_duration(duration_ms):
        """
        Validate the duration of a timed movement. Return this duration

        :param int duration_ms: duration in milliseconds

        :raises ValueError: if the duration is negative
        """
        if duration_ms < 0:
            raise ValueError("Duration must be non-negative")
        return duration_ms

    @staticmethod
    def _check_controller_reading(state):
        """
        Check whether a controller reading is valid. Return True if the reading is valid, otherwise false.
        Valid controller IDs are 0x41 (digital mode), 0x73 and 0x79 (analog modes).

        :param dict state: reading returned by WLGamepad.read_controller()
        """
        return isinstance(state, dict) and state.get("id") in (0x41, 0x73, 0x79)

    def _convert_controller_stick_value(self, value):
        """
        Convert a raw joystick value into a normalized axis value. Return axis value from -1.0 to 1.0.
        Values inside the dead zone around the center position (128) return 0.

        :param int value: raw joystick value from 0 to 255
        """
        offset = value - 128.0
        if abs(offset) <= self._deadzone:
            return 0.0
        return max(-1.0, min(1.0, offset / 127.0))

    def _apply_controller_input(self, servo_index, input_value, speed, dt):
        """
        Move a servo incrementally according to a controller input.
        The angle changes by input_value * speed * dt. Servos without a known position are ignored.
        Controller input cancels a running timed movement of the servo.

        :param int servo_index: servo index from 0 to 5
        :param float input_value: direction and strength of the input from -1.0 to 1.0
        :param float speed: maximum speed in degrees per second
        :param float dt: time since the last controller update in seconds

        :raises ValueError: if servo index or angle is invalid
        """
        servo_index = self._check_servo(servo_index)
        if input_value == 0 or self._position[servo_index] is None:
            return
        angle = self._check_angle(self._position[servo_index] + input_value * speed * dt)
        self.set_angle_of_servo(servo_index, angle)
    
    def _poll_controller(self, now):
        """
        Read the controller and move the servos accordingly.
        The controller is polled at most every CONTROLLER_POLL_SECONDS. The sticks control
        servos 0 to 3 (analog mode only), L1/R1 control servo 4 and L2/R2 control servo 5.
        The first valid reading after (re)connecting is only used to mark the controller as ready.

        :param float now: current time from time.monotonic() in seconds
        """
        if not self._controller_enabled or self._controller is None:
            return
        elapsed = now - self._last_poll
        if elapsed < self.CONTROLLER_POLL_SECONDS:
            return
        self._last_poll = now
        was_ready = self._controller_ready
        state = self._controller.read_controller()
        if not self._check_controller_reading(state):
            self._controller_ready = False
            return
        self._controller_ready = True
        if not was_ready:
            return
        dt = min(elapsed, 0.1)
        if state["id"] in (0x73, 0x79):
            sticks = state["sticks"]
            inputs = [self._convert_controller_stick_value(sticks["LX"]), -self._convert_controller_stick_value(sticks["LY"]), -self._convert_controller_stick_value(sticks["RY"]), self._convert_controller_stick_value(sticks["RX"])]
            for single_servo, input_value in enumerate(inputs):
                self._apply_controller_input(single_servo, input_value, self._stick_speed, dt)
        buttons = state["buttons"]
        self._apply_controller_input(4, int("L1" in buttons) - int("R1" in buttons), self._button_speed, dt)
        self._apply_controller_input(5, int("L2" in buttons) - int("R2" in buttons), self._button_speed, dt)
        
    def _require_begin(self):
        """
        Ensure that begin() was called before servos are controlled.

        :raises RuntimeError: if the robot arm is not initialized
        """
        if not self._initialized:
            raise RuntimeError("Call begin() before controlling servos")

    def _write_angle(self, servo_index, angle):
        """
        Output an angle to a servo without cancelling a running timed movement.
        The angle is mapped linearly onto the calibrated pulse width range of the servo
        and written to its MotoPi channel. The angle is stored as the current position.

        :param int servo_index: servo index from 0 to 5
        :param float angle: angle in degrees, limited to 0 to 180

        :raises ValueError: if servo index or angle is invalid
        """
        angle = self._check_angle(angle)
        servo_index = self._check_servo(servo_index)
        min_us, max_us = self._servo_pulse_width[servo_index]
        self._motopi.write_microseconds(self._channels[servo_index], min_us + (max_us - min_us) * angle / 180)
        self._position[servo_index] = angle

    def begin(self):
        """
        Initialize the robot arm and set the PWM frequency of the MotoPi to 50 Hz.
        If the controller was enabled before, it is set up now. Calling begin() again has no effect.
        Returns True when the robot arm is ready.
        """
        if self._initialized:
            return True
        self._motopi.frequency = 50
        self._initialized = True
        if self._controller_enabled:
            self.use_external_controller(True, self._controller)
        return True

    @property
    def controller_ready(self):
        """
        True if the controller delivered a valid reading at the last poll.
        """
        return self._controller_ready
    
    def set_servo_calibration(self, servo_index, min_pulse_us, max_pulse_us):
        """
        Set the pulse width range of a single servo.
        The new range is used from the next angle command on.

        :param int servo_index: servo index from 0 to 5
        :param int min_pulse_us: pulse width for 0 degrees in microseconds
        :param int max_pulse_us: pulse width for 180 degrees in microseconds

        :raises ValueError: if the servo index or the pulse width range is invalid
        """
        min_pulse_us, max_pulse_us = self._check_pulse_range(min_pulse_us, max_pulse_us)
        self._servo_pulse_width[self._check_servo(servo_index)] = (min_pulse_us, max_pulse_us)

    def set_pulse_width_of_all_servos(self, min_pulse_us, max_pulse_us):
        """
        Set the same pulse width range for all six servos.

        :param int min_pulse_us: pulse width for 0 degrees in microseconds
        :param int max_pulse_us: pulse width for 180 degrees in microseconds

        :raises ValueError: if the pulse width range is invalid
        """
        for single_servo in range(self.SERVO_COUNT):
            self.set_servo_calibration(single_servo, min_pulse_us, max_pulse_us)

    def set_angle_of_servo(self, servo_index, angle):
        """
        Move a selected servo immediately to an angle.
        A running timed movement of this servo is cancelled.

        :param int servo_index: servo index from 0 to 5
        :param float angle: target angle in degrees, limited to 0 to 180

        :raises RuntimeError: if begin() was not called
        :raises ValueError: if servo index or angle is invalid
        """
        self._require_begin()
        servo_index = self._check_servo(servo_index)
        self._write_angle(servo_index, angle)
        self._movement[servo_index] = None

    def set_angle_of_all_servos(self, angle):
        """
        Move all six servos immediately to the same angle.

        :param float angle: target angle in degrees, limited to 0 to 180

        :raises RuntimeError: if begin() was not called
        :raises ValueError: if angle is invalid
        """
        for single_servo in range(self.SERVO_COUNT):
            self.set_angle_of_servo(single_servo, angle)

    def get_position_of_servo(self, servo_index):
        """
        Returns the last commanded angle of a servo. Returns None if no angle was commanded yet.
        This is the commanded position, not a measured one.

        :param int servo_index: servo index from 0 to 5

        :raises ValueError: if servo index is invalid
        """
        return self._position[self._check_servo(servo_index)]

    def get_position_of_all_servo(self):
        """
        Returns a list of the last commanded angles of all six servos. 
        If a servo has no commanded angle, instead of a value there will be None.
        """
        return self._position.copy()

    def set_home_position(self, servo_index, angle):
        """
        Set the home position of a servo. The servo does not move until reset_position() is called.

        :param int servo_index: servo index from 0 to 5
        :param float angle: home angle in degrees, limited to 0 to 180

        :raises ValueError: if servo index or angle is invalid
        """
        self._home_position[self._check_servo(servo_index)] = self._check_angle(angle)

    def reset_position(self, servo_index):
        """
        Move a servo immediately to its home position.

        :param int servo_index: servo index from 0 to 5

        :raises ValueError: if servo index is invalid
        """
        self.set_angle_of_servo(servo_index, self._home_position[self._check_servo(servo_index)])

    def reset_all_positions(self):
        """
        Move all six servos immediately to their home positions.
        """
        for single_servo in range(self.SERVO_COUNT):
            self.reset_position(single_servo)

    def move_to(self, servo_index, angle, duration_ms):
        """
        Start a timed movement of a servo to an angle. Returns False if start position of the servo is unknown, otherwise True.
        The method returns immediately; call update() frequently to perform the movement.
        A duration of 0 moves the servo immediately.

        :param int servo_index: servo index from 0 to 5
        :param float angle: target angle in degrees, limited to 0 to 180
        :param int duration_ms: duration of the movement in milliseconds

        :raises RuntimeError: if begin() was not called
        :raises ValueError: if the servo index, the angle or the duration is invalid
        """
        self._require_begin()
        servo_index = self._check_servo(servo_index)
        duration_ms = self._check_duration(duration_ms)
        if self._position[servo_index] is None:
            return False
        if duration_ms == 0:
            self.set_angle_of_servo(servo_index, angle)
        else:
            self._movement[servo_index] = (self._position[servo_index], self._check_angle(angle), time.monotonic(), duration_ms)
        return True

    def move_all_to(self, angles, duration_ms):
        """
        Start a timed movement of all six servos to a pose. Return False if the start position of any servo is unknown, otherwise True
        All servos start and finish at the same time. 
        The method returns immediately, call update() frequently to perform the movement. A duration of 0 moves the servos immediately.

        :param angles: six target angles in degrees, ordered from bottom to top
        :param int duration_ms: duration of the movement in milliseconds
        
        :raises ValueError: if not exactly six valid angles are given or the duration is invalid
        :raises RuntimeError: if begin() was not called
        """
        self._require_begin()
        angles = [self._check_angle(angle) for angle in angles]
        duration_ms = self._check_duration(duration_ms)
        if len(angles) != self.SERVO_COUNT:
            raise ValueError("Provide exactly 6 target angles")
        if any(position is None for position in self._position):
            return False
        if duration_ms == 0:
            for single_servo, angle in enumerate(angles):
                self.set_angle_of_servo(single_servo, angle)
        else:
            now = time.monotonic()
            for single_servo, angle in enumerate(angles):
                self._movement[single_servo] = (self._position[single_servo], angle, now, duration_ms)
        return True

    def is_moving(self):
        """
        Check whether a timed movement is still running. Returns True if at least one servo is moving, otherwise False.
        """
        return any(move is not None for move in self._movement)

    def use_external_controller(self, external=True, controller=None):
        """
        Enable or disable control with the SBC-WLGamepad. Returns True if the controller is ready or was disabled, False if it did not answer
        If called before begin(), the controller is only marked as enabled and is set up in begin().
        Without a controller object, a WLGamepad with its default pins is created.

        :param bool external: True to enable, False to disable the controller (default: True)
        :param controller: optional controller object with setup_controller() and read_controller()
        """
        if controller is not None:
            self._controller = controller
        if not external:
            self._controller_ready = False
            self._controller_enabled = False
            self._last_poll = None
            return True
        if not self._initialized:
            self._controller_enabled = True
            self._controller_ready = False
            self._last_poll = None
            return True
        if self._controller is None:
            from wlgamepad import WLGamepad
            self._controller = WLGamepad()
        self._controller.setup_controller()
        state = self._controller.read_controller()
        self._controller_ready = self._check_controller_reading(state)
        self._last_poll = time.monotonic()
        self._controller_enabled = True
        return self.controller_ready

    def update(self):
        """
        Advance timed movements and poll the controller.
        Call this method frequently from the main loop. If an error occurs, all movements
        are stopped, the controller is marked as not ready and the error is raised again.
        """
        if not self._initialized:
            return
        now = time.monotonic()
        try:
            for single_servo, movement in enumerate(self._movement):
                if movement is None:
                    continue
                start_angle, target_angle, start_time, duration_ms = movement
                progress = max(0.0, min(1.0, (now - start_time) * 1000.0 / duration_ms))
                self._write_angle(single_servo, start_angle + (target_angle - start_angle) * progress)
                if progress >= 1.0:
                    self._movement[single_servo] = None
            self._poll_controller(now)
        except Exception:
            self.stop()
            self._controller_ready = False
            raise

    def stop(self):
        """
        Stop all timed movements. The servos keep their current angles.
        """
        self._movement[:] = self.SERVO_COUNT * [None]

    def release(self):
        """
        Disable the controller, stop all movements and move the servos to their home positions.
        """
        self.use_external_controller(False)
        self.stop()
        if self._initialized:
            self.reset_all_positions()