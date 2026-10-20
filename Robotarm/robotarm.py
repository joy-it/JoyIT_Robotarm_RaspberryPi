import adafruit_motor
import RPi.GPIO as GPIO
import time
import math

class Robotarm():
    SERVO_COUNT = 6
    DEFAULT_HOME = [90, 90, 20, 110, 30, 150]
    DEFAULT_SERVO_MIN_US = 500
    DEFAULT_SERVO_MAX_US = 2500
    CONTROLLER_POLL_SECONDS = 0.020

    def __init__(self, motopi: "MotoPi", servo = range(6)):
        if len(servo) != self.SERVO_COUNT or any(channel < 0 or channel > 15 for channel in servo):
            raise ValueError("Provide six distinct PWM channels from 0 to 15")
        self._motopi = motopi
        self._servos = [adafruit_motor.servo.Servo(self._motopi.channels[i], min_pulse=self.DEFAULT_SERVO_MIN_US, max_pulse=self.DEFAULT_SERVO_MAX_US) for i in servo]
        self._home_position = self.DEFAULT_HOME
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
    def _check_servo(servo):
        if not 0 <= servo < Robotarm.SERVO_COUNT:
            raise ValueError("Servo index must be from 0 to 5")
        return servo

    @staticmethod
    def _check_angle(angle):
        return max(0, min(180, angle))

    @staticmethod
    def _check_pulse_range(min, max):
        if not 0 < min < max < 20_000:
            raise ValueError("Pulse width must satisfy 0 < min < max < 20.000 us")
        return min, max

    @staticmethod
    def _check_duration(duration_ms):
        if duration_ms < 0:
            raise ValueError("Duration must be non-negative")
        return duration_ms

    @staticmethod
    def _valid_state(self, state):
        return isinstance(state, dict) and state.get("id") in (0x41, 0x73, 0x79)

    def _controller_axis(self, value):
        offset = max(0.0, min(255.0, value - 128.0))
        if abs(offset) <= self._deadzone:
            return 0.0
        return max(-1.0, min(1.0, offset / 127.0))

    def _apply_controller_input(self, servo, input_value, speed, dt):
        serv = self._check_servo(servo)
        if input_value == 0 or self._position[servo] is None:
            return
        angle = self._check_angle(self._position[servo] + input_value * speed * dt)
        self.set_angle_of_servo(servo, angle)
    
    def _poll_controller(self, now):
        if not self._controller_enabled:
            return
        elapsed = now - self._last_poll
        if elapsed < self.CONTROLLER_POLL_SECONDS:
            return
        self._last_poll = now
        was_ready = self.controller_ready
        state = self._controller.read_controller()
        if not self._valid_state(state):
            self.controller_ready = False
            return
        if not was_ready:
            return
        dt = min(elapsed, 0.1)
        if state["id"] in (0x73, 0x79):
            sticks = state["sticks"]
            inputs = [self._controller_axis(sticks["LX"]), -self._controller_axis(sticks["LY"]), -self._controller_axis(sticks["RY"]), self._controller_axis(sticks["RX"])]
            for single_servo, input_value in inputs:
                self._apply_controller_input(single_servo, input_value, self._stick_speed, dt)
            buttons = state["buttons"]
            self._apply_controller_input(4, int("L1" in buttons) - int("R1" in buttons), self._button_speed, dt)
            self._apply_controller_input(5, int("L2" in buttons) - int("R2" in buttons), self._button_speed, dt)

        
    def _require_begin(self):
        if not self._initialized:
            raise RuntimeError("Call begin() before controlling servos")

    def begin(self):
        if self._initialized:
            return True
        self._motopi.frequency = 50
        self._initialized = True
        if self._controller_enabled:
            self.useExternalController(True)
        return True

    @property
    def controller_ready(self):
        return self._controller_ready
    
    def set_servo_calibration(self, servo, min_pulse_us, max_pulse_us):
        min_pulse_us, maxPulseUs = self._check_pulse_range(min_pulse_us, maxPulseUs)
        self._servos[self._check_servo(servo)].set_pulse_width_range(min_pulse=min_pulse_us, max_pulse=max_pulse_us)

    def set_pulse_width_of_all_servos(self, min_pulse_us, max_pulse_us):
        for single_servo in self._servos:
            self.set_servo_calibration(single_servo, min_pulse_us, max_pulse_us)

    def set_angle_of_servo(self, servo, angle):
        self._require_begin()
        angle, servo = self._check_angle(angle), self._check_servo(servo)
        self._servos[servo].angle = angle
        self._position[servo] = angle
        self._movement[servo] = None

    def set_angle_of_all_servos(self, angle):
        for single_servo in range(self.SERVO_COUNT):
            self.set_angle_of_servo(single_servo, angle)

    def get_position_of_servo(self, servo):
        return self._position[self._check_servo(servo)]

    def get_position_of_all_servo(self):
        return self._position.copy()

    def set_home_position(self, servo, angle):
        self._home_position[self._check_servo(servo)] = self._check_angle(angle)

    def reset_position(self, servo):
        self.set_angle_of_servo(servo, self._home_position[self._check_servo(servo)])

    def reset_all_positions(self):
        for single_servo in range(self.SERVO_COUNT):
            self.reset_position(single_servo)

    def move_to(self, servo, angle, duration_ms):
        self._require_begin()
        servo = self._check_servo(servo)
        if self._position is None:
            return False
        if duration_ms == 0:
            self.set_angle_of_servo(servo, angle)
        else:
            self._movement[servo] = (self._position[servo], self._check_angle(angle), time.monotonic(), self._check_duration(duration_ms))
        return True

    def move_all_to(self, angles, duration_ms):
        self._require_begin()
        angles = [self._check_angle(angle) for angle in angles]
        if len(angles) != self.SERVO_COUNT:
            raise ValueError("Provide exactly 6 target angles")
        if any(position is None for position in self._position):
            return False
        if duration_ms == 0:
            for single_servo, angle in angles:
                self.set_angle_of_servo(single_servo, angle)
        else:
            now = time.monotonic()
            for single_servo, angle in angles:
                self._movement[single_servo] = (self._position[single_servo], angle, now, self._check_duration(duration_ms))
        return True

    def is_moving(self):
        return any(move is not None for move in self._movement)

    def use_external_controller(self, external=True, controller=None):
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
        if controller is None:
            from wlgamepad import WLGamepad
            self._controller = WLGamepad()
        self._controller.setup_controller()
        state = self._controller.read_controller()
        self._controller_ready = self._valid_state(state)
        self._last_poll = time.monotonic()
        self._controller_enabled = True
        return self.controller_ready

    def update(self):
        if not self._initialized:
            return
        now = time.monotonic()
        try:
            for single_servo, movement in self._movement:
                if movement is None:
                    continue
                start_angle, target_angle, start_time, duration = movement
                progress = max(0.0, min(1.0, (now - start_time) / duration))
                self.set_angle_of_servo(single_servo, start_angle + (target_angle - start_angle) * progress)
                if progress >= 1.0:
                    self._movement[single_servo] = None
            self._poll_controller(now)
        except Exception:
            self.stop()
            self.controller_ready = False
            raise

    def stop(self):
        self._movement[:] = self.SERVO_COUNT * [None]

    def release(self):
        self.use_external_controller(False)
        self.stop()
        try:
            self.reset_all_positions()
        finally:
            self._position[:] = self._home_position[:]