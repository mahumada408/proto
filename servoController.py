import time

# Import the PCA9685 module.
import Adafruit_PCA9685

class Servo(object, channel):

    def __init__(self, driver_board, channel):

        self.driver_board = driver_board
        self.channel = channel

        # Configure min and max servo pulse lengths
        self.servo_min = 150  # Min pulse length out of 4096
        self.servo_max = 600  # Max pulse length out of 4096

    # Helper function to make setting a servo pulse width simpler.
    def set_servo_pulse(self, pulse):
        pulse_length = 1000000  # 1,000,000 us per second
        pulse_length //= 60  # 60 Hz
        pulse_length //= 4096  # 12 bits of resolution
        pulse *= 1000
        pulse //= pulse_length
        self.driver_board.set_pwm(self.channel, 0, pulse)


if __name__ == "__main__":
    Servo()
