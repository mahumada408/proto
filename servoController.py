import time

# Import the PCA9685 module.
import Adafruit_PCA9685

class Servo(object):

    def __init__(self, driver_board, channel):

        self.driver_board = driver_board
        self.channel = channel

        # Configure min and max servo pulse lengths
        self.servo_min = 100  # Min pulse length out of 4096
        self.servo_max = 600  # Max pulse length out of 4096

    # Helper function to make setting a servo pulse width simpler.
    def set_servo_pulse(self, pulse):
        self.driver_board.set_pwm(self.channel, 0, pulse)


if __name__ == "__main__":
    Servo()
