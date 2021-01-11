# Originally made by Nica (ncai@rrc.ca)
# Modified "reportDistance(self)" part by Jungwon for the determine the direction and distance.

import pigpio
import time
import math

class Wheel:
    """
        Wheel class of Wheelchair
        The wheelchair would have two wheels
        The Wheel class depends on pigpio module
    """
    def __init__(self, pi, gpioA, gpioB, callback, left_mount = True):
        """
        Define pins and callback function,
        left mounted rotary encoder and right mounted have diffrent step (1 / -1)
        piGPIO callback function is called when EITHER_EDGE reached
        """
        self.pi = pi
        self.gpioA = gpioA
        self.gpioB = gpioB
        self.callback = callback
        self.pos = 0
        self.left_mount = left_mount
        self.step = 1 if left_mount else -1

        self.levA = 0
        self.levB = 0

        self.lastGpio = None

        self.pi.set_mode(gpioA, pigpio.INPUT)
        self.pi.set_mode(gpioB, pigpio.INPUT)

        self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
        self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

        self.cbA = self.pi.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
        self.cbB = self.pi.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)

    ''' _pulse function is called when defined event condition raised
        See: https://github.com/joan2937/pigpio/blob/master/EXAMPLES/Python/ROTARY_ENCODER/rotary_encoder.py
    '''

    def _pulse(self, gpio, level, tick):
        if gpio == self.gpioA:
            self.levA = level
        else:
            self.levB = level

        if gpio != self.lastGpio:
            self.lastGpio = gpio

            if gpio == self.gpioA and level == 1:
                if self.levB == 1:
                    self.pos += self.step
                    self.callback(self.pos, self.left_mount, True) # Moving clockwise
            elif gpio == self.gpioB and level == 1:
                if self.levA == 1:
                    self.pos -= self.step
                    self.callback(self.pos, self.left_mount, False)  # Moving counter-clockwise

    def cancel(self):
        self.cbA.cancel()
        self.cbB.cancel()


class Wheels:
    """
        Wheels class combines two Wheels
        path: the position of two wheels and time
        route: the location of the wheelchair and time
        reset_wheels: reset the above information to (0,0)
    """

    def left_callback(self, way, left_mount = True, clockwise = True):
        forward = clockwise # Move forward when left wheel goes clockwise
        self.update_loc(left_mount, forward)
        self.path.append(((self.leftWheel.pos, self.rightWheel.pos), self.facing, time.time()))

    def righ_callback(self, way, left_mount = True, clockwise = True):
        forward = not clockwise # Move forward when right wheel goes counter clockwise
        self.update_loc(left_mount, forward)
        self.path.append(((self.leftWheel.pos, self.rightWheel.pos), self.facing, time.time()))

    def update_loc(self, LEFT = True, forward = True):

        deltaLoc = 2 * math.pi * self.RADIUS / self.TICKS_PER_REV
        if not forward :
            deltaLoc = - deltaLoc

        if LEFT :
            deltaAngle = deltaLoc / self.LENGTH
            # .loc is meter based wheel position
            self.loc = (self.loc[0] + deltaLoc / 2 * math.cos(deltaAngle), self.loc[1] + deltaLoc / 2 * math.sin(deltaAngle))
        else:
            deltaAngle = - deltaLoc / self.LENGTH
            self.loc = (self.loc[0] + deltaLoc / 2 * math.cos(deltaAngle), self.loc[1] + deltaLoc / 2 * math.sin(deltaAngle))
        self.facing += deltaAngle

        if self.facing < -math.pi :
            self.facing += 2 * math.pi
        elif self.facing > math.pi :
            self.facing -= 2 * math.pi

        self.route.append((self.loc, self.facing, time.time()))
        self.log.append(((self.leftWheel.pos, self.rightWheel.pos), self.loc, self.facing, time.time()))

    def __init__(self, pi, left_A, left_B, right_A, right_B, RADIUS = 0.3, LENGTH = 0.6100, TICKS_PER_REV = 1200):
        """
            initial two wheels
            The default length between the two wheels is 0.6100 meters
            The default radius of the wheel is 0.3 meters
            The radius of the wheel and distance between the two wheel has great impact of the movement calculation
            Due to the nature of measuring, the data and calculations based on that are not accurate enough for long-time gaming/training.
            Other sensors should be used to assist the calculation so a satisfying result can be researched.
        """
        self.RADIUS = RADIUS
        self.LENGTH = LENGTH
        self.TICKS_PER_REV = TICKS_PER_REV

        self.last_loc = (0,0)
        self.last_facing = 0

        self.init_wheels()
        self.leftWheel = Wheel(pi, left_A, left_B, self.left_callback, True)
        self.rightWheel = Wheel(pi, right_A, right_B, self.righ_callback, False)

    def reportDistance(self):
        (distance, delta_angle) = (0,0)
        if(len(self.route)>0):
            self.new_loc = (self.route[-1][0][0], self.route[-1][0][1])
            delta_left = self.new_loc[0] - self.last_loc[0]
            delta_right = self.new_loc[1] - self.last_loc[1]
            delta_angle = (-self.facing + self.last_facing) 
            distance = (delta_left + delta_right)/2    # Modified the distance calculation
            self.last_loc = self.new_loc
            self.last_facing = self.facing
        return distance, delta_angle

    def get_path(self, num_of_pos = 0):
        return self.path[-num_of_pos:]

    def get_route(self, num_of_loc = 0):
        return self.route[-num_of_loc:]

    def get_log(self, num_of_log = 0):
        return self.log[-num_of_log:]

    # report the direction the wheelchair is facing
    # the value is in radius in 1 pi range
    # the value is a radius value from the beginning direction of the wheelchair
    # the value is positive when moved to the right
    # the value is negative when moved to the left
    def get_facing(self):
        return self.facing

    # report the current location of the Wheelchair
    # the location is the calculate value in meters
    # (1,2) means 1 meter forward and 2 meters to the right
    def get_loc(self):
        return self.loc

    def init_wheels(self):
        self.facing = 0
        self.wheel_pos = (0,0)
        self.path = []
        self.path.append(((0,0), self.facing, time.time()))
        self.route = []
        self.loc = (0,0)
        self.log = []
        self.log.append((self.wheel_pos, self.loc, self.facing, time.time()))

    def reset_wheels(self):
        self.init_wheels()

    def cancel(self):
        self.leftWheel.cancel()
        self.rightWheel.cancel()


def main():
    import time
    import pigpio
    import Wheel

    pi = pigpio.pi()

    wheels = Wheels(pi, 23, 27, 17, 22)
    wheel_pos = (0,0)
    i = 0
    try:
        while True:
            time.sleep(10)
            logs = wheels.log
            with open('../logs/log'+str(i)+'.csv', "w") as log_file:
                for line in logs:
                    log_file.write("".join(str(line)) + "\n")

    finally:
        wheels.cancel()

    pi.stop()


if __name__ == '__main__': main()
