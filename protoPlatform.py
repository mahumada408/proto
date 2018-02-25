
from math import *
import numpy as np


class StewartPlatform(object):

    def __init__(self):

        # constants based on geometry
        base_radius = 0.06
        platform_radius = 0.05
        # base
        base_mounting_theta_s = 15 * pi / 180
        base_mounting_theta_l = 90 * pi / 180
        # platform
        platform_mounting_theta_s = 6.84 * pi / 180
        platform_mounting_theta_l = 106.31 * pi / 180

        self.servo_horn_length = 0.016
        self.pushrod_length = 0.138

        # beta on the base
        self.beta = [base_mounting_theta_s + pi/2, base_mounting_theta_s*2 + pi/2, base_mounting_theta_l + base_mounting_theta_s*2 + pi/2,
                     base_mounting_theta_l + base_mounting_theta_s*3 + pi/2, base_mounting_theta_l*2 + base_mounting_theta_s*3 + pi/2,
                     base_mounting_theta_l*2 + base_mounting_theta_s*4 + pi/2]

        self.servo_zeros = [pi, 0, pi, 0, pi, 0]
        self.servo_angle = []
        self.r_ob_ai_b = []
        self.r_ai_pi_b = []
        self.eff_length = []

        # initialize container of vectors
        self.r_ob_bi_b = []
        self.r_op_pi_p = []
        self.r_op_pi_b = []

        # initialize last servo vectors
        self.last_servo_angles = self.servo_zeros


        # fill up with vectors
        base_angle_step = 0
        platform_angle_step = 0
        for i in range(0, 6):
            # set up vectors to mounting points on the base and platform

            if i == 2 or i == 4:
                base_angle_step += base_mounting_theta_l
                platform_angle_step += platform_mounting_theta_l
            else:
                base_angle_step += base_mounting_theta_s
                platform_angle_step += platform_mounting_theta_s

            # base in b frame
            self.r_ob_bi_b.append(np.asarray([[base_radius * cos(base_angle_step)],
                                              [base_radius * sin(base_angle_step)],
                                              [0]]))

            # platform in p frame
            self.r_op_pi_p.append(np.asarray([[platform_radius * cos(platform_angle_step)],
                                              [platform_radius * sin(platform_angle_step)],
                                              [0]]))

        print("stewart init done")

    def update_platform(self, desired_platform_pose):

        # simplifications for rotation matrix later based on desired position
        sin_psi = sin(desired_platform_pose[3])
        cos_psi = cos(desired_platform_pose[3])
        sin_theta = sin(desired_platform_pose[4])
        cos_theta = cos(desired_platform_pose[4])
        sin_rho = sin(desired_platform_pose[5])
        cos_rho = cos(desired_platform_pose[5])

        # creating rotation matrix
        rotation_matrix_p_b = np.asarray([[cos_psi * cos_theta, -1 * sin_psi * cos_rho + cos_psi * sin_theta * sin_rho,
                                           sin_psi * sin_rho + cos_psi * sin_theta * cos_rho],
                                          [sin_psi * cos_theta, cos_psi * cos_rho + sin_psi * sin_theta * sin_rho,
                                           -1 * cos_psi * sin_rho + sin_psi * sin_theta * cos_rho],
                                          [-1 * sin_theta, cos_theta * sin_rho, cos_theta * cos_rho]])

        # vector Ob -> Op
        self.r_ob_op = np.asarray([[desired_platform_pose[0]], [desired_platform_pose[1]], [desired_platform_pose[2]]])

        # pi now in B
        self.r_op_pi_b = []
        for vector in self.r_op_pi_p:
        #    self.r_op_pi_b.append(rotation_matrix_p_b * vector))
            hey_hey = np.dot(rotation_matrix_p_b, vector)
            self.r_op_pi_b.append(hey_hey)
            
        # solving for bi -> pi in b frame
        self.r_bi_pi_b = []

        for i in range(0,6):
            r_bi_pi_b_vector = self.r_ob_op + self.r_op_pi_b[i] - self.r_ob_bi_b[i]
            self.r_bi_pi_b.append(r_bi_pi_b_vector)

        # adding origin for plotting purposes
        self.r_bi_pi_b.append(self.r_ob_op)

        # solving for ob -> pi
        self.r_ob_pi_b = []
        for i in range(0,6):
            self.r_ob_pi_b.append(self.r_ob_bi_b[i] + self.r_bi_pi_b[i])
            
        self.get_servo_angle()

    def get_servo_angle(self):
        self.servo_angle = []
        self.r_ob_ai_b = []

        for i in range(0, 6):

            L = np.power(np.linalg.norm(self.r_bi_pi_b[i]), 2) - (np.power(self.pushrod_length, 2) -
                                                    np.power(self.servo_horn_length, 2))
            M = 2 * self.servo_horn_length * self.r_ob_pi_b[i][2][0]
            N = 2 * self.servo_horn_length * (cos(self.beta[i]) * (self.r_ob_pi_b[i][0][0] - self.r_ob_bi_b[i][0][0]) +
                                              sin(self.beta[i]) * (self.r_ob_pi_b[i][1][0] - self.r_ob_bi_b[i][1][0]))

            oh_hey = L / sqrt(pow(M, 2) + pow(N, 2))

            if oh_hey < -1:
                if i % 2 == 1:
                    current_servo_angle = -1*pi/2

                else:
                    current_servo_angle = 3*pi/2

            elif oh_hey > 1:
                current_servo_angle = pi/2

            else:
                alpha = asin(oh_hey) - atan2(N, M)
                if i % 2 == 1:
                    current_servo_angle = self.servo_zeros[i] + alpha

                else:
                    current_servo_angle = self.servo_zeros[i] - alpha

            #print(" oh hey: " + str(oh_hey))

            self.servo_angle.append(current_servo_angle)
            #print("servo: " + str(i) + " current angle: " + str(current_servo_angle*180/pi))

            # self.last_servo_angles[i] = current_servo_angle

            x_ai = self.servo_horn_length * cos(self.servo_angle[i]) * cos(self.beta[i]) + self.r_ob_bi_b[i][0][0]
            y_ai = self.servo_horn_length * cos(self.servo_angle[i]) * sin(self.beta[i]) + self.r_ob_bi_b[i][1][0]
            z_ai = self.servo_horn_length * sin(self.servo_angle[i])

            self.r_ob_ai_b.append(np.asarray([[x_ai], [y_ai], [z_ai]]))


    def go_home(self):
        # home conditions
        home_x = 0
        home_y = 0
        home_height = 0.13456

        home_pose = [home_x, home_y, home_height, 0, 0, 0]

        self.update_platform(home_pose)

    def output_variables(self):

        # print(self.r_ob_ai_b)
        return self.r_ob_bi_b, self.r_ob_pi_b, self.r_ob_ai_b, self.servo_angle


if __name__ == "__main__":

    StewartPlatform()

