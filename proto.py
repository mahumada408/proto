from math import *
import time
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
#from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import protoPlatform # my own class!!
from tkinter import *

# Import the PCA9685 module.
import Adafruit_PCA9685
import servoController


def main():

    # setting up the plot
    #plt.ion()
    #fig = plt.figure()
    #ax = Axes3D(fig)


    # stewart platform object from platform class
    stewart = protoPlatform.StewartPlatform()

    # gui stuff
    gui = Tk()
    x_slider, y_slider, z_slider, roll_slider, pitch_slider, yaw_slider = gui_stuff(gui, stewart)
    gui_list = [x_slider, y_slider, z_slider, roll_slider, pitch_slider, yaw_slider]

    global end_flag
    end_flag = False

    # Initialise the PCA9685 using the default address (0x40).
    driver_board = Adafruit_PCA9685.PCA9685()
    # Set frequency to 60hz, good for servos.
    driver_board.set_pwm_freq(60)
    servos = [servoController.Servo(driver_board,0), servoController.Servo(driver_board,1), servoController.Servo(driver_board,2),
              servoController.Servo(driver_board, 3), servoController.Servo(driver_board,4), servoController.Servo(driver_board,5)]

    while not end_flag:

        # desired platform pose
        desired_platform_pose = [0 + x_slider.get()/10000, 0 + y_slider.get()/10000, 0.122 + z_slider.get()/10000,
                                 roll_slider.get()/1000, pitch_slider.get()/1000, yaw_slider.get()/1000]

        # update platform pose by solving the IK
        stewart.update_platform(desired_platform_pose)

        # get base and platform vertices for plotting
        r_ob_bi_b, r_ob_pi_b , r_ob_ai_b, servo_angles = stewart.output_variables()

        set_servo_angles(servos, servo_angles)

        # plot_stuff(ax, r_ob_bi_b, r_ob_pi_b, r_ob_ai_b)

        gui.update()

        event_checker(stewart, gui_list)

        #plt.draw()

    #plt.ioff()
    #plt.show()



def event_checker(stewart, gui_list):
    global end_flag
    global home_flag

    if home_flag == True:
        stewart.go_home()
        print("homes")

        x = gui_list[0]
        y_axis = gui_list[1]
        z = gui_list[2]
        r = gui_list[3]
        p = gui_list[4]
        y = gui_list[5]
        x.set(0)
        y_axis.set(0)
        z.set(0)
        r.set(0)
        p.set(0)
        y.set(0)
        home_flag = False


def end_simulator():
    global end_flag
    end_flag = True

def plot_stuff(ax, r_ob_bi_b, r_ob_pi_b, r_ob_ai_b):

    # clear plot for next iteration
    ax.clear()
    # plotting
    # plots mounting vertices

    for i in range(0, 6):

        ax.scatter(r_ob_bi_b[i][0], r_ob_bi_b[i][1], r_ob_bi_b[i][2])
        ax.scatter(r_ob_pi_b[i][0], r_ob_pi_b[i][1], r_ob_pi_b[i][2])
        ax.scatter(r_ob_ai_b[i][0], r_ob_ai_b[i][1], r_ob_ai_b[i][2])

        # draw lines between the bottom base and top plate
        ax.plot([r_ob_bi_b[i][0][0], r_ob_pi_b[i][0][0]],
                [r_ob_bi_b[i][1][0], r_ob_pi_b[i][1][0]],
                [r_ob_bi_b[i][2][0], r_ob_pi_b[i][2][0]], linewidth = 0.3)
        # draw in lines for servo arm and push rod
        ax.plot([r_ob_bi_b[i][0][0], r_ob_ai_b[i][0][0]],
                [r_ob_bi_b[i][1][0], r_ob_ai_b[i][1][0]],
                [r_ob_bi_b[i][2][0], r_ob_ai_b[i][2][0]])
        ax.plot([r_ob_pi_b[i][0][0], r_ob_ai_b[i][0][0]],
                [r_ob_pi_b[i][1][0], r_ob_ai_b[i][1][0]],
                [r_ob_pi_b[i][2][0], r_ob_ai_b[i][2][0]])

    # 3d polygon for top platform

    mount_point = []
    verts = []

    for i in range(0,6):
        mount_point.append(r_ob_pi_b[i][0][0])
        mount_point.append(r_ob_pi_b[i][1][0])
        mount_point.append(r_ob_pi_b[i][2][0])

        verts.append(mount_point)

        mount_point = []

    ax.add_collection3d(Poly3DCollection([verts]))
    ax.grid()
    ax.set_xlim(-0.08, 0.08)
    ax.set_ylim(-0.08, 0.08)
    ax.set_zlim(0, 0.2)


def gui_stuff(gui, stewart):

    x_slider = Scale(gui, label="x", from_=-200, to=200)
    x_slider.pack()
    x_slider.set(0)
    y_slider = Scale(gui, label="y", from_=-200, to=200)
    y_slider.pack()
    y_slider.set(0)
    z_slider = Scale(gui, label="z", from_=-200, to=200)
    z_slider.pack()
    z_slider.set(0)
    roll_slider = Scale(gui, label="roll", from_=-200, to=200)
    roll_slider.pack()
    pitch_slider = Scale(gui, label="pitch", from_=-200, to=200)
    pitch_slider.pack()
    yaw_slider = Scale(gui, label="yaw", from_=-200, to=200)
    yaw_slider.pack()
    home_button = Button(gui, text="Home Platform", command=home_callback).pack()
    end_button = Button(gui, text="End this shit", command=end_simulator).pack()

    return x_slider, y_slider, z_slider, roll_slider, pitch_slider, yaw_slider


def home_callback():
    global home_flag
    home_flag = True

def set_servo_angles(servos, servo_angles):
    for i in range(0,6):
        servo = servos[i]
        # transform angle to pulse
        servo_pulse = int(round((25000/157)*servo_angles[i] + 350))
        if i == 1:
            print(servo_angles[i]*(180/pi))
            print(servo_pulse)
        servo.set_servo_pulse(servo_pulse)



if __name__ == '__main__':
    global end_flag
    global end_buton
    global home_flag

    end_flag = False
    end_buton = False
    home_flag = False
    main()
