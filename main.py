import threading
import math

from Robot_Control import RobotControl
from TR_Controller import MyController
from _Odometry import _Odometry

from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

FLAG_MODULE_MOTORS = True

# robot_ini_pose = [sdnl_targets[10][0], sdnl_targets[10][1]]
robot_ini_pose = [0, 0]

x_val = []
y_val = []

controller_flag = True

def thread_controller(name):
    controller.listen()

if __name__ == "__main__":

    # class constructor
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    th_con = threading.Thread(target=thread_controller, args=(1,), daemon=True)
    th_con.start()

    if FLAG_MODULE_MOTORS:
        robot = RobotControl()
        enc_odom = _Odometry(robot, coord_rel_x=robot_ini_pose[0], coord_rel_y=robot_ini_pose[1])

    # when the main processor first establishes communication with the motor controller,
    # it first needs to update some values that may already been altered in the motor controller
    if FLAG_MODULE_MOTORS:
        robot.set_omni_flags(robot.RESET_ENCODERS, True)
        robot.set_omni_variables(robot.ACCELERATION, 1)
        print(robot.get_omni_variables(robot.ACCELERATION))

    rospy.init_node('odometry_publisher')

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    #origem e velocidade inicial
    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0.0
    vy = 0.0
    vth = 0.0

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    #frequência de publicacao de poses (frequência >= images & lidar)
    r = rospy.Rate(30.0) 

    while not rospy.is_shutdown():

        if controller.analogs_updated:
            #print(controller.L3dist)
            if FLAG_MODULE_MOTORS:
                if controller.L3dist == 0.0:  # < 0.15 sometimes when the analog stick is let go the last event is not zero
                    robot.omni_move(lin_=int(0))
                else:
                    if controller_flag:
                        robot.omni_move(dir_=int(controller.L3ang), lin_=int(controller.L3dist*100/3))
                    elif not controller_flag:
                        robot.omni_move(dir_=int(controller.L3ang), lin_=int(controller.L3dist*100/5))
                    # print(controller_flag)

                robot.omni_move(ang_=100+int(-controller.R3dist*math.sin(math.radians(controller.R3ang))*20))

            #print(controller.R3dist)
            # print(int(-controller.R3dist*math.sin(math.radians(controller.R3ang))*20))
                # print(controller_flag)

            controller.analogs_updated = False
            # in this case it is still possible to filter the number of packages sent to the motor board
            # filter: if the value is very similar to the previous value there is no need to resend the values

        if controller.buttons_updated:

            if controller.button_state(controller.R1) == controller.RISING:
                target_number += 1
                if target_number > 25:
                    target_number = 0
            if controller.button_state(controller.L1) == controller.RISING:
                target_number -= 1
                if target_number < 0:
                    target_number = 25

            if FLAG_MODULE_MOTORS:
                if controller.button_state(controller.ARROW_UP) == controller.RISING_OR_ON:
                    robot.set_omni_flags(robot.LIN_ACT_LEGS_MOVEM, True)
                    robot.set_omni_flags(robot.LIN_ACT_LEGS_ACTIVE, True)

                if controller.button_state(controller.ARROW_UP) == controller.FALLING_OR_OFF:
                    robot.set_omni_flags(robot.LIN_ACT_LEGS_ACTIVE, False)

                if controller.button_state(controller.ARROW_DOWN) == controller.RISING_OR_ON:
                    robot.set_omni_flags(robot.LIN_ACT_LEGS_MOVEM, False)
                    robot.set_omni_flags(robot.LIN_ACT_LEGS_ACTIVE, True)

                if controller.button_state(controller.ARROW_DOWN) == controller.FALLING_OR_OFF:
                    robot.set_omni_flags(robot.LIN_ACT_LEGS_ACTIVE, False)

                if controller.button_state(controller.ARROW_RIGHT) == controller.RISING_OR_ON:
                    robot.set_omni_flags(robot.LIN_ACT_TORSO_MOVEM, True)
                    robot.set_omni_flags(robot.LIN_ACT_TORSO_ACTIVE, True)

                if controller.button_state(controller.ARROW_RIGHT) == controller.FALLING_OR_OFF:
                    robot.set_omni_flags(robot.LIN_ACT_TORSO_ACTIVE, False)

                if controller.button_state(controller.ARROW_LEFT) == controller.RISING_OR_ON:
                    robot.set_omni_flags(robot.LIN_ACT_TORSO_MOVEM, False)
                    robot.set_omni_flags(robot.LIN_ACT_TORSO_ACTIVE, True)

                if controller.button_state(controller.ARROW_LEFT) == controller.FALLING_OR_OFF:
                    robot.set_omni_flags(robot.LIN_ACT_TORSO_ACTIVE, False)

            controller.buttons_updated = False

        if FLAG_MODULE_MOTORS:
            coord_rel_x_, coord_rel_y_, coord_rel_t = enc_odom.encoder_odometry()
            # start = time.time()
            x_val.append(coord_rel_x_)
            y_val.append(coord_rel_y_)

            print("X: ", coord_rel_x_, ",Y: ", coord_rel_y_, ",theta: ", coord_rel_t)

        current_time = rospy.Time.now()

        #buscar ao codigo_tiago_odom
        x = coord_rel_x_/1000
        y = coord_rel_y_/1000
        th = coord_rel_t/1000

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        odom_pub.publish(odom)

        last_time = current_time
        r.sleep()


