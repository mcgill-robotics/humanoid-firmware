import random
import threading
from std_msgs.msg import Float32MultiArray
from odrive.utils import dump_errors
from odrive.enums import AxisState, ProcedureResult
from enum import Enum
import rospy
import os
import sys

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)

# TODO: Figure out why catkin on the Jetson isn't playing nice with this import. It worked on my PC -Eren
# import init_functions


class Node_servo_simple:
    def __init__(self):
        # SETPOINT VARIABLES, holds the setpoint for each joint
        self.joint_setpoint_dict = {
            "left_00": None,
            "left_01": None,
            "left_02": None,
        }

        # STATE VARIABLES, holds the current state of each joint
        self.joint_state_dict = {
            "left_00": None,
            "left_01": None,
            "left_02": None,
        }

        rospy.init_node("servo_node_simple")

        # Subscriptions
        self.state_subscriber = rospy.Subscriber(
            "/servo_state", Float32MultiArray, self.handle_servo_state
        )

        # Publishers
        self.command_publisher = rospy.Publisher(
            "/servo_cmd", Float32MultiArray, queue_size=1
        )

        # Frequency of the ODrive I/O
        self.rate = rospy.Rate(100)
        self.run()

    # Receive setpoint from external control node
    def handle_servo_state(self, msg):
        self.joint_state_dict["left_00"] = msg.data[0]
        self.joint_state_dict["left_01"] = msg.data[1]
        self.joint_state_dict["left_02"] = msg.data[2]

    # For future use
    def watchdog_func(self):
        pass

    def run(self):
        # SETUP
        self.watchdog_stop_event = threading.Event()
        self.watchdog_thread = threading.Thread(target=self.watchdog_func)
        self.watchdog_thread.start()

        # MAIN LOOP
        while not rospy.is_shutdown():
            # PRINT TIMESTAMP
            print(f"Time: {rospy.get_time()}")

            # PUBLISH SETPOINT, do additional logic (kinematics) here
            servo_cmd = Float32MultiArray()
            for joint_name, joint_pos in self.joint_setpoint_dict.items():
                try:
                    # Simulate random servo positions increment
                    temp = joint_pos + random.uniform(-20, 20)
                    self.joint_setpoint_dict[joint_name] = temp
                    servo_cmd.data.append(
                        temp
                    )
                # Default value in case lose connection to ODrive
                except:
                    servo_cmd.data.append(0.0)
                # Publish feedback
                self.command_publisher.publish(servo_cmd)

            # PRINT POSITIONS TO CONSOLE
            for joint_name, joint_pos in self.joint_state_dict.items():
                print(f"{joint_name}: {joint_pos}")
                pass

            print()
            self.rate.sleep()

        # On shutdown, bring motors to 0 position
        print("Shutdown prompt received. Setting all motors to idle state.")
        for joint_name, joint_pos in self.joint_state_dict.items():
            pass


if __name__ == "__main__":
    driver = Node_servo_simple()
    rospy.spin()
