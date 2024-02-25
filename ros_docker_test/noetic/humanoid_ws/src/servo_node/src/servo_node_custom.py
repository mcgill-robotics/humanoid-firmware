import rospy
import os
import sys
import random
import threading
from std_msgs.msg import Float32MultiArray
from enum import Enum
from ServoJoint import ServoJoint
from servo_node.msg import ServoState


currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)


# TODO change message type to custom message
class Node_servo_simple:
    def __init__(self):
        # SETPOINT VARIABLES, holds the setpoint for each joint
        self.joint_dict = {
            "left_ankle": None,
            "left_knee": None,
            "right_ankle": None,
            "right_knee": None,
        }

        self.joint_dict["left_ankle"] = ServoJoint(16, "left_ankle")
        self.joint_dict["left_knee"] = ServoJoint(17, "left_knee")
        self.joint_dict["right_ankle"] = ServoJoint(18, "right_ankle")
        self.joint_dict["right_knee"] = ServoJoint(19, "right_knee")

        rospy.init_node("servo_node_custom")

        # Subscriptions
        self.state_subscriber = rospy.Subscriber(
            "/servo_state", ServoState, self.handle_servo_state
        )

        # Publishers
        self.command_publisher = rospy.Publisher("/servo_cmd", ServoState, queue_size=1)

        # Frequency of the ODrive I/O
        self.rate = rospy.Rate(100)
        self.run()

    # Receive setpoint from external control node
    def handle_servo_state(self, msg):
        self.joint_dict["left_ankle"].set_pos_deg(msg.left_ankle)
        self.joint_dict["left_knee"].set_pos_deg(msg.left_knee)
        self.joint_dict["right_ankle"].set_pos_deg(msg.right_ankle)
        self.joint_dict["right_knee"].set_pos_deg(msg.right_knee)
        for joint_name, joint_obj in self.joint_dict.items():
            if joint_obj.pos_raw < 0 or joint_obj.pos_raw > 1023:
                joint_obj.is_faulted = True
                print(f"{joint_name} is faulted or disconnected.")

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
            for joint_name, joint_obj in self.joint_dict.items():
                print(f"Setting {joint_name} to {joint_obj}")
                if joint_obj.is_faulted:
                    print(f"Setting {joint_obj.name} to 0")
                    servo_cmd.data.append(0.0)
                    continue
                # Simulate random servo positions increment for testing
                temp = joint_obj.get_pos_deg() + random.uniform(-20, 20)

                print(f"Setting {joint_name} to {temp}")
                joint_obj.set_setpoint_deg(temp)
                servo_cmd.data.append(temp)

            # Publish command to servo
            self.command_publisher.publish(servo_cmd)

            # PRINT POSITIONS TO CONSOLE
            for joint_name, joint_obj in self.joint_dict.items():
                print(f"{joint_name}: {joint_obj.get_pos_deg()}")
                pass

            print()
            self.rate.sleep()

        # On shutdown, bring motors to 0 position
        if rospy.is_shutdown():
            print("Shutdown prompt received. Setting 0 angle.")
            for joint_name, joint_obj in self.joint_dict.items():
                pass


if __name__ == "__main__":
    driver = Node_servo_simple()
    rospy.spin()
