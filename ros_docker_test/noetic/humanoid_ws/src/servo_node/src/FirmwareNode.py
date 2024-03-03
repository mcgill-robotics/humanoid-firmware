import rospy
import random
import threading

from ServoJoint import ServoJoint
from servo_node.msg import ServoCommand, ServoFeedback


class FirmwareNode:
    def __init__(self):
        # Joint dictionary
        self.joint_dict = {
            "left_leg_ankle": ServoJoint(),
            "left_leg_knee": ServoJoint(),
            "left_leg_hip_pitch": ServoJoint(),
            "left_leg_hip_roll": ServoJoint()
        }

        rospy.init_node("FirmwareNode")

        # Subscribers
        self.feedback_subscriber = rospy.Subscriber(
            "/servosFeedback", ServoFeedback, self.parseFeedback
        )

        # Publishers
        self.command_publisher = rospy.Publisher("/servosCommand", ServoCommand, queue_size=1)

        # Frequency of the Servo I/O
        self.rate = rospy.Rate(100)
        self.run()

    # Receive setpoint from external control node
    def parseFeedback(self, msg):
        for name, joint in self.joint_dict.items():
            joint.storeFeedback(getattr(msg, name + "_fb"))

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
            # servo_cmd = Float32MultiArray()
            servo_cmd = ServoCommand()
            for name, joint in self.joint_dict.items():
                temp_setpoint = random.uniform(0, 30)
                temp_setpoint += 150
                # setattr(servo_cmd, name, joint.getSetpoint())
                setattr(servo_cmd, name, temp_setpoint)

            # Publish command to servo
            self.command_publisher.publish(servo_cmd)

            # PRINT POSITIONS TO CONSOLE
            # for joint_name, joint_obj in self.joint_dict.items():
            #     print(f"{joint_name}: {joint_obj.get_pos_deg()}")
            #     pass

            # print()
            self.rate.sleep()

        # On shutdown, bring motors to 0 position
        if rospy.is_shutdown():
            print("Shutdown prompt received. Setting 0 angle.")
            servo_cmd = ServoCommand()
            for name, joint in self.joint_dict.items():
                setattr(servo_cmd, name, 150.0)
                joint.setAngle(150.0)
            self.command_publisher.publish(servo_cmd)
                
    def setJointAngle(self, name, angle):
        self.joint_dict[name].setAngle(angle)
        
    def getJointPosition(self, name):
        return self.joint_dict[name].getPosition()
    
    def getJointVelocity(self, name):
        return self.joint_dict[name].getVelocity()
    
    def getJointLoad(self, name):
        return self.joint_dict[name].getLoad()


if __name__ == "__main__":
    driver = FirmwareNode()
    rospy.spin()
