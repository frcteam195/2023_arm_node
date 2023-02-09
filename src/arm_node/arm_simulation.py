from ck_utilities_py_node.transform_links import *
from ck_utilities_py_node.rviz_shapes import *
import math

class ArmSimulation:
    def __init__(self) -> None:
        drivebase = Cube("robot_base", 1, "base_link")
        drivebase.set_scale(Scale(0.6096, 0.6096, 0.127))
        drivetrans = Transform()
        drivetrans.linear.z = 0.1143
        drivebase.set_transform(drivetrans)
        drivebase.set_color(Color(.5, 0, 1.0, 1.0))
        drivebase.publish()

        arm_cube = Cube("robot_parts", 2, "arm_base")
        arm_transform = Transform()
        arm_transform.linear.z = 0.56515
        arm_cube.set_transform(arm_transform)
        arm_cube.set_scale(Scale(0.0508, 0.508, 1.1303)) #44.5 tall, 20 apart
        arm_cube.set_color(Color(.7, .7, .7, 1.0))
        arm_cube.publish()

        arm_upper_cube = Cube("robot_parts", 3, "arm_upper")
        arm_upper_transform = Transform()
        arm_upper_transform.linear.z = 0.3048
        arm_upper_cube.set_transform(arm_upper_transform)
        arm_upper_cube.set_scale(Scale(0.0508, 0.381, 0.6096)) #24, 15, 2
        arm_upper_cube.set_color(Color(.7, .7, .7, 1.0))
        arm_upper_cube.publish()

        arm_extender_cube = Cube("robot_parts", 4, "arm_extender")
        arm_extender_transform = Transform()
        arm_extender_transform.linear.z = 0.2032 #12 in
        arm_extender_cube.set_transform(arm_extender_transform)
        arm_extender_cube.set_scale(Scale(0.0508, 0.304673, 0.4064)) #14 out (11.995, 16, 2)
        arm_extender_cube.set_color(Color(.7, .7, .7, 1.0))
        arm_extender_cube.publish()

        wrist_cube = Cube("robot_parts", 5, "wrist_link")
        wrist_transform = Transform()
        wrist_transform.linear.z = 0.2032 #12 in
        wrist_cube.set_transform(wrist_transform)
        wrist_cube.set_scale(Scale(0.1, 0.1, 0.01))
        wrist_cube.set_color(Color(1, .7, .7, 1.0))
        wrist_cube.publish()

        

    def publish_arm_base_link(self, degrees : float):
        transform = Transform()
        transform.linear.z = 0.24511 # Quarter of a meter up? - need to check cad for pivot point of base arm
        transform.angular.pitch = math.radians(degrees)

        transform_link = TransformLink("arm_base", "base_link") # originate from base link which is the center of the robot
        transform_link.set_transform(transform)
        transform_link.publish()


    def publish_arm_upper_link(self, degrees : float):
        transform = Transform()
        transform.linear.z = 0.847725 # 0.87630

        transform.angular.pitch = math.radians(degrees)

        transform_link = TransformLink("arm_upper", "arm_base")
        transform_link.set_transform(transform)
        transform_link.publish()


    def publish_arm_extender_link(self, degrees : float, extension : float):
        transform = Transform()

        transform.angular.pitch = math.radians(degrees)
        transform.linear.z = extension * 0.3556 + 0.2032

        transform_link = TransformLink("arm_extender", "arm_upper")
        transform_link.set_transform(transform)
        transform_link.publish()


    def publish_arm_wrist_link(self, degrees : float):
        transform = Transform()
        transform.angular.yaw = math.radians(degrees)
        transform_link = TransformLink("wrist_link", "arm_extender")
        transform_link.set_transform(transform)
        transform_link.publish()