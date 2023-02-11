from ck_utilities_py_node.transform_links import *
from ck_utilities_py_node.rviz_shapes import *
import math
from ck_utilities_py_node.ckmath import *

class ArmSimulation:
    def __init__(self) -> None:
        #Bumpers
        drivebase = Cube("base_link")
        drivebase.set_scale(Scale(0.6096, 0.6096, 0.127))
        drivetrans = Transform()
        drivetrans.linear.z = 0.1143
        drivebase.set_transform(drivetrans)
        drivebase.set_color(Color(.5, 0, 1.0, 1.0))
        drivebase.publish()


        base_arm_1 = Cube("arm_base")
        base_arm_1_transform = Transform()
        base_arm_1_transform.linear.y = inches_to_meters(9.5)
        base_arm_1_transform.linear.z = inches_to_meters(18.25)
        base_arm_1.set_transform(base_arm_1_transform)
        base_arm_1.set_scale(Scale(inches_to_meters(2), inches_to_meters(1), inches_to_meters(44.5))) #44.5 tall, 20 apart
        base_arm_1.set_color(Color(.7, .7, .7, 1.0))
        base_arm_1.publish()

        base_arm_2 = Cube("arm_base")
        base_arm_2_transform = Transform()
        base_arm_2_transform.linear.y = inches_to_meters(-9.5)
        base_arm_2_transform.linear.z = inches_to_meters(18.25)
        base_arm_2.set_transform(base_arm_2_transform)
        base_arm_2.set_scale(Scale(inches_to_meters(2), inches_to_meters(1), inches_to_meters(44.5))) #44.5 tall, 20 apart
        base_arm_2.set_color(Color(.7, .7, .7, 1.0))
        base_arm_2.publish()

        base_arm_cross = Cube("arm_base")
        base_arm_cross_transform = Transform()
        base_arm_cross_transform.linear.z = inches_to_meters(40)
        base_arm_cross.set_transform(base_arm_cross_transform)
        base_arm_cross.set_scale(Scale(inches_to_meters(2), inches_to_meters(18), inches_to_meters(1))) #44.5 tall, 20 apart
        base_arm_cross.set_color(Color(.7, .7, .7, 1.0))
        base_arm_cross.publish()

        upper_arm_1 = Cube("arm_upper")
        upper_arm_1_transform = Transform()
        upper_arm_1_transform.linear.y = inches_to_meters(7)
        upper_arm_1_transform.linear.z = inches_to_meters(8)
        upper_arm_1.set_transform(upper_arm_1_transform)
        upper_arm_1.set_scale(Scale(inches_to_meters(2), inches_to_meters(1), inches_to_meters(24))) #24, 15, 2
        upper_arm_1.set_color(Color(.7, .7, .7, 1.0))
        upper_arm_1.publish()

        upper_arm_2 = Cube("arm_upper")
        upper_arm_2_transform = Transform()
        upper_arm_2_transform.linear.y = inches_to_meters(-7)
        upper_arm_2_transform.linear.z = inches_to_meters(8)
        upper_arm_2.set_transform(upper_arm_2_transform)
        upper_arm_2.set_scale(Scale(inches_to_meters(2), inches_to_meters(1), inches_to_meters(24))) #24, 15, 2
        upper_arm_2.set_color(Color(.7, .7, .7, 1.0))
        upper_arm_2.publish()

        upper_arm_cross = Cube("arm_upper")
        upper_arm_cross_transform = Transform()
        upper_arm_cross_transform.linear.z = inches_to_meters(-3.5)
        upper_arm_cross.set_transform(upper_arm_cross_transform)
        upper_arm_cross.set_scale(Scale(inches_to_meters(2), inches_to_meters(13), inches_to_meters(1)))
        upper_arm_cross.set_color(Color(.7, .7, .7, 1.0))
        upper_arm_cross.publish()

        arm_extender_1_cube = Cube("arm_extender")
        arm_extender_1_transform = Transform()
        arm_extender_1_transform.linear.y = inches_to_meters(-5.420734)
        arm_extender_1_cube.set_transform(arm_extender_1_transform)
        arm_extender_1_cube.set_scale(Scale(inches_to_meters(2), inches_to_meters(1), inches_to_meters(18)))
        arm_extender_1_cube.set_color(Color(.7, .7, .7, 1.0))
        arm_extender_1_cube.publish()

        arm_extender_2_cube = Cube("arm_extender")
        arm_extender_2_transform = Transform()
        arm_extender_2_transform.linear.y = inches_to_meters(5.420734)
        arm_extender_2_cube.set_transform(arm_extender_2_transform)
        arm_extender_2_cube.set_scale(Scale(inches_to_meters(2), inches_to_meters(1), inches_to_meters(18)))
        arm_extender_2_cube.set_color(Color(.7, .7, .7, 1.0))
        arm_extender_2_cube.publish()

        arm_extender_cross_cube = Cube("arm_extender")
        arm_extender_cross_transform = Transform()
        arm_extender_cross_transform.linear.z = inches_to_meters(8.5)
        arm_extender_cross_cube.set_transform(arm_extender_cross_transform)
        arm_extender_cross_cube.set_scale(Scale(inches_to_meters(2), inches_to_meters(10), inches_to_meters(1)))
        arm_extender_cross_cube.set_color(Color(.7, .7, .7, 1.0))
        arm_extender_cross_cube.publish()

        wrist_cube = Cube("wrist_link")
        wrist_transform = Transform()
        wrist_transform.linear.z = 0.005
        wrist_cube.set_transform(wrist_transform)
        wrist_cube.set_scale(Scale(inches_to_meters(5), inches_to_meters(5), inches_to_meters(1.5)))
        wrist_cube.set_color(Color(1, .7, .7, 1.0))
        wrist_cube.publish()

        

    def publish_arm_base_link(self, degrees : float):
        transform = Transform()
        transform.linear.z = inches_to_meters(7.75222441) #Arm rotation point is 7.75222441 in off the ground
        transform.angular.pitch = math.radians(degrees)

        transform_link = TransformLink("arm_base", "base_link") # originate from base link which is the center of the robot
        transform_link.set_transform(transform)
        transform_link.publish()


    def publish_arm_upper_link(self, degrees : float):
        transform = Transform()
        transform.linear.z = inches_to_meters(34.50000000) #Upper joint rotation is 34.5 in above base joint

        transform.angular.pitch = math.radians(degrees)
        transform.angular.roll = math.radians(180)
        transform.angular.yaw = math.radians(180)

        transform_link = TransformLink("arm_upper", "arm_base")
        transform_link.set_transform(transform)
        transform_link.publish()


    def publish_arm_extender_link(self, extension : float):
        transform = Transform()
        transform.linear.z = extension * inches_to_meters(12) + inches_to_meters(12.436533)

        transform_link = TransformLink("arm_extender", "arm_upper")
        transform_link.set_transform(transform)
        transform_link.publish()


    def publish_arm_wrist_link(self, degrees : float):
        transform = Transform()
        transform.angular.yaw = math.radians(degrees)
        transform.linear.z = inches_to_meters(9.763878)
        transform_link = TransformLink("wrist_link", "arm_extender")
        transform_link.set_transform(transform)
        transform_link.publish()