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

        intake_rectangle = Cube("intake")
        intake_rectangle_transform = Transform()
        intake_rectangle.set_transform(intake_rectangle_transform)
        intake_rectangle.set_scale(Scale(inches_to_meters(2), inches_to_meters(8), inches_to_meters(10))) 
        intake_rectangle.set_color(Color(.7, .7, .7, 1.0))
        intake_rectangle.publish()

        wide_intake_rectangle = Cube("wide_intake")
        wide_intake_rectangle_transform = Transform()
        wide_intake_rectangle.set_transform(wide_intake_rectangle_transform)
        wide_intake_rectangle.set_scale(Scale(inches_to_meters(8), inches_to_meters(8), inches_to_meters(2))) 
        wide_intake_rectangle.set_color(Color(.7, .7, .7, 1.0))
        wide_intake_rectangle.publish()

        intake_arrow = Arrow("intake_arrow")
        intake_arrow_transform = Transform()
        intake_arrow_transform.linear.y = inches_to_meters(-4)
        intake_arrow.set_transform(intake_arrow_transform)
        intake_arrow.set_scale(Scale(0.5, 0.1, 0.1))
        intake_arrow.set_color(Color(0.949, 0.875, 0.027, 1.0)) 
        intake_arrow.publish()

        intake_arrow_2 = Arrow("intake_arrow")
        intake_arrow_2_transform = Transform()
        intake_arrow_2_transform.linear.y = inches_to_meters(4)
        intake_arrow_2.set_transform(intake_arrow_2_transform)
        intake_arrow_2.set_scale(Scale(0.5, 0.1, 0.1))
        intake_arrow_2.set_color(Color(0.533, 0.141, 0.878, 1.0)) 
        intake_arrow_2.publish()

        intake_support_1_cube = Cube("intake_support")
        intake_support_1_transform = Transform()
        intake_support_1_transform.linear.y = inches_to_meters(-5.420734)
        intake_support_1_cube.set_transform(intake_support_1_transform)
        intake_support_1_cube.set_scale(Scale(inches_to_meters(2), inches_to_meters(1), inches_to_meters(9)))
        intake_support_1_cube.set_color(Color(.7, .7, .7, 1.0))
        intake_support_1_cube.publish()

        intake_support_2_cube = Cube("intake_support")
        intake_support_2_transform = Transform()
        intake_support_2_transform.linear.y = inches_to_meters(5.420734)
        intake_support_2_cube.set_transform(intake_support_2_transform)
        intake_support_2_cube.set_scale(Scale(inches_to_meters(2), inches_to_meters(1), inches_to_meters(9)))
        intake_support_2_cube.set_color(Color(.7, .7, .7, 1.0))
        intake_support_2_cube.publish()

    def publish_intake_link(self, degrees : float):
        transform = Transform()
        transform.linear.z = inches_to_meters(14)
        transform.angular.pitch = math.radians(degrees)

        transform_link = TransformLink("intake", "arm_extender")
        transform_link.set_transform(transform)
        transform_link.publish()

    def publish_wide_intake_link(self, degrees : float):
        transform = Transform()
        transform.linear.z = inches_to_meters(5)
        transform.angular.pitch = 0#math.radians(degrees)

        transform_link = TransformLink("wide_intake", "intake")
        transform_link.set_transform(transform)
        transform_link.publish()
    
    def publish_intake_arrow_link(self, pitch_degrees : float, intake: float):
        transform = Transform()
        
        if intake > 0 :
            transform.angular.pitch = math.radians(pitch_degrees)
            transform.linear.z = 1 + 0.1
        elif intake < 0:
            transform.angular.pitch = math.radians(-pitch_degrees)
            transform.linear.z = 0.5 + 0.1
        else:
            transform.linear.z = 100

        transform_link = TransformLink("intake_arrow", "arm_extender")
        transform_link.set_transform(transform)
        transform_link.publish()
    
    def publish_intake_support_link(self):
        transform = Transform()
        transform.linear.z = inches_to_meters(9)

        transform_link = TransformLink("intake_support", "arm_extender")
        transform_link.set_transform(transform)
        transform_link.publish()

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

    