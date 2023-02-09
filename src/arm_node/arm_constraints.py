from ck_utilities_py_node.constraints import *
from dataclasses import dataclass

@dataclass
class ArmConstraints:
    middle_robot_conflict : KeepoutRange = KeepoutRange(mechanism_a_lower_limit=0,
                                                        mechanism_a_upper_limit=0,
                                                        mechanism_a_limit_value_if_constrained=0,
                                                        mechanism_b_lower_limit=0,
                                                        mechanism_b_upper_limit=0,
                                                        mechanism_b_limit_value_if_constrained=0,
                                                        system_to_limit=LimitSystem.Both)
