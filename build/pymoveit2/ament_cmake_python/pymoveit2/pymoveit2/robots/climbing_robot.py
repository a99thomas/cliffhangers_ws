from typing import List

MOVE_GROUP_LEFT_ARM: str = "left_arm"
MOVE_GROUP_RIGHT_ARM: str = "right_arm"


robot_prefix = "climbing_robot"

def left_arm_joint_names(prefix: str = robot_prefix) -> List[str]:
    return [
        "l1",
        "l2",
        "l3_1",
        "l3_roll_joint",
        "l3_pitch_joint",
        "l3_yaw_joint"
    ]
def right_arm_joint_names(prefix: str = robot_prefix) -> List[str]:
    return [
        "r1",
        "r2",
        "r3_1",
        "r3_roll_joint",
        "r3_pitch_joint",
        "r3_yaw_joint"
    ]


def base_link_name(prefix: str = robot_prefix) -> str:
    return "assembly_7"


def right_arm_tip_link_name(prefix: str = robot_prefix) -> str:
    return "r3_yaw"

def left_arm_tip_link_name(prefix: str = robot_prefix) -> str:
    return "l3_yaw"