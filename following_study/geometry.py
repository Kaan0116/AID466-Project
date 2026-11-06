"""Gap geometry shared by the follower, the capture scripts and the evaluation.

gap = distance, measured along the person's heading, from the robot's FRONT edge
      to the person's BACK surface.

    person_back = person_xy - person_back_offset * h          (h = person heading unit vector)
    robot_front = robot_xy  + robot_front_offset * r          (r = robot heading unit vector)
    gap         = (person_back - robot_front) . h
"""

import math
from dataclasses import dataclass


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Relation:
    gap: float          # m, along person heading
    lateral: float      # m, robot offset left (+) / right (-) of the person's path line
    heading_err: float  # rad, person_yaw - robot_yaw, wrapped
    bearing: float      # rad, direction of the person's back as seen from the robot front
    person_yaw: float
    robot_yaw: float


def relation_xy(px, py, pyaw, rx, ry, ryaw,
                person_back_offset: float, robot_front_offset: float) -> Relation:
    hx, hy = math.cos(pyaw), math.sin(pyaw)
    bx, by = px - person_back_offset * hx, py - person_back_offset * hy
    fx, fy = rx + robot_front_offset * math.cos(ryaw), ry + robot_front_offset * math.sin(ryaw)
    dx, dy = bx - fx, by - fy
    gap = dx * hx + dy * hy
    # robot position expressed in the person's frame (y axis = left of the person)
    lateral = -(rx - px) * hy + (ry - py) * hx
    bearing = wrap(math.atan2(dy, dx) - ryaw)
    return Relation(gap, lateral, wrap(pyaw - ryaw), bearing, pyaw, ryaw)


def relation(person_pose, robot_pose, person_back_offset, robot_front_offset) -> Relation:
    """Same as relation_xy, from geometry_msgs/Pose objects."""
    return relation_xy(person_pose.position.x, person_pose.position.y,
                       yaw_from_quat(person_pose.orientation),
                       robot_pose.position.x, robot_pose.position.y,
                       yaw_from_quat(robot_pose.orientation),
                       person_back_offset, robot_front_offset)


def robot_spawn_x(person_x, gap, person_back_offset, robot_front_offset, extra=0.0) -> float:
    """Robot centre x that puts the robot `gap (+extra)` behind a person facing +x."""
    return person_x - person_back_offset - gap - robot_front_offset - extra
