"""Decoders for WPILib struct-serialized NetworkTables topics.

DogLog publishes Pose2d/ChassisSpeeds as raw struct bytes; these decode them.
"""

import math
import struct

# (x meters, y meters, heading degrees)
Pose2dTuple = tuple[float, float, float]
# (vx m/s, vy m/s, omega rad/s)
ChassisSpeedsTuple = tuple[float, float, float]


def decode_pose2d(data: bytes) -> Pose2dTuple | None:
    """Decode a struct:Pose2d payload (translation x/y + rotation radians, little-endian)."""
    if len(data) != 24:
        return None
    x, y, rotation_radians = struct.unpack("<3d", data)
    return (x, y, math.degrees(rotation_radians))


def decode_chassis_speeds(data: bytes) -> ChassisSpeedsTuple | None:
    """Decode a struct:ChassisSpeeds payload (vx, vy, omega radians/sec, little-endian)."""
    if len(data) != 24:
        return None
    vx, vy, omega = struct.unpack("<3d", data)
    return (vx, vy, omega)
