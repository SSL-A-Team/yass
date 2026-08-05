from libc.stdint cimport int8_t, int16_t, int32_t, int64_t, intptr_t
from libc.stdint cimport uint8_t, uint16_t, uint32_t, uint64_t, uintptr_t
cdef extern from *:
  ctypedef bint bool
  ctypedef struct va_list

cdef extern from *:

  cdef enum TeamColor:
    TeamColor_Blue,
    TeamColor_Yellow,

  cdef enum MotionCommandFrame:
    MotionCommandFrame_Local,
    MotionCommandFrame_Global,

  cdef struct Vector3:
    double x;
    double y;
    double z;

  cdef struct VectorSE2:
    double x;
    double y;
    double theta;

  cdef struct Robot:
    uint8_t id;
    bool present;
    VectorSE2 pose;
    VectorSE2 velocity;
    VectorSE2 acceleration;
    bool ball_sense;

  cdef struct Ball:
    Vector3 position;
    Vector3 velocity;
    Vector3 acceleration;

  cdef struct SimulationState:
    Robot blue_robots[16];
    Robot yellow_robots[16];
    Ball ball;

  cdef enum MotionCommand_Tag:
    MotionCommand_Off,
    MotionCommand_Velocity,
    MotionCommand_Force,

  cdef struct MotionCommand_Velocity_Body:
    double x;
    double y;
    double theta;

  cdef struct MotionCommand_Force_Body:
    double x;
    double y;
    double theta;

  cdef struct MotionCommand:
    MotionCommand_Tag tag;
    MotionCommand_Velocity_Body velocity;
    MotionCommand_Force_Body force;

  cdef enum KickCommand_Tag:
    # Kicker does not fire
    KickCommand_Off,
    # Kicker fires immediately at the given speed, in m/s
    KickCommand_KickImmediate,
    # Kicker fires if the ball is touching the front of the robot, in m/s
    KickCommand_KickOnTouch,

  cdef struct KickCommand_KickImmediate_Body:
    double speed;

  cdef struct KickCommand_KickOnTouch_Body:
    double speed;

  cdef struct KickCommand:
    KickCommand_Tag tag;
    KickCommand_KickImmediate_Body kick_immediate;
    KickCommand_KickOnTouch_Body kick_on_touch;

  cdef enum DribblerCommand_Tag:
    # Ball will not stick to the dribbler
    DribblerCommand_Off,
    # Ball will glue to the front of the robot
    DribblerCommand_Sticky,
    # Ball will spin towards the robot, in rad/s
    DribblerCommand_Rotation,
    # Ball will be pulled towards the front of the robot with the given force, in N
    DribblerCommand_Force,

  cdef struct DribblerCommand_Rotation_Body:
    double speed;

  cdef struct DribblerCommand_Force_Body:
    double force;

  cdef struct DribblerCommand:
    DribblerCommand_Tag tag;
    DribblerCommand_Rotation_Body rotation;
    DribblerCommand_Force_Body force;

  cdef struct RobotCommand:
    uint8_t id;
    MotionCommand motion_command;
    MotionCommandFrame motion_command_frame;
    KickCommand kick_command;
    DribblerCommand dribbler_command;
