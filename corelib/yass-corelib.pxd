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
    float x;
    float y;
    float z;

  cdef struct VectorSE2:
    float x;
    float y;
    float theta;

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

  # Field dimensions, in meters.
  cdef struct FieldDimensions:
    # Distance between goal lines
    float length;
    # Distance between touch lines
    float width;
    # Distance from the field lines to the boundary walls
    float boundary_width;
    # Distance between the inner edges of the goal posts
    float goal_width;
    # Distance from the goal line to the inner back of the goal
    float goal_depth;
    # Depth of the penalty area, measured between line centers
    float penalty_area_depth;
    # Width of the penalty area, measured between line centers
    float penalty_area_width;
    # Radius of the center circle, measured to the line center
    float center_circle_radius;
    # Thickness of the field lines
    float line_thickness;

  cdef struct SimulationState:
    Robot blue_robots[16];
    Robot yellow_robots[16];
    Ball ball;

  cdef enum MotionCommand_Tag:
    MotionCommand_Off,
    MotionCommand_Velocity,
    MotionCommand_Acceleration,
    MotionCommand_Wrench,

  cdef struct MotionCommand_Velocity_Body:
    float x;
    float y;
    float theta;

  cdef struct MotionCommand_Acceleration_Body:
    float x;
    float y;
    float theta;

  cdef struct MotionCommand_Wrench_Body:
    float x;
    float y;
    float theta;

  cdef struct MotionCommand:
    MotionCommand_Tag tag;
    MotionCommand_Velocity_Body velocity;
    MotionCommand_Acceleration_Body acceleration;
    MotionCommand_Wrench_Body wrench;

  cdef struct KickCommand:
    # Speed the ball should move at immediately after kick, in m/s
    float speed;
    # Angle above the ground the ball should be kicked, in radians
    float angle;

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
    float speed;

  cdef struct DribblerCommand_Force_Body:
    float force;

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
