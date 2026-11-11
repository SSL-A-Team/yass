#ifndef YASS_CORELIB_HPP
#define YASS_CORELIB_HPP

#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

namespace yass {

enum class TeamColor {
  Blue,
  Yellow,
};

enum class MotionCommandFrame {
  Local,
  Global,
};

struct Vector3 {
  float x;
  float y;
  float z;
};

struct VectorSE2 {
  float x;
  float y;
  float theta;
};

struct Robot {
  uint8_t id;
  bool present;
  VectorSE2 pose;
  VectorSE2 velocity;
  VectorSE2 acceleration;
  bool ball_sense;
};

struct Ball {
  Vector3 position;
  Vector3 velocity;
  Vector3 acceleration;
};

struct SimulationState {
  Robot blue_robots[16];
  Robot yellow_robots[16];
  Ball ball;
};

struct MotionCommand {
  enum class Tag {
    Off,
    Velocity,
    Acceleration,
    Wrench,
  };

  struct Velocity_Body {
    float x;
    float y;
    float theta;
  };

  struct Acceleration_Body {
    float x;
    float y;
    float theta;
  };

  struct Wrench_Body {
    float x;
    float y;
    float theta;
  };

  Tag tag;
  union {
    Velocity_Body velocity;
    Acceleration_Body acceleration;
    Wrench_Body wrench;
  };
};

struct KickCommand {
  /// Speed the ball should move at immediately after kick, in m/s
  float speed;
  /// Angle above the ground the ball should be kicked, in radians
  float angle;
};

struct DribblerCommand {
  enum class Tag {
    /// Ball will not stick to the dribbler
    Off,
    /// Ball will glue to the front of the robot
    Sticky,
    /// Ball will spin towards the robot, in rad/s
    Rotation,
    /// Ball will be pulled towards the front of the robot with the given force, in N
    Force,
  };

  struct Rotation_Body {
    float speed;
  };

  struct Force_Body {
    float force;
  };

  Tag tag;
  union {
    Rotation_Body rotation;
    Force_Body force;
  };
};

struct RobotCommand {
  uint8_t id;
  MotionCommand motion_command;
  MotionCommandFrame motion_command_frame;
  KickCommand kick_command;
  DribblerCommand dribbler_command;
};

}  // namespace yass

#endif  // YASS_CORELIB_HPP
