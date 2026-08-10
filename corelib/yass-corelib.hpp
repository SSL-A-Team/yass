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
  double x;
  double y;
  double z;
};

struct VectorSE2 {
  double x;
  double y;
  double theta;
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
    double x;
    double y;
    double theta;
  };

  struct Acceleration_Body {
    double x;
    double y;
    double theta;
  };

  struct Wrench_Body {
    double x;
    double y;
    double theta;
  };

  Tag tag;
  union {
    Velocity_Body velocity;
    Acceleration_Body acceleration;
    Wrench_Body wrench;
  };
};

struct KickCommand {
  enum class Tag {
    /// Kicker does not fire
    Off,
    /// Kicker fires immediately at the given speed, in m/s
    KickImmediate,
    /// Kicker fires if the ball is touching the front of the robot, in m/s
    KickOnTouch,
  };

  struct KickImmediate_Body {
    double speed;
  };

  struct KickOnTouch_Body {
    double speed;
  };

  Tag tag;
  union {
    KickImmediate_Body kick_immediate;
    KickOnTouch_Body kick_on_touch;
  };
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
    double speed;
  };

  struct Force_Body {
    double force;
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
