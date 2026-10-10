#ifndef YASS_CORELIB_H
#define YASS_CORELIB_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef enum yass_TeamColor {
  yass_TeamColor_Blue,
  yass_TeamColor_Yellow,
} yass_TeamColor;

typedef enum yass_MotionCommandFrame {
  yass_MotionCommandFrame_Local,
  yass_MotionCommandFrame_Global,
} yass_MotionCommandFrame;

typedef struct yass_Vector3 {
  float x;
  float y;
  float z;
} yass_Vector3;

typedef struct yass_VectorSE2 {
  float x;
  float y;
  float theta;
} yass_VectorSE2;

typedef struct yass_Robot {
  uint8_t id;
  bool present;
  struct yass_VectorSE2 pose;
  struct yass_VectorSE2 velocity;
  struct yass_VectorSE2 acceleration;
  bool ball_sense;
} yass_Robot;

typedef struct yass_Ball {
  struct yass_Vector3 position;
  struct yass_Vector3 velocity;
  struct yass_Vector3 acceleration;
} yass_Ball;

typedef struct yass_SimulationState {
  struct yass_Robot blue_robots[16];
  struct yass_Robot yellow_robots[16];
  struct yass_Ball ball;
} yass_SimulationState;

typedef enum yass_MotionCommand_Tag {
  yass_MotionCommand_Off,
  yass_MotionCommand_Velocity,
  yass_MotionCommand_Acceleration,
  yass_MotionCommand_Wrench,
} yass_MotionCommand_Tag;

typedef struct yass_MotionCommand_yass_Velocity_Body {
  float x;
  float y;
  float theta;
} yass_MotionCommand_yass_Velocity_Body;

typedef struct yass_MotionCommand_yass_Acceleration_Body {
  float x;
  float y;
  float theta;
} yass_MotionCommand_yass_Acceleration_Body;

typedef struct yass_MotionCommand_yass_Wrench_Body {
  float x;
  float y;
  float theta;
} yass_MotionCommand_yass_Wrench_Body;

typedef struct yass_MotionCommand {
  yass_MotionCommand_Tag tag;
  union {
    yass_MotionCommand_yass_Velocity_Body velocity;
    yass_MotionCommand_yass_Acceleration_Body acceleration;
    yass_MotionCommand_yass_Wrench_Body wrench;
  };
} yass_MotionCommand;

typedef enum yass_KickCommand_Tag {
  /**
   * Kicker does not fire
   */
  yass_KickCommand_Off,
  /**
   * Kicker fires immediately at the given speed, in m/s
   */
  yass_KickCommand_KickImmediate,
  /**
   * Kicker fires if the ball is touching the front of the robot, in m/s
   */
  yass_KickCommand_KickOnTouch,
} yass_KickCommand_Tag;

typedef struct yass_KickCommand_yass_KickImmediate_Body {
  float speed;
} yass_KickCommand_yass_KickImmediate_Body;

typedef struct yass_KickCommand_yass_KickOnTouch_Body {
  float speed;
} yass_KickCommand_yass_KickOnTouch_Body;

typedef struct yass_KickCommand {
  yass_KickCommand_Tag tag;
  union {
    yass_KickCommand_yass_KickImmediate_Body kick_immediate;
    yass_KickCommand_yass_KickOnTouch_Body kick_on_touch;
  };
} yass_KickCommand;

typedef enum yass_DribblerCommand_Tag {
  /**
   * Ball will not stick to the dribbler
   */
  yass_DribblerCommand_Off,
  /**
   * Ball will glue to the front of the robot
   */
  yass_DribblerCommand_Sticky,
  /**
   * Ball will spin towards the robot, in rad/s
   */
  yass_DribblerCommand_Rotation,
  /**
   * Ball will be pulled towards the front of the robot with the given force, in N
   */
  yass_DribblerCommand_Force,
} yass_DribblerCommand_Tag;

typedef struct yass_DribblerCommand_yass_Rotation_Body {
  float speed;
} yass_DribblerCommand_yass_Rotation_Body;

typedef struct yass_DribblerCommand_yass_Force_Body {
  float force;
} yass_DribblerCommand_yass_Force_Body;

typedef struct yass_DribblerCommand {
  yass_DribblerCommand_Tag tag;
  union {
    yass_DribblerCommand_yass_Rotation_Body rotation;
    yass_DribblerCommand_yass_Force_Body force;
  };
} yass_DribblerCommand;

typedef struct yass_RobotCommand {
  uint8_t id;
  struct yass_MotionCommand motion_command;
  enum yass_MotionCommandFrame motion_command_frame;
  struct yass_KickCommand kick_command;
  struct yass_DribblerCommand dribbler_command;
} yass_RobotCommand;

#endif  /* YASS_CORELIB_H */
