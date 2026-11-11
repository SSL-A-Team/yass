use rapier3d::prelude::*;
use yass_corelib::{
    Ball, DribblerCommand, KickCommand, MotionCommandFrame, Robot, TeamColor, Vector3, VectorSE2,
};

use crate::{
    config::{Config, FieldConfig, RobotConfig},
    physics::{constants, mesh},
};

pub struct PhsyicsEngine {
    phys_world: PhysicsWorld,
    ball_handle: RigidBodyHandle,
    yellow_robot_handles: Vec<RigidBodyHandle>,
    blue_robot_handles: Vec<RigidBodyHandle>,
    yellow_robot_dribbler_fixed_joint_handles: Vec<Option<ImpulseJointHandle>>,
    blue_robot_dribbler_fixed_joint_handles: Vec<Option<ImpulseJointHandle>>,
}

impl PhsyicsEngine {
    pub fn new(config: &Config) -> Self {
        let mut phys_world = PhysicsWorld::new();
        phys_world.gravity = Vec3::Z * -9.81;
        Self::add_field(&mut phys_world, &config.field);
        let ball_handle = Self::add_ball(&mut phys_world);
        let yellow_robot_handles = (0..config.yellow_team.robot_count)
            .map(|i| {
                Self::add_robot(
                    &mut phys_world,
                    &config.yellow_team.robot_props,
                    Vec3::new(-1.0, i as f32 * 0.18, 0.0),
                )
            })
            .collect();
        let blue_robot_handles = (0..config.blue_team.robot_count)
            .map(|i| {
                Self::add_robot(
                    &mut phys_world,
                    &config.blue_team.robot_props,
                    Vec3::new(-1.0, i as f32 * 0.18, 0.0),
                )
            })
            .collect();
        Self {
            phys_world,
            ball_handle,
            yellow_robot_handles,
            blue_robot_handles,
            yellow_robot_dribbler_fixed_joint_handles: vec![None; config.yellow_team.robot_count as usize],
            blue_robot_dribbler_fixed_joint_handles: vec![None; config.blue_team.robot_count as usize],
        }
    }

    pub fn run_frame(&mut self) {
        self.phys_world.step();
    }

    fn add_field(world: &mut PhysicsWorld, config: &FieldConfig) {
        let field_length = config.length;
        let field_width = config.width;
        let boundary_width = config.boundary_width;
        let goal_width = config.goal_width;
        let goal_depth = config.goal_depth;

        let wall_height = 0.16;
        let wall_thickness = 0.02;
        let world_length = field_length + (2.0 * boundary_width) + (2.0 * wall_thickness);
        let world_width = field_width + (2.0 * boundary_width) + (2.0 * wall_thickness);
        // Ground
        world.insert_collider(
            ColliderBuilder::cuboid(world_length, world_width, 0.1).translation(Vec3::Z * -0.05),
            None,
        );
        // Long walls
        world.insert_collider(
            ColliderBuilder::cuboid(world_length, wall_thickness, wall_height).translation(
                Vec3::new(
                    0.0,
                    (world_width / 2.0) - (wall_thickness / 2.0),
                    wall_height / 2.0,
                ),
            ),
            None,
        );
        world.insert_collider(
            ColliderBuilder::cuboid(world_length, wall_thickness, wall_height).translation(
                Vec3::new(
                    0.0,
                    -((world_width / 2.0) - (wall_thickness / 2.0)),
                    wall_height / 2.0,
                ),
            ),
            None,
        );
        // Short walls
        world.insert_collider(
            ColliderBuilder::cuboid(wall_thickness, world_width, wall_height).translation(
                Vec3::new(
                    (world_length / 2.0) - (wall_thickness / 2.0),
                    0.0,
                    wall_height / 2.0,
                ),
            ),
            None,
        );
        world.insert_collider(
            ColliderBuilder::cuboid(wall_thickness, world_width, wall_height).translation(
                Vec3::new(
                    -((world_length / 2.0) - (wall_thickness / 2.0)),
                    0.0,
                    wall_height / 2.0,
                ),
            ),
            None,
        );
        // Positive Goal
        world.insert_collider(
            ColliderBuilder::cuboid(boundary_width, wall_thickness, wall_height).translation(
                Vec3::new(
                    (field_length / 2.0) + (boundary_width / 2.0),
                    (goal_width / 2.0) + (wall_thickness / 2.0),
                    wall_height / 2.0,
                ),
            ),
            None,
        );
        world.insert_collider(
            ColliderBuilder::cuboid(boundary_width, wall_thickness, wall_height).translation(
                Vec3::new(
                    (field_length / 2.0) + (boundary_width / 2.0),
                    -((goal_width / 2.0) + (wall_thickness / 2.0)),
                    wall_height / 2.0,
                ),
            ),
            None,
        );
        world.insert_collider(
            ColliderBuilder::cuboid(wall_thickness, goal_width, wall_height).translation(
                Vec3::new(
                    (field_length / 2.0) + goal_depth + (wall_thickness / 2.0),
                    0.0,
                    wall_height / 2.0,
                ),
            ),
            None,
        );
        // Negative Goal
        world.insert_collider(
            ColliderBuilder::cuboid(boundary_width, wall_thickness, wall_height).translation(
                Vec3::new(
                    -((field_length / 2.0) + (boundary_width / 2.0)),
                    (goal_width / 2.0) + (wall_thickness / 2.0),
                    wall_height / 2.0,
                ),
            ),
            None,
        );
        world.insert_collider(
            ColliderBuilder::cuboid(boundary_width, wall_thickness, wall_height).translation(
                Vec3::new(
                    -((field_length / 2.0) + (boundary_width / 2.0)),
                    -((goal_width / 2.0) + (wall_thickness / 2.0)),
                    wall_height / 2.0,
                ),
            ),
            None,
        );
        world.insert_collider(
            ColliderBuilder::cuboid(wall_thickness, goal_width, wall_height).translation(
                Vec3::new(
                    -((field_length / 2.0) + goal_depth + (wall_thickness / 2.0)),
                    0.0,
                    wall_height / 2.0,
                ),
            ),
            None,
        );
    }

    fn add_ball(world: &mut PhysicsWorld) -> RigidBodyHandle {
        let (ball_handle, _) = world.insert(
            RigidBodyBuilder::dynamic(),
            ColliderBuilder::ball(constants::BALL_RADIUS)
                .mass(constants::BALL_MASS)
                .restitution(constants::BALL_RESTITUTION),
        );
        ball_handle
    }

    fn add_robot(
        world: &mut PhysicsWorld,
        robot_config: &RobotConfig,
        position: Vec3,
    ) -> RigidBodyHandle {
        let (bot_handle, _) = world.insert(
            RigidBodyBuilder::dynamic()
                .translation(position)
                .enabled_translations(true, true, false)
                .enabled_rotations(false, false, true),
            ColliderBuilder::new(
                mesh::generate_robot_mesh(
                    robot_config.radius,
                    robot_config.height,
                    robot_config.mouth_width,
                )
                .expect("Failed to generate robot physics mesh"),
            )
            .mass(robot_config.mass),
        );
        let robot_radius = robot_config.radius;
        let mouth_width = robot_config.mouth_width;
        let ball_sense_height = 1.1 * constants::BALL_RADIUS * 2.0;
        world.insert_collider(
            ColliderBuilder::cuboid(0.001, mouth_width, ball_sense_height)
                .sensor(true)
                .translation(Vec3::new(
                    robot_radius * f32::cos(f32::asin((mouth_width / 2.0) / robot_radius)),
                    0.0,
                    ball_sense_height / 2.0,
                ))
                .translation(position),
            Some(bot_handle),
        );
        bot_handle
    }

    pub fn get_ball_state(&self) -> Ball {
        let ball_body = &self.phys_world.bodies[self.ball_handle];
        let translation = &ball_body.translation();
        let velocity = &ball_body.linvel();
        Ball {
            position: Vector3 {
                x: translation.x,
                y: translation.y,
                z: translation.z,
            },
            velocity: Vector3 {
                x: velocity.x,
                y: velocity.y,
                z: velocity.z,
            },
            // TODO: rapier doesn't let you query acceleration
            acceleration: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        }
    }

    fn get_bot_handle(&self, color: TeamColor, id: u8) -> Option<RigidBodyHandle> {
        let handles = match color {
            TeamColor::Blue => &self.blue_robot_handles,
            TeamColor::Yellow => &self.yellow_robot_handles,
        };
        if id as usize >= handles.len() {
            return None;
        }
        Some(handles[id as usize])
    }

    fn get_bot_state_internal(&self, id: u8, handle: RigidBodyHandle) -> Robot {
        let bot_body = &self.phys_world.bodies[handle];
        let translation = &bot_body.translation();
        let rotation = &bot_body.rotation();
        let (theta, _, _) = rotation.to_euler(glamx::EulerRot::YXZ);
        let velocity = &bot_body.linvel();
        let angular_velocity = &bot_body.angvel();
        Robot {
            pose: VectorSE2 {
                x: translation.x,
                y: translation.y,
                theta: theta,
            },
            id,
            present: true,
            velocity: VectorSE2 {
                x: velocity.x,
                y: velocity.y,
                theta: angular_velocity.z,
            },
            // TODO: rapier doesn't let you query acceleration
            acceleration: VectorSE2 {
                x: 0.0,
                y: 0.0,
                theta: 0.0,
            },
            ball_sense: self.get_ball_sense(&bot_body),
        }
    }

    fn get_ball_sense(&self, robot_body: &RigidBody) -> bool {
        let ball_body = &self.phys_world.bodies[self.ball_handle];
        let ball_collider = ball_body.colliders()[0];
        for &collider_handle in robot_body.colliders() {
            let collider = &self.phys_world.colliders[collider_handle];
            if collider.is_sensor() {
                return self
                    .phys_world
                    .narrow_phase
                    .intersection_pair(collider_handle, ball_collider)
                    .unwrap_or(false);
            }
        }
        false
    }

    pub fn get_bot_state(&self, color: yass_corelib::TeamColor, id: u8) -> Robot {
        let Some(handle) = self.get_bot_handle(color, id) else {
            let mut robot = Robot::default();
            robot.id = id;
            robot.present = false;
            return robot;
        };
        self.get_bot_state_internal(id, handle)
    }

    pub fn set_bot_velocity(
        &mut self,
        color: TeamColor,
        id: u8,
        velocity: VectorSE2,
        frame: MotionCommandFrame,
    ) {
        let Some(handle) = self.get_bot_handle(color, id) else {
            return;
        };
        let bot_body = &mut self.phys_world.bodies[handle];
        let world_vel = match frame {
            MotionCommandFrame::Global => Vector::new(velocity.x, velocity.y, 0.0),
            MotionCommandFrame::Local => {
                bot_body.rotation() * Vector::new(velocity.x, velocity.y, 0.0)
            }
        };
        bot_body.set_linvel(world_vel, true);
        bot_body.set_angvel(Vector::new(0.0, 0.0, velocity.theta), true);
    }

    pub fn set_bot_acceleration(
        &mut self,
        color: TeamColor,
        id: u8,
        acceleration: VectorSE2,
        frame: MotionCommandFrame,
    ) {
        let Some(handle) = self.get_bot_handle(color, id) else {
            return;
        };
        let bot_body = &self.phys_world.bodies[handle];
        let mass = bot_body.mass();
        let moment_of_inertia = bot_body
            .mass_properties()
            .local_mprops
            .reconstruct_inertia_matrix()
            .z_axis
            .z;
        let wrench = VectorSE2 {
            x: acceleration.x * mass,
            y: acceleration.y * mass,
            theta: acceleration.theta * moment_of_inertia,
        };
        self.set_bot_wrench(color, id, wrench, frame);
    }

    pub fn set_bot_wrench(
        &mut self,
        color: TeamColor,
        id: u8,
        wrench: VectorSE2,
        frame: MotionCommandFrame,
    ) {
        let Some(handle) = self.get_bot_handle(color, id) else {
            return;
        };
        let bot_body = &mut self.phys_world.bodies[handle];
        let world_force = match frame {
            MotionCommandFrame::Global => Vector::new(wrench.x, wrench.y, 0.0),
            MotionCommandFrame::Local => bot_body.rotation() * Vector::new(wrench.x, wrench.y, 0.0),
        };
        bot_body.add_force(world_force, true);
        bot_body.add_torque(Vector::new(0.0, 0.0, wrench.theta), true);
    }

    pub fn set_bot_kick(&mut self, color: TeamColor, id: u8, command: KickCommand) {
        if command.speed <= 0.0 {
            return;
        }
        let Some(handle) = self.get_bot_handle(color, id) else {
            return;
        };
        let bot_body = &self.phys_world.bodies[handle];
        if !self.get_ball_sense(bot_body) {
            return;
        }
        let chip_angle = f32::max(command.angle, 0.0);
        let local_dir = Vector::new(f32::cos(chip_angle), 0.0, f32::sin(chip_angle));
        let kick_dir = bot_body.rotation() * local_dir;
        // Kick speed is relative to the robot's velocity along the kick direction
        let bot_speed_along_kick = bot_body.linvel().dot(kick_dir);
        let desired_speed_along_kick = bot_speed_along_kick + command.speed;

        // Only change the ball's velocity along the kick direction, preserving perpendicular components
        let ball_body = &mut self.phys_world.bodies[self.ball_handle];
        let ball_speed_along_kick = ball_body.linvel().dot(kick_dir);
        let impulse =
            kick_dir * ((desired_speed_along_kick - ball_speed_along_kick) * ball_body.mass());
        ball_body.apply_impulse(impulse, true);
    }

    fn remove_dribbler_fixed_joint(&mut self, color: TeamColor, id: u8) {
        let fixed_joint_handles = match color {
            TeamColor::Blue => &mut self.blue_robot_dribbler_fixed_joint_handles,
            TeamColor::Yellow => &mut self.yellow_robot_dribbler_fixed_joint_handles
        };
        let Some(handle) = fixed_joint_handles[id as usize] else {
            return;
        };
        self.phys_world.impulse_joints.remove(handle, true);
        fixed_joint_handles[id as usize] = None;
    }

    pub fn set_bot_dribbler(&mut self, color: TeamColor, id: u8, command: DribblerCommand) {
        let Some(handle) = self.get_bot_handle(color, id) else {
            return;
        };
        let bot_body = &self.phys_world.bodies[handle];
        let bot_position = bot_body.position().clone();
        let bot_rotation = bot_body.rotation().clone();
        if !self.get_ball_sense(bot_body) {
            self.remove_dribbler_fixed_joint(color, id);
            return;
        }
        if let DribblerCommand::Sticky = command {
            let ball_body = &mut self.phys_world.bodies[self.ball_handle];
            let rel_pos = bot_position.inv_mul(ball_body.position());
            let joint = FixedJointBuilder::new()
                .local_frame1(rel_pos)
                .local_frame2(Pose::identity())
                .contacts_enabled(false);
            let joint_handle = self.phys_world
                .impulse_joints
                .insert(handle, self.ball_handle, joint, true);
            let fixed_joint_handles = match color {
                TeamColor::Blue => &mut self.blue_robot_dribbler_fixed_joint_handles,
                TeamColor::Yellow => &mut self.yellow_robot_dribbler_fixed_joint_handles
            };
            fixed_joint_handles[id as usize] = Some(joint_handle);
        } else {
            self.remove_dribbler_fixed_joint(color, id);
        }
        let ball_body = &mut self.phys_world.bodies[self.ball_handle];
        match command {
            DribblerCommand::Off => (),
            DribblerCommand::Sticky => (),  // handled above
            DribblerCommand::Rotation { speed } => {
                let mut angular_vel = Vector::new(0.0, -speed, 0.0);
                angular_vel = bot_rotation * angular_vel;
                ball_body.set_angvel(angular_vel, true);
            }
            DribblerCommand::Force { force } => {
                let mut f = Vector::new(0.0, -force, 0.0);
                f = bot_rotation * f;
                ball_body.add_force(f, true);
            }
        }
    }
}
