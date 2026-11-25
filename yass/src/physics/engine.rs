use crate::{
    config::{BallConfig, Config, FieldConfig, RobotConfig},
    physics::mesh,
};
use rapier3d::{
    glamx::{EulerRot, Quat},
    prelude::*,
};
use yass_corelib::{
    Ball, DribblerCommand, KickCommand, MotionCommandFrame, Robot, TeamColor, Vector3, VectorSE2,
};

struct VelocityCache {
    ball_vel: Vec3,
    yellow_bot_vels: [Vec3; 16],
    blue_bot_vels: [Vec3; 16],
}

impl Default for VelocityCache {
    fn default() -> Self {
        Self {
            ball_vel: Default::default(),
            yellow_bot_vels: Default::default(),
            blue_bot_vels: Default::default(),
        }
    }
}

pub struct PhsyicsEngine {
    phys_world: PhysicsWorld,
    ball_handle: RigidBodyHandle,
    yellow_robot_handles: Vec<RigidBodyHandle>,
    blue_robot_handles: Vec<RigidBodyHandle>,
    yellow_robot_dribbler_fixed_joint_handles: Vec<Option<ImpulseJointHandle>>,
    blue_robot_dribbler_fixed_joint_handles: Vec<Option<ImpulseJointHandle>>,
    yellow_robot_is_kicking: Vec<bool>,
    blue_robot_is_kicking: Vec<bool>,
    velocity_cache: VelocityCache,
    ball_acceleration: Vec3,
    yellow_robot_accelerations: [Vec3; 16],
    blue_robot_accelerations: [Vec3; 16],
    ball_params: BallPhysicsParams,
}

const GRAVITY: f32 = 9.81;

/// Rapier parameters derived from the two-phase ball model in [`BallConfig`].
///
/// - Sliding: a sliding sphere decelerates at `mu * g`, so the ball-ground friction coefficient
///   is `sliding_deceleration / g`.
/// - Sliding-to-rolling transition: the speed at which a ball kicked with no spin starts rolling
///   depends on its moment of inertia `I = k * m * r^2` (without rolling resistance it is
///   `v0 / (1 + k)`). The ball's inertia is chosen to put the transition at
///   `sliding_to_rolling_ratio * v0`.
/// - Rolling: Rapier has no rolling resistance, so a constant torque opposing the ball's spin is
///   applied each step. While rolling without slipping, a torque `tau` decelerates the ball at
///   `tau * r / (I + m * r^2)`.
#[derive(Debug, Clone, Copy)]
struct BallPhysicsParams {
    radius: f32,
    inertia: f32,
    ground_friction: f32,
    rolling_resistance_torque: f32,
}

impl BallPhysicsParams {
    fn from_config(config: &BallConfig) -> Self {
        let BallConfig {
            radius,
            mass,
            sliding_deceleration,
            rolling_deceleration,
            sliding_to_rolling_ratio,
            ..
        } = *config;
        // The rolling resistance torque also acts while sliding, slowing spin-up. Accounting for
        // it, a ball kicked at v0 with no spin starts rolling at
        //   v0 * (1 - q * a_slide / (a_slide - a_roll)),  where q = k / (1 + k),
        // so solve that for the inertia factor k.
        let q = (1.0 - sliding_to_rolling_ratio) * (sliding_deceleration - rolling_deceleration)
            / sliding_deceleration;
        let inertia_factor = q / (1.0 - q);
        let inertia = inertia_factor * mass * radius * radius;
        Self {
            radius,
            inertia,
            ground_friction: sliding_deceleration / GRAVITY,
            rolling_resistance_torque: rolling_deceleration * (inertia + mass * radius * radius)
                / radius,
        }
    }
}

impl PhsyicsEngine {
    pub fn new(config: &Config) -> Self {
        let mut phys_world = PhysicsWorld::new();
        phys_world.gravity = Vec3::Z * -GRAVITY;
        phys_world.integration_parameters.length_unit = 0.1;
        let ball_params = BallPhysicsParams::from_config(&config.ball);
        Self::add_field(&mut phys_world, &config.field, &ball_params);
        let ball_handle = Self::add_ball(&mut phys_world, &config.ball, &ball_params);
        let yellow_start_x = ((config.yellow_team.robot_count as f32 - 1.0) / 2.0) * 0.18;
        let yellow_robot_handles = (0..config.yellow_team.robot_count)
            .map(|i| {
                Self::add_robot(
                    &mut phys_world,
                    &config.yellow_team.robot_props,
                    config.ball.radius,
                    Vec3::new(1.0, (i as f32 * 0.18) - yellow_start_x, 0.0),
                    std::f32::consts::PI,
                )
            })
            .collect();
        let blue_start_x = ((config.blue_team.robot_count as f32 - 1.0) / 2.0) * 0.18;
        let blue_robot_handles = (0..config.blue_team.robot_count)
            .map(|i| {
                Self::add_robot(
                    &mut phys_world,
                    &config.blue_team.robot_props,
                    config.ball.radius,
                    Vec3::new(-1.0, (i as f32 * 0.18) - blue_start_x, 0.0),
                    0.0,
                )
            })
            .collect();
        Self {
            phys_world,
            ball_handle,
            yellow_robot_handles,
            blue_robot_handles,
            yellow_robot_dribbler_fixed_joint_handles: vec![
                None;
                config.yellow_team.robot_count as usize
            ],
            blue_robot_dribbler_fixed_joint_handles: vec![
                None;
                config.blue_team.robot_count as usize
            ],
            yellow_robot_is_kicking: vec![false; config.yellow_team.robot_count as usize],
            blue_robot_is_kicking: vec![false; config.blue_team.robot_count as usize],
            velocity_cache: VelocityCache::default(),
            ball_acceleration: Vec3::default(),
            yellow_robot_accelerations: [Vec3::default(); 16],
            blue_robot_accelerations: [Vec3::default(); 16],
            ball_params,
        }
    }

    pub fn run_frame(&mut self, delta_t: std::time::Duration) {
        let delta_t_f32 = delta_t.as_secs_f32();
        self.phys_world.integration_parameters.dt = delta_t_f32;
        self.cache_velocities();
        self.apply_ball_rolling_resistance(delta_t_f32);
        self.phys_world.step();
        self.update_acceleration_tracking(delta_t_f32);
        // Rapier keeps user forces and torques until they are cleared explicitly. Commands are
        // reapplied every frame, so clear them after each step.
        for (_, body) in self.phys_world.bodies.iter_mut() {
            body.reset_forces(false);
            body.reset_torques(false);
        }
    }

    /// Applies a constant torque opposing the ball's horizontal spin while it is on the ground,
    /// emulating rolling resistance (see [`BallPhysicsParams`]).
    fn apply_ball_rolling_resistance(&mut self, dt: f32) {
        let params = self.ball_params;
        let ball_body = &mut self.phys_world.bodies[self.ball_handle];
        // Only resist rolling while touching the ground (small tolerance for contact penetration).
        if ball_body.translation().z > params.radius * 1.05 {
            return;
        }
        let angvel = ball_body.angvel();
        let spin = Vec3::new(angvel.x, angvel.y, 0.0);
        let spin_rate = spin.length();
        if spin_rate < 1e-6 {
            return;
        }
        // Never apply more torque than is needed to stop the spin within this step.
        let max_torque = params.inertia * spin_rate / dt;
        let torque_magnitude = f32::min(params.rolling_resistance_torque, max_torque);
        ball_body.add_torque(-spin / spin_rate * torque_magnitude, true);
    }

    fn add_field(world: &mut PhysicsWorld, config: &FieldConfig, ball_params: &BallPhysicsParams) {
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
        // The ball keeps Rapier's default friction so ball-robot and ball-wall contacts are
        // unchanged; the ground uses the Multiply rule so the ball-ground coefficient comes out to
        // exactly the calibrated value.
        world.insert_collider(
            ColliderBuilder::cuboid(world_length / 2.0, world_width / 2.0, 0.1 / 2.0)
                .translation(Vec3::Z * -0.05)
                .friction(ball_params.ground_friction / ColliderBuilder::default_friction())
                .friction_combine_rule(CoefficientCombineRule::Multiply),
            None,
        );
        // Long walls
        world.insert_collider(
            ColliderBuilder::cuboid(world_length / 2.0, wall_thickness / 2.0, wall_height / 2.0)
                .translation(Vec3::new(
                    0.0,
                    (world_width / 2.0) - (wall_thickness / 2.0),
                    wall_height / 2.0,
                )),
            None,
        );
        world.insert_collider(
            ColliderBuilder::cuboid(world_length / 2.0, wall_thickness / 2.0, wall_height / 2.0)
                .translation(Vec3::new(
                    0.0,
                    -((world_width / 2.0) - (wall_thickness / 2.0)),
                    wall_height / 2.0,
                )),
            None,
        );
        // Short walls
        world.insert_collider(
            ColliderBuilder::cuboid(wall_thickness / 2.0, world_width / 2.0, wall_height / 2.0)
                .translation(Vec3::new(
                    (world_length / 2.0) - (wall_thickness / 2.0),
                    0.0,
                    wall_height / 2.0,
                )),
            None,
        );
        world.insert_collider(
            ColliderBuilder::cuboid(wall_thickness / 2.0, world_width / 2.0, wall_height / 2.0)
                .translation(Vec3::new(
                    -((world_length / 2.0) - (wall_thickness / 2.0)),
                    0.0,
                    wall_height / 2.0,
                )),
            None,
        );
        // Positive Goal
        world.insert_collider(
            ColliderBuilder::cuboid(
                boundary_width / 2.0,
                wall_thickness / 2.0,
                wall_height / 2.0,
            )
            .translation(Vec3::new(
                (field_length / 2.0) + (boundary_width / 2.0),
                (goal_width / 2.0) + (wall_thickness / 2.0),
                wall_height / 2.0,
            )),
            None,
        );
        world.insert_collider(
            ColliderBuilder::cuboid(
                boundary_width / 2.0,
                wall_thickness / 2.0,
                wall_height / 2.0,
            )
            .translation(Vec3::new(
                (field_length / 2.0) + (boundary_width / 2.0),
                -((goal_width / 2.0) + (wall_thickness / 2.0)),
                wall_height / 2.0,
            )),
            None,
        );
        world.insert_collider(
            ColliderBuilder::cuboid(wall_thickness / 2.0, goal_width / 2.0, wall_height / 2.0)
                .translation(Vec3::new(
                    (field_length / 2.0) + goal_depth + (wall_thickness / 2.0),
                    0.0,
                    wall_height / 2.0,
                )),
            None,
        );
        // Negative Goal
        world.insert_collider(
            ColliderBuilder::cuboid(
                boundary_width / 2.0,
                wall_thickness / 2.0,
                wall_height / 2.0,
            )
            .translation(Vec3::new(
                -((field_length / 2.0) + (boundary_width / 2.0)),
                (goal_width / 2.0) + (wall_thickness / 2.0),
                wall_height / 2.0,
            )),
            None,
        );
        world.insert_collider(
            ColliderBuilder::cuboid(
                boundary_width / 2.0,
                wall_thickness / 2.0,
                wall_height / 2.0,
            )
            .translation(Vec3::new(
                -((field_length / 2.0) + (boundary_width / 2.0)),
                -((goal_width / 2.0) + (wall_thickness / 2.0)),
                wall_height / 2.0,
            )),
            None,
        );
        world.insert_collider(
            ColliderBuilder::cuboid(wall_thickness / 2.0, goal_width / 2.0, wall_height / 2.0)
                .translation(Vec3::new(
                    -((field_length / 2.0) + goal_depth + (wall_thickness / 2.0)),
                    0.0,
                    wall_height / 2.0,
                )),
            None,
        );
    }

    fn add_ball(
        world: &mut PhysicsWorld,
        config: &BallConfig,
        ball_params: &BallPhysicsParams,
    ) -> RigidBodyHandle {
        let (ball_handle, _) = world.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vec3::Z * config.radius)
                .ccd_enabled(true),
            ColliderBuilder::ball(config.radius)
                .mass_properties(MassProperties::new(
                    Vec3::ZERO,
                    config.mass,
                    Vec3::splat(ball_params.inertia),
                ))
                .restitution(config.restitution),
        );
        ball_handle
    }

    fn add_robot(
        world: &mut PhysicsWorld,
        robot_config: &RobotConfig,
        ball_radius: f32,
        position: Vec3,
        heading: f32,
    ) -> RigidBodyHandle {
        let (bot_handle, _) = world.insert(
            RigidBodyBuilder::dynamic()
                .rotation(Vector::new(0.0, 0.0, heading))
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
        let ball_sense_height = 1.1 * ball_radius * 2.0;
        world.insert_collider(
            ColliderBuilder::cuboid(0.001, mouth_width, ball_sense_height)
                .sensor(true)
                .translation(Vec3::new(
                    robot_radius * f32::cos(f32::asin((mouth_width / 2.0) / robot_radius)),
                    0.0,
                    ball_sense_height / 2.0,
                )),
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
            acceleration: Vector3 {
                x: self.ball_acceleration.x,
                y: self.ball_acceleration.y,
                z: self.ball_acceleration.z,
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

    fn get_bot_state_internal(&self, color: TeamColor, id: u8, handle: RigidBodyHandle) -> Robot {
        let bot_body = &self.phys_world.bodies[handle];
        let translation = &bot_body.translation();
        let rotation = &bot_body.rotation();
        let (theta, _, _) = rotation.to_euler(glamx::EulerRot::ZXY);
        let velocity = &bot_body.linvel();
        let angular_velocity = &bot_body.angvel();
        let acceleration = match color {
            TeamColor::Yellow => self.yellow_robot_accelerations[id as usize],
            TeamColor::Blue => self.blue_robot_accelerations[id as usize],
        };
        Robot {
            pose: VectorSE2 {
                x: translation.x,
                y: translation.y,
                theta,
            },
            id,
            present: true,
            velocity: VectorSE2 {
                x: velocity.x,
                y: velocity.y,
                theta: angular_velocity.z,
            },
            acceleration: VectorSE2 {
                x: acceleration.x,
                y: acceleration.y,
                theta: acceleration.z,
            },
            ball_sense: self.get_ball_sense(bot_body),
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

    pub fn get_bot_state(&self, color: TeamColor, id: u8) -> Robot {
        let Some(handle) = self.get_bot_handle(color, id) else {
            return Robot {
                id,
                present: false,
                ..Default::default()
            };
        };
        self.get_bot_state_internal(color, id, handle)
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
        let mass = bot_body.mass();
        let moment_of_inertia = bot_body
            .mass_properties()
            .local_mprops
            .reconstruct_inertia_matrix()
            .z_axis
            .z;
        let world_vel = match frame {
            MotionCommandFrame::Global => Vector::new(velocity.x, velocity.y, 0.0),
            MotionCommandFrame::Local => {
                bot_body.rotation() * Vector::new(velocity.x, velocity.y, 0.0)
            }
        };
        let impulse = mass * (world_vel - bot_body.linvel());
        bot_body.apply_impulse(impulse, true);
        let torque_impulse = Vec3::Z * (moment_of_inertia * (velocity.theta - bot_body.angvel().z));
        bot_body.apply_torque_impulse(torque_impulse, true);
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

    fn set_is_kicking(&mut self, color: TeamColor, id: u8, is_kicking: bool) {
        match color {
            TeamColor::Yellow => {
                if (id as usize) < self.yellow_robot_is_kicking.len() {
                    self.yellow_robot_is_kicking[id as usize] = is_kicking;
                }
            }
            TeamColor::Blue => {
                if (id as usize) < self.blue_robot_is_kicking.len() {
                    self.blue_robot_is_kicking[id as usize] = is_kicking;
                }
            }
        }
    }

    pub fn set_bot_kick(&mut self, color: TeamColor, id: u8, command: KickCommand) {
        if command.speed <= 0.0 {
            self.set_is_kicking(color, id, false);
            return;
        }
        let Some(handle) = self.get_bot_handle(color, id) else {
            self.set_is_kicking(color, id, false);
            return;
        };
        let bot_body = &self.phys_world.bodies[handle];
        if !self.get_ball_sense(bot_body) {
            self.set_is_kicking(color, id, false);
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
        self.set_is_kicking(color, id, true);
        self.remove_dribbler_fixed_joint(color, id);
    }

    fn bot_has_dribbler_fixed_joint(&mut self, color: TeamColor, id: u8) -> bool {
        let fixed_joint_handles = match color {
            TeamColor::Blue => &mut self.blue_robot_dribbler_fixed_joint_handles,
            TeamColor::Yellow => &mut self.yellow_robot_dribbler_fixed_joint_handles,
        };
        fixed_joint_handles[id as usize].is_some()
    }

    fn remove_dribbler_fixed_joint(&mut self, color: TeamColor, id: u8) {
        let fixed_joint_handles = match color {
            TeamColor::Blue => &mut self.blue_robot_dribbler_fixed_joint_handles,
            TeamColor::Yellow => &mut self.yellow_robot_dribbler_fixed_joint_handles,
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
        let bot_position = *bot_body.position();
        let bot_rotation = *bot_body.rotation();
        if !self.get_ball_sense(bot_body) {
            // self.remove_dribbler_fixed_joint(color, id);
            return;
        }
        let is_kicking = match color {
            TeamColor::Yellow => self.yellow_robot_is_kicking[id as usize],
            TeamColor::Blue => self.blue_robot_is_kicking[id as usize],
        };
        if let DribblerCommand::Sticky = command
            && !is_kicking
        {
            if !self.bot_has_dribbler_fixed_joint(color, id) {
                let ball_body = &mut self.phys_world.bodies[self.ball_handle];
                let rel_pos = bot_position.inv_mul(ball_body.position());
                let joint = FixedJointBuilder::new()
                    .local_frame1(rel_pos)
                    .local_frame2(Pose::identity())
                    .contacts_enabled(false);
                let joint_handle =
                    self.phys_world
                        .impulse_joints
                        .insert(handle, self.ball_handle, joint, true);
                let fixed_joint_handles = match color {
                    TeamColor::Blue => &mut self.blue_robot_dribbler_fixed_joint_handles,
                    TeamColor::Yellow => &mut self.yellow_robot_dribbler_fixed_joint_handles,
                };
                fixed_joint_handles[id as usize] = Some(joint_handle);
            }
        } else {
            self.remove_dribbler_fixed_joint(color, id);
        }
        let ball_body = &mut self.phys_world.bodies[self.ball_handle];
        match command {
            DribblerCommand::Off => (),
            DribblerCommand::Sticky => (), // handled above
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

    pub fn teleport_ball(
        &mut self,
        x: Option<f32>,
        y: Option<f32>,
        z: Option<f32>,
        vel_x: Option<f32>,
        vel_y: Option<f32>,
        vel_z: Option<f32>,
    ) {
        let ball_body = &mut self.phys_world.bodies[self.ball_handle];
        let mut pos = *ball_body.position();
        pos.translation.x = x.unwrap_or(pos.translation.x);
        pos.translation.y = y.unwrap_or(pos.translation.y);
        pos.translation.z = z.unwrap_or(pos.translation.z);
        ball_body.set_position(pos, true);
        let current_vel = ball_body.linvel();
        let mut target_vel = current_vel;
        target_vel.x = vel_x.unwrap_or(target_vel.x);
        target_vel.y = vel_y.unwrap_or(target_vel.y);
        target_vel.z = vel_z.unwrap_or(target_vel.z);
        let impulse = ball_body.mass() * (target_vel - current_vel);
        ball_body.apply_impulse(impulse, true);
        ball_body.set_angvel(Vec3::ZERO, true);
    }

    #[allow(clippy::too_many_arguments)]
    pub fn teleport_robot(
        &mut self,
        color: TeamColor,
        id: u8,
        x: Option<f32>,
        y: Option<f32>,
        theta: Option<f32>,
        vel_x: Option<f32>,
        vel_y: Option<f32>,
        vel_theta: Option<f32>,
    ) {
        let Some(handle) = self.get_bot_handle(color, id) else {
            return;
        };
        let bot_body = &mut self.phys_world.bodies[handle];
        let mut pos = *bot_body.position();
        pos.translation.x = x.unwrap_or(pos.translation.x);
        pos.translation.y = y.unwrap_or(pos.translation.y);
        if let Some(t) = theta {
            pos.rotation = Quat::from_euler(EulerRot::ZXY, t, 0.0, 0.0);
        }
        bot_body.set_position(pos, true);

        let current_vel = bot_body.linvel();
        let mut target_vel = current_vel;
        target_vel.x = vel_x.unwrap_or(target_vel.x);
        target_vel.y = vel_y.unwrap_or(target_vel.y);
        let impulse = bot_body.mass() * (target_vel - current_vel);
        bot_body.apply_impulse(impulse, true);

        let current_ang_vel = bot_body.angvel();
        let mut target_ang_vel = current_ang_vel;
        target_ang_vel.z = vel_theta.unwrap_or(target_ang_vel.z);
        let moment_of_inertia = bot_body
            .mass_properties()
            .local_mprops
            .reconstruct_inertia_matrix()
            .z_axis
            .z;
        let ang_impulse = moment_of_inertia * (target_ang_vel - current_ang_vel);
        bot_body.apply_torque_impulse(ang_impulse, true);
    }

    fn cache_velocities(&mut self) {
        let ball_body = &self.phys_world.bodies[self.ball_handle];
        self.velocity_cache.ball_vel = ball_body.linvel();
        self.velocity_cache.yellow_bot_vels =
            std::array::from_fn(|id| self.get_bot_velocity(TeamColor::Yellow, id as u8));
        self.velocity_cache.blue_bot_vels =
            std::array::from_fn(|id| self.get_bot_velocity(TeamColor::Blue, id as u8));
    }

    fn get_bot_velocity(&self, color: TeamColor, id: u8) -> Vec3 {
        let Some(handle) = self.get_bot_handle(color, id) else {
            return Vec3::default();
        };
        let bot_body = &self.phys_world.bodies[handle];
        let velocity = &bot_body.linvel();
        let angular_velocity = &bot_body.angvel();
        Vec3::new(velocity.x, velocity.y, angular_velocity.z)
    }

    fn update_acceleration_tracking(&mut self, delta_t: f32) {
        let ball_body = &self.phys_world.bodies[self.ball_handle];
        let new_ball_vel = ball_body.linvel();
        let new_yellow_bot_vels: [Vec3; 16] =
            std::array::from_fn(|id| self.get_bot_velocity(TeamColor::Yellow, id as u8));
        let new_blue_bot_vels: [Vec3; 16] =
            std::array::from_fn(|id| self.get_bot_velocity(TeamColor::Blue, id as u8));
        self.ball_acceleration = (new_ball_vel - self.velocity_cache.ball_vel) / delta_t;
        self.yellow_robot_accelerations = std::array::from_fn(|id| {
            (new_yellow_bot_vels[id] - self.velocity_cache.yellow_bot_vels[id]) / delta_t
        });
        self.blue_robot_accelerations = std::array::from_fn(|id| {
            (new_blue_bot_vels[id] - self.velocity_cache.blue_bot_vels[id]) / delta_t
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{ControlConfig, TeamConfig};

    fn test_config() -> Config {
        let team = TeamConfig {
            firmware: None,
            robot_props: RobotConfig::default(),
            robot_count: 1,
        };
        Config {
            field: FieldConfig::default(),
            ball: BallConfig::default(),
            blue_team: team.clone(),
            yellow_team: team,
            vision_plugins: vec![],
            control: ControlConfig::default(),
        }
    }

    #[test]
    fn robot_stops_at_wall_under_local_velocity() {
        let config = test_config();
        let mut engine = PhsyicsEngine::new(&config);
        let start = engine.get_bot_state(TeamColor::Blue, 0);
        // Drive the robot (local -Y, i.e. to its right) into the -Y wall for 6 seconds.
        for _ in 0..360 {
            engine.set_bot_velocity(
                TeamColor::Blue,
                0,
                VectorSE2 {
                    x: 0.0,
                    y: -1.0,
                    theta: 0.0,
                },
                MotionCommandFrame::Local,
            );
            engine.run_frame(std::time::Duration::from_secs_f32(1.0 / 60.0));
        }
        let end = engine.get_bot_state(TeamColor::Blue, 0);
        println!(
            "start=({}, {}, {}) end=({}, {}, {})",
            start.pose.x, start.pose.y, start.pose.theta, end.pose.x, end.pose.y, end.pose.theta
        );
        let wall_inner_face = -(config.field.width / 2.0 + config.field.boundary_width);
        let radius = config.blue_team.robot_props.radius;
        assert!(
            end.pose.y >= wall_inner_face + radius - 0.01,
            "robot passed through wall: y={}",
            end.pose.y
        );
        assert!(
            (end.pose.theta - start.pose.theta).abs() < 0.05,
            "heading drifted"
        );
    }

    struct KickSample {
        t: f32,
        speed: f32,
        position: Vec3,
    }

    /// Places the ball on the ground at `start`, gives it `velocity` with no spin, and records the
    /// ball's state every 1 ms step (the headless simulator's rate) for `duration` seconds.
    fn simulate_kick(
        config: &Config,
        start: Vec3,
        velocity: Vec3,
        duration: f32,
    ) -> Vec<KickSample> {
        let mut engine = PhsyicsEngine::new(config);
        engine.teleport_ball(
            Some(start.x),
            Some(start.y),
            Some(config.ball.radius),
            Some(velocity.x),
            Some(velocity.y),
            Some(0.0),
        );
        let dt = 0.001;
        (1..=(duration / dt) as usize)
            .map(|i| {
                engine.run_frame(std::time::Duration::from_secs_f32(dt));
                let ball = engine.get_ball_state();
                let v = Vec3::new(ball.velocity.x, ball.velocity.y, 0.0);
                KickSample {
                    t: i as f32 * dt,
                    speed: v.length(),
                    position: Vec3::new(ball.position.x, ball.position.y, 0.0),
                }
            })
            .collect()
    }

    fn speed_at(samples: &[KickSample], t: f32) -> f32 {
        samples
            .iter()
            .min_by(|a, b| (a.t - t).abs().total_cmp(&(b.t - t).abs()))
            .unwrap()
            .speed
    }

    fn assert_close(name: &str, actual: f32, expected: f32, rel_tol: f32) {
        assert!(
            (actual - expected).abs() <= rel_tol * expected.abs(),
            "{name}: expected {expected} (±{:.1}%), got {actual}",
            rel_tol * 100.0
        );
    }

    /// Checks a simulated kick against the two-phase ball model's closed-form predictions.
    fn assert_follows_two_phase_model(config: &Config, start: Vec3, velocity: Vec3) {
        let BallConfig {
            sliding_deceleration: a_slide,
            rolling_deceleration: a_roll,
            sliding_to_rolling_ratio: c_switch,
            ..
        } = config.ball;
        let v_shoot = velocity.length();
        let v_switch = c_switch * v_shoot;
        let t_switch = (v_shoot - v_switch) / a_slide;
        let t_stop = t_switch + v_switch / a_roll;
        let expected_distance = (v_shoot * v_shoot - v_switch * v_switch) / (2.0 * a_slide)
            + (v_switch * v_switch) / (2.0 * a_roll);

        let samples = simulate_kick(config, start, velocity, t_stop + 1.0);

        // Sliding phase deceleration.
        let (t0, t1) = (0.1 * t_switch, 0.9 * t_switch);
        let measured_slide = (speed_at(&samples, t0) - speed_at(&samples, t1)) / (t1 - t0);
        assert_close("sliding deceleration", measured_slide, a_slide, 0.02);

        // Transition: the first step where deceleration falls below the midpoint of the phases.
        let threshold = 0.5 * (a_slide + a_roll) * 0.001;
        let switch_sample = samples
            .windows(2)
            .find(|w| w[0].speed - w[1].speed < threshold)
            .map(|w| &w[1])
            .expect("ball never left the sliding phase");
        assert_close("switch speed", switch_sample.speed, v_switch, 0.02);
        assert_close("switch time", switch_sample.t, t_switch, 0.05);

        // Rolling phase deceleration.
        let (t0, t1) = (t_switch + 0.5, t_stop - 0.5);
        let measured_roll = (speed_at(&samples, t0) - speed_at(&samples, t1)) / (t1 - t0);
        assert_close("rolling deceleration", measured_roll, a_roll, 0.03);

        // The ball comes to rest at the predicted distance and stays there.
        let last = samples.last().unwrap();
        assert!(last.speed < 1e-3, "ball still moving: {} m/s", last.speed);
        let distance = (last.position - start).length();
        assert_close("stopping distance", distance, expected_distance, 0.03);
        let resting = &samples[samples.len() - 500];
        assert!(
            (last.position - resting.position).length() < 1e-4,
            "resting ball drifted"
        );
    }

    #[test]
    fn ball_follows_two_phase_model_with_default_config() {
        // Example kick from the ER-Force 2016 TDP, Fig. 6.
        let config = test_config();
        assert_follows_two_phase_model(
            &config,
            Vec3::new(-4.5, 2.0, 0.0),
            Vec3::new(3.3, 0.0, 0.0),
        );
    }

    #[test]
    fn ball_follows_two_phase_model_with_custom_config() {
        let mut config = test_config();
        config.ball.sliding_deceleration = 4.0;
        config.ball.rolling_deceleration = 0.5;
        config.ball.sliding_to_rolling_ratio = 5.0 / 7.0;
        assert_follows_two_phase_model(
            &config,
            Vec3::new(-3.0, -2.0, 0.0),
            Vec3::new(0.8, 0.6, 0.0) * 3.0,
        );
    }

    #[test]
    fn teleporting_ball_velocity_clears_spin() {
        let config = test_config();
        let mut engine = PhsyicsEngine::new(&config);
        engine.teleport_ball(Some(0.0), Some(2.0), None, Some(2.0), Some(0.0), Some(0.0));
        for _ in 0..1000 {
            engine.run_frame(std::time::Duration::from_millis(1));
        }
        assert!(
            engine.phys_world.bodies[engine.ball_handle]
                .angvel()
                .length()
                > 1.0
        );
        engine.teleport_ball(None, None, None, Some(0.0), Some(0.0), Some(0.0));
        assert_eq!(
            engine.phys_world.bodies[engine.ball_handle].angvel(),
            Vec3::ZERO
        );
    }

    #[test]
    fn wrench_applies_for_a_single_frame() {
        let config = test_config();
        let mut engine = PhsyicsEngine::new(&config);
        let dt = std::time::Duration::from_millis(1);
        engine.set_bot_wrench(
            TeamColor::Blue,
            0,
            VectorSE2 {
                x: 10.0,
                y: 0.0,
                theta: 0.0,
            },
            MotionCommandFrame::Global,
        );
        engine.run_frame(dt);
        let speed_after_wrench = engine.get_bot_state(TeamColor::Blue, 0).velocity.x;
        assert!(speed_after_wrench > 0.0);
        for _ in 0..100 {
            engine.run_frame(dt);
        }
        let speed_later = engine.get_bot_state(TeamColor::Blue, 0).velocity.x;
        assert!(
            speed_later <= speed_after_wrench + 1e-6,
            "force persisted: {speed_after_wrench} -> {speed_later}"
        );
    }
}
