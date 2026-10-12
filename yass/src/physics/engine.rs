use rapier3d::prelude::*;
use yass_corelib::{Ball, Robot, Vector3, VectorSE2};

use crate::physics::mesh;

pub struct PhsyicsEngine {
    phys_world: PhysicsWorld,
    ball_handle: RigidBodyHandle,
    yellow_robot_handles: [RigidBodyHandle; 16],
    blue_robot_handles: [RigidBodyHandle; 16],
}

impl PhsyicsEngine {
    pub fn new() -> Self {
        let mut phys_world = PhysicsWorld::new();
        phys_world.gravity = Vector::new(0.0, 0.0, -9.81);
        Self::add_field(&mut phys_world);
        let ball_handle = Self::add_ball(&mut phys_world);
        let yellow_robot_handles = std::array::from_fn(|i| {
            Self::add_robot(
                &mut phys_world,
                Vec3 {
                    x: -1.0,
                    y: i as f32 * 0.18,
                    z: 0.0,
                },
            )
        });
        let blue_robot_handles = std::array::from_fn(|i| {
            Self::add_robot(
                &mut phys_world,
                Vec3 {
                    x: 1.0,
                    y: i as f32 * 0.18,
                    z: 0.0,
                },
            )
        });
        Self {
            phys_world,
            ball_handle,
            yellow_robot_handles,
            blue_robot_handles,
        }
    }

    pub fn run_frame(&mut self) {
        self.phys_world.step();
    }

    fn add_field(world: &mut PhysicsWorld) {
        // TODO use config sizes
        let field_length = 9.0;
        let field_width = 6.0;
        let boundary_width = 0.3;
        let goal_width = 1.0;
        let goal_depth = 0.16;

        let wall_height = 0.16;
        let wall_thickness = 0.02;
        let world_length = field_length + (2.0 * boundary_width) + (2.0 * wall_thickness);
        let world_width = field_width + (2.0 * boundary_width) + (2.0 * wall_thickness);
        // Ground
        world.insert_collider(
            ColliderBuilder::cuboid(world_length, world_width, 0.1)
                .translation(Vec3::new(0.0, 0.0, -0.05)),
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
                    wall_height / 2.0
                ),
            ),
            None
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
                    wall_height / 2.0
                ),
            ),
            None
        );
    }

    fn add_ball(world: &mut PhysicsWorld) -> RigidBodyHandle {
        let (ball_handle, _) = world.insert(
            RigidBodyBuilder::dynamic(),
            ColliderBuilder::ball(0.00213)
                .mass(0.04593)
                .restitution(0.65),
        );
        ball_handle
    }

    fn add_robot(world: &mut PhysicsWorld, position: Vec3) -> RigidBodyHandle {
        // TODO add flat front to robots
        let (bot_handle, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(position),
            ColliderBuilder::new(mesh::generate_robot_mesh().expect("Failed to generate robot physics mesh")),
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

    fn get_bot_state(&self, id: u8, handle: RigidBodyHandle) -> Robot {
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
            ball_sense: false,
        }
    }

    pub fn get_yellow_bot_state(&self, id: u8) -> Robot {
        self.get_bot_state(id, self.yellow_robot_handles[id as usize])
    }

    pub fn get_blue_bot_state(&self, id: u8) -> Robot {
        self.get_bot_state(id, self.blue_robot_handles[id as usize])
    }
}
