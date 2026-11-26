use core::f32;

use kiss3d::{glamx::EulerRot, prelude::*};
use yass::config::{Config, RobotConfig};
use yass_corelib::{Robot, SimulationState, TeamColor};

use crate::color::srgb_to_linear;

const WALL_THICKNESS: f32 = 0.02;
const WALL_HEIGHT: f32 = 0.16;
const LINE_THICKNESS: f32 = 0.01;

pub struct Renderer {
    config: Config,
    ball_handle: SceneNode3d,
    yellow_robot_handles: [SceneNode3d; 16],
    blue_robot_handles: [SceneNode3d; 16],
}

impl Renderer {
    pub fn new(scene: &mut SceneNode3d, config: &Config) -> Self {
        scene
            .add_light(Light::point(30.0))
            .set_position(Vec3::new(0.0, 0.0, 10.0));
        init_ground_plane(scene, config);
        init_walls(scene, config);
        init_goals(scene, config);
        init_field_lines(scene, config);
        Self {
            config: config.clone(),
            ball_handle: init_ball(scene, config),
            yellow_robot_handles: init_robots(scene, config, TeamColor::Yellow),
            blue_robot_handles: init_robots(scene, config, TeamColor::Blue),
        }
    }

    pub fn update(&mut self, state: SimulationState) {
        self.ball_handle.set_position(Vec3::new(
            state.ball.position.x,
            state.ball.position.y,
            state.ball.position.z,
        ));
        self.update_robots(&state);
    }

    fn update_robots(&mut self, state: &SimulationState) {
        for i in 0..16 {
            let robot_handle = &mut self.yellow_robot_handles[i as usize];
            let robot_state = state.yellow_robots[i as usize];
            update_robot(
                robot_handle,
                robot_state,
                self.config.yellow_team.robot_props.height,
            );

            let robot_handle = &mut self.blue_robot_handles[i as usize];
            let robot_state = state.blue_robots[i as usize];
            update_robot(
                robot_handle,
                robot_state,
                self.config.blue_team.robot_props.height,
            );
        }
    }
}

fn init_ground_plane(scene: &mut SceneNode3d, config: &Config) {
    let ground_width =
        config.field.width + (2.0 * config.field.boundary_width) + (2.0 * WALL_THICKNESS);
    let ground_length =
        config.field.length + (2.0 * config.field.boundary_width) + (2.0 * WALL_THICKNESS);
    let mut ground = scene.add_quad(ground_length, ground_width, 1, 1);
    ground.set_color(srgb_to_linear(GREEN));
}

fn init_wall(
    scene: &mut SceneNode3d,
    x_dim: f32,
    y_dim: f32,
    x_pos: f32,
    y_pos: f32,
) -> SceneNode3d {
    scene
        .add_cube(x_dim, y_dim, WALL_HEIGHT)
        .set_position(Vec3::new(x_pos, y_pos, WALL_HEIGHT / 2.0))
}

fn init_walls(scene: &mut SceneNode3d, config: &Config) {
    init_wall(
        scene,
        config.field.length + (2.0 * config.field.boundary_width) + WALL_THICKNESS,
        WALL_THICKNESS,
        0.0,
        (config.field.width / 2.0) + config.field.boundary_width + (WALL_THICKNESS / 2.0),
    )
    .set_color(srgb_to_linear(BLACK));
    init_wall(
        scene,
        config.field.length + (2.0 * config.field.boundary_width) + WALL_THICKNESS,
        WALL_THICKNESS,
        0.0,
        -1.0 * ((config.field.width / 2.0) + config.field.boundary_width + (WALL_THICKNESS / 2.0)),
    )
    .set_color(srgb_to_linear(BLACK));
    init_wall(
        scene,
        WALL_THICKNESS,
        config.field.width + (2.0 * config.field.boundary_width) + WALL_THICKNESS,
        (config.field.length / 2.0) + config.field.boundary_width + (WALL_THICKNESS / 2.0),
        0.0,
    )
    .set_color(srgb_to_linear(BLACK));
    init_wall(
        scene,
        WALL_THICKNESS,
        config.field.width + (2.0 * config.field.boundary_width) + WALL_THICKNESS,
        -1.0 * ((config.field.length / 2.0) + config.field.boundary_width + (WALL_THICKNESS / 2.0)),
        0.0,
    )
    .set_color(srgb_to_linear(BLACK));
}

fn init_goal(scene: &mut SceneNode3d, config: &Config, side: f32) {
    init_wall(
        scene,
        WALL_THICKNESS,
        config.field.goal_width + (2.0 * WALL_THICKNESS),
        side * ((config.field.length / 2.0) + config.field.goal_depth + (WALL_THICKNESS / 2.0)),
        0.0,
    )
    .set_color(srgb_to_linear(WHITE));
    init_wall(
        scene,
        config.field.goal_depth + WALL_THICKNESS,
        WALL_THICKNESS,
        side * ((config.field.length / 2.0) + ((config.field.goal_depth + WALL_THICKNESS) / 2.0)),
        (config.field.goal_width / 2.0) + (WALL_THICKNESS / 2.0),
    )
    .set_color(srgb_to_linear(WHITE));
    init_wall(
        scene,
        config.field.goal_depth + WALL_THICKNESS,
        WALL_THICKNESS,
        side * ((config.field.length / 2.0) + ((config.field.goal_depth + WALL_THICKNESS) / 2.0)),
        -1.0 * ((config.field.goal_width / 2.0) + (WALL_THICKNESS / 2.0)),
    )
    .set_color(srgb_to_linear(WHITE));
}

fn init_goals(scene: &mut SceneNode3d, config: &Config) {
    init_goal(scene, config, 1.0);
    init_goal(scene, config, -1.0);
}

fn init_field_lines(scene: &mut SceneNode3d, config: &Config) {
    let half_length = config.field.length / 2.0;
    let half_width = config.field.width / 2.0;
    init_field_line(scene, -half_length, -half_width, half_length, -half_width);
    init_field_line(scene, -half_length, half_width, half_length, half_width);
    init_field_line(scene, -half_length, -half_width, -half_length, half_width);
    init_field_line(scene, half_length, -half_width, half_length, half_width);
    init_field_line(scene, 0.0, -half_width, 0.0, half_width);
    init_field_line(scene, -half_length, 0.0, half_length, 0.0);
    init_field_line(
        scene,
        half_length,
        -config.field.penalty_area_width / 2.0,
        half_length - config.field.penalty_area_depth,
        -config.field.penalty_area_width / 2.0,
    );
    init_field_line(
        scene,
        half_length,
        config.field.penalty_area_width / 2.0,
        half_length - config.field.penalty_area_depth,
        config.field.penalty_area_width / 2.0,
    );
    init_field_line(
        scene,
        half_length - config.field.penalty_area_depth,
        -config.field.penalty_area_width / 2.0,
        half_length - config.field.penalty_area_depth,
        config.field.penalty_area_width / 2.0,
    );
    init_field_line(
        scene,
        -half_length,
        -config.field.penalty_area_width / 2.0,
        -(half_length - config.field.penalty_area_depth),
        -config.field.penalty_area_width / 2.0,
    );
    init_field_line(
        scene,
        -half_length,
        config.field.penalty_area_width / 2.0,
        -(half_length - config.field.penalty_area_depth),
        config.field.penalty_area_width / 2.0,
    );
    init_field_line(
        scene,
        -(half_length - config.field.penalty_area_depth),
        -config.field.penalty_area_width / 2.0,
        -(half_length - config.field.penalty_area_depth),
        config.field.penalty_area_width / 2.0,
    );
    init_field_line_circle(scene, 0.0, 0.0, config.field.center_circle_radius);
}

fn init_field_line(scene: &mut SceneNode3d, x1: f32, y1: f32, x2: f32, y2: f32) {
    let width = (x2 - x1).hypot(y2 - y1) + LINE_THICKNESS;
    let height = LINE_THICKNESS;
    let cx = x1.midpoint(x2);
    let cy = y1.midpoint(y2);
    let yaw = (y2 - y1).atan2(x2 - x1);
    scene
        .add_quad(width, height, 1, 1)
        .set_color(srgb_to_linear(WHITE))
        .set_position(Vec3::new(cx, cy, 0.001))
        .set_rotation(Quat::from_axis_angle(Vec3::Z, yaw));
}

fn init_field_line_circle(scene: &mut SceneNode3d, cx: f32, cy: f32, r: f32) {
    const SEGMENTS: u32 = 100;
    let mut vertices = vec![];
    let mut indices = vec![];
    let inner_radius = r - (LINE_THICKNESS / 2.0);
    let outer_radius = r + (LINE_THICKNESS / 2.0);

    for i in 0..SEGMENTS {
        let theta = (i as f32 / SEGMENTS as f32) * f32::consts::PI * 2.0;
        let (sin, cos) = theta.sin_cos();
        vertices.push(Vec3::new(cos * inner_radius, sin * inner_radius, 0.0));
        vertices.push(Vec3::new(cos * outer_radius, sin * outer_radius, 0.0));

        let current_inner = i * 2;
        let current_outer = current_inner + 1;
        let next_inner = ((i + 1) % SEGMENTS) * 2;
        let next_outer = next_inner + 1;

        indices.push([current_inner, current_outer, next_outer]);
        indices.push([current_inner, next_outer, next_inner]);
    }

    scene
        .add_trimesh(vertices, indices, Vec3::ONE, true)
        .set_color(srgb_to_linear(WHITE))
        .set_position(Vec3::new(cx, cy, 0.001));
}

fn init_ball(scene: &mut SceneNode3d, config: &Config) -> SceneNode3d {
    scene
        .add_sphere(config.ball.radius)
        .set_color(srgb_to_linear(ORANGE))
}

fn init_robot(scene: &mut SceneNode3d, config: &Config, color: TeamColor) -> SceneNode3d {
    let props = match color {
        TeamColor::Yellow => &config.yellow_team.robot_props,
        TeamColor::Blue => &config.blue_team.robot_props,
    };
    let viz_color = match color {
        TeamColor::Yellow => YELLOW,
        TeamColor::Blue => BLUE,
    };
    let mut vertices = vec![];
    let mut indices = vec![];
    generate_robot_mesh(&mut vertices, &mut indices, props);
    scene
        .add_trimesh(vertices, indices, Vec3::ONE, true)
        .set_color(srgb_to_linear(viz_color))
        
}

fn generate_robot_mesh(
    vertices: &mut Vec<Vec3>,
    indices: &mut Vec<[u32; 3]>,
    config: &RobotConfig,
) {
    const SUBDIVISIONS: u32 = 100;
    vertices.push(Vec3::new(0.0, -config.height / 2.0, 0.0));
    let start = f32::asin((config.mouth_width / 2.0) / config.radius);
    let end = (2.0 * f32::consts::PI) - start;
    let step = (end - start) / (SUBDIVISIONS as f32);
    for angle in (0..=SUBDIVISIONS).map(|i| start + (i as f32) * step) {
        vertices.push(Vec3::new(
            config.radius * f32::cos(angle),
            -config.height / 2.0,
            config.radius * f32::sin(angle),
        ));
    }
    for ind in 1..=(SUBDIVISIONS - 1) {
        indices.push([0, ind, (ind + 1)]);
    }
    indices.push([0, SUBDIVISIONS, 1]);

    vertices.push(Vec3::new(0.0, config.height / 2.0, 0.0));
    let start = f32::asin((config.mouth_width / 2.0) / config.radius);
    let end = (2.0 * f32::consts::PI) - start;
    let step = (end - start) / (SUBDIVISIONS as f32);
    for angle in (0..=SUBDIVISIONS).map(|i| start + (i as f32) * step) {
        vertices.push(Vec3::new(
            config.radius * f32::cos(angle),
            config.height / 2.0,
            config.radius * f32::sin(angle),
        ));
    }
    for ind in (1..=(SUBDIVISIONS)).rev() {
        indices.push([
            (SUBDIVISIONS + 2),
            ind + (SUBDIVISIONS + 2),
            (ind - 1) + (SUBDIVISIONS + 2),
        ]);
    }
    indices.push([
        SUBDIVISIONS + 3,
        (2 * (SUBDIVISIONS)) + 2,
        (SUBDIVISIONS + 2),
    ]);

    for ind in 1..=SUBDIVISIONS {
        indices.push([ind + SUBDIVISIONS + 2, ind + 1, ind]);
        indices.push([ind + SUBDIVISIONS + 2, ind + 1 + SUBDIVISIONS + 2, ind + 1]);
    }
    indices.push([(2 * SUBDIVISIONS) + 2, 1, SUBDIVISIONS]);
    indices.push([(2 * SUBDIVISIONS) + 2, SUBDIVISIONS + 3, 1]);
}

fn init_robots(scene: &mut SceneNode3d, config: &Config, color: TeamColor) -> [SceneNode3d; 16] {
    std::array::from_fn(|_| init_robot(scene, config, color))
}

fn update_robot(handle: &mut SceneNode3d, state: Robot, height: f32) {
    handle.set_position(Vec3::new(state.pose.x, state.pose.y, height / 2.0));
    handle.set_rotation(Quat::from_euler(
        EulerRot::XYZ,
        (90 as f32).to_radians(),
        state.pose.theta,
        0.0,
    ));
    handle.set_visible(state.present);
}
