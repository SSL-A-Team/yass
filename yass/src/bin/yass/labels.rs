use std::sync::Arc;

use kiss3d::prelude::*;
use yass_corelib::{Robot, SimulationState};

pub fn draw_robot_id_labels(window: &mut Window, camera: &OrbitCamera3d, state: SimulationState) {
    let font = Font::default();
    for robot in &state.yellow_robots {
        draw_robot_id(window, camera, robot, &font);
    }
    for robot in &state.blue_robots {
        draw_robot_id(window, camera, robot, &font);
    }
}

fn draw_robot_id(window: &mut Window, camera: &OrbitCamera3d, robot: &Robot, font: &Arc<Font>) {
    if !robot.present {
        return;
    }
    let window_size = window.size().as_vec2();
    let mut screen_coords = camera.project(Vec3::new(robot.pose.x, robot.pose.y, 0.18), window_size);
    // kiss3d's `project` returns coordinates with a bottom-left origin (despite its docs), but
    // `draw_text` expects a top-left origin, so flip the vertical axis.
    screen_coords.y = window_size.y - screen_coords.y;
    let font_scale = 30.0;
    screen_coords.y -= font_scale / 2.0;
    screen_coords.x -= (font_scale / 4.0) * robot.id.to_string().len() as f32;
    window.draw_text(&robot.id.to_string(), screen_coords, font_scale, font, BLACK);
}
