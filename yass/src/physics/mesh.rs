use rapier3d::prelude::*;

pub fn generate_robot_mesh() -> Option<SharedShape> {
    // TODO: pull dimensions from config
    let radius = 0.09;
    let height = 0.180;
    let mouth_width = 0.08;

    let mut vertices = vec![];
    generate_robot_footprint_vertices(&mut vertices, radius, mouth_width, 0.0);
    generate_robot_footprint_vertices(&mut vertices, radius, mouth_width, height);

    SharedShape::convex_hull(&vertices)
}

fn generate_robot_footprint_vertices(vertices: &mut Vec<Vec3>, radius: f32, mouth_width: f32, z: f32) {
    let start = f32::asin((mouth_width/2.0) / radius);
    let end = f32::two_pi() - start;
    let num_points = 100;
    let step = (end - start) / (num_points as f32);
    for angle in (0..=num_points).map(|i| start + (i as f32) * step) {
        vertices.push(Vec3::new(radius * f32::cos(angle), radius * f32::cos(angle), z));
    }
}
