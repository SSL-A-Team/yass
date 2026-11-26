use rapier3d::prelude::*;

pub fn generate_robot_mesh(radius: f32, height: f32, mouth_width: f32) -> Option<SharedShape> {
    let mut vertices = vec![];
    generate_robot_footprint_vertices(&mut vertices, radius, mouth_width, 0.0);
    generate_robot_footprint_vertices(&mut vertices, radius, mouth_width, height);

    SharedShape::convex_hull(&vertices)
}

fn generate_robot_footprint_vertices(
    vertices: &mut Vec<Vec3>,
    radius: f32,
    mouth_width: f32,
    z: f32,
) {
    let start = f32::asin((mouth_width / 2.0) / radius);
    let end = (2.0 * std::f32::consts::PI) - start;
    let num_points = 100;
    let step = (end - start) / (num_points as f32);
    for angle in (0..=num_points).map(|i| start + (i as f32) * step) {
        vertices.push(Vec3::new(
            radius * f32::cos(angle),
            radius * f32::sin(angle),
            z,
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn robot_mesh_has_full_footprint() {
        let radius = 0.09;
        let shape = generate_robot_mesh(radius, 0.15, 0.06).expect("mesh");
        let aabb = shape.compute_local_aabb();
        let extents = aabb.extents();
        println!("aabb mins={:?} maxs={:?}", aabb.mins, aabb.maxs);
        // Back of the robot and both sides should reach the full radius.
        assert!((aabb.mins.x + radius).abs() < 1e-3);
        assert!((aabb.mins.y + radius).abs() < 1e-3);
        assert!((aabb.maxs.y - radius).abs() < 1e-3);
        assert!((extents.z - 0.15).abs() < 1e-3);
        // Volume should be close to a full cylinder (minus the small mouth cutout).
        let volume = shape.mass_properties(1.0).mass();
        let cylinder = std::f32::consts::PI * radius * radius * 0.15;
        assert!(
            volume > 0.9 * cylinder,
            "volume {volume} vs cylinder {cylinder}"
        );
    }
}
