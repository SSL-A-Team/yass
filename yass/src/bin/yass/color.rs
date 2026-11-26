use kiss3d::color::Color;

pub fn srgb_to_linear(c: Color) -> Color {
    fn channel(v: f32) -> f32 {
        // if v <= 0.04045 {
        //     v / 12.92
        // } else {
        //     ((v + 0.055) / 1.055).powf(2.4)
        // }
        v.powf(2.2)
    }
    Color::new(channel(c.r), channel(c.g), channel(c.b), c.a)
}
