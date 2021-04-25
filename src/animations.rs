use micromath::F32Ext;
use smart_leds::RGB8;

pub fn tixy(t: f32, _i: usize, _x: usize, _y: usize) -> RGB8 {
    render(t.sin())
}

fn render(mut val: f32) -> RGB8 {
    if val >= 1.0 {
        val = 1.0
    } else if val <= -1.0 {
        val = -1.0;
    }
    if val < 0.0 {
        RGB8::new((val * val * 100.0) as u8, 0, 0)
    } else {
        RGB8::new(0, (val * val * 100.0) as u8, 0)
    }
}
