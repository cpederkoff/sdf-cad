use three_d::*;

/// Camera control that orbits around a target point and zooms towards the cursor.
pub struct CameraControl {
    pub target: Vec3,
    pub min_distance: f32,
    pub max_distance: f32,
}

impl CameraControl {
    pub fn new(target: Vec3, min_distance: f32, max_distance: f32) -> Self {
        Self {
            target,
            min_distance,
            max_distance,
        }
    }

    pub fn handle_events(&mut self, camera: &mut Camera, events: &mut [Event]) -> bool {
        let mut change = false;
        for event in events.iter_mut() {
            match event {
                Event::MouseMotion {
                    delta,
                    button,
                    handled,
                    ..
                } => {
                    if !*handled && *button == Some(MouseButton::Left) {
                        let speed = 0.01;
                        camera.rotate_around_with_fixed_up(
                            self.target,
                            speed * delta.0,
                            speed * delta.1,
                        );
                        *handled = true;
                        change = true;
                    }
                }
                Event::MouseWheel {
                    delta,
                    position,
                    handled,
                    ..
                } => {
                    if !*handled {
                        let distance = self.target.distance(camera.position());
                        let speed = 0.01 * distance + 0.001;

                        // Cast a ray from the cursor position into the scene
                        let ray_dir = camera.view_direction_at_pixel(*position);
                        let cursor_point = camera.position() + ray_dir * distance;

                        camera.zoom_towards(
                            cursor_point,
                            speed * delta.1,
                            self.min_distance,
                            self.max_distance,
                        );
                        // Update orbit target to match the camera's new target
                        self.target = camera.target();

                        *handled = true;
                        change = true;
                    }
                }
                Event::PinchGesture {
                    delta,
                    position,
                    handled,
                    ..
                } => {
                    if !*handled {
                        let distance = self.target.distance(camera.position());
                        let speed = distance + 0.1;

                        let ray_dir = camera.view_direction_at_pixel(*position);
                        let cursor_point = camera.position() + ray_dir * distance;

                        camera.zoom_towards(
                            cursor_point,
                            speed * *delta,
                            self.min_distance,
                            self.max_distance,
                        );
                        self.target = camera.target();

                        *handled = true;
                        change = true;
                    }
                }
                _ => {}
            }
        }
        change
    }
}
