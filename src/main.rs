use macroquad::prelude::*;

const SCREEN_WIDTH: f32 = 1920.0;
const SCREEN_HEIGHT: f32 = 1080.0;

#[derive(Clone, Copy, Debug)]
struct Boid {
    position: Vec2,
    velocity: Vec2,
}

impl Boid {
    fn new(x: f32, y: f32) -> Self {
        let angle = rand::gen_range(0.0, 5.0 * std::f32::consts::PI);
        Self {
            position: vec2(x, y),
            velocity: vec2(angle.cos(), angle.sin()) * rand::gen_range(00.0, 10.0),
        }
    }

    fn update(&mut self, max_speed: f32) {
        self.velocity = self.velocity.clamp_length_max(max_speed);
        self.position += self.velocity;

        // Wrap around the screen
        if self.position.x > SCREEN_WIDTH {
            self.position.x = 0.0;
        } else if self.position.x < 0.0 {
            self.position.x = SCREEN_WIDTH;
        }
        if self.position.y > SCREEN_HEIGHT {
            self.position.y = 0.0;
        } else if self.position.y < 0.0 {
            self.position.y = SCREEN_HEIGHT;
        }
    }
}

fn rule1_cohesion(boid: &Boid, boids: &[Boid], interaction_radius: f32) -> Vec2 {
    let mut center_of_mass = Vec2::ZERO;
    let mut count = 0;

    for other in boids {
        if boid.position.distance(other.position) < interaction_radius {
            center_of_mass += other.position;
            count += 1;
        }
    }

    if count > 0 {
        center_of_mass /= count as f32;
        return (center_of_mass - boid.position).normalize_or_zero() * 0.2; // Adjust scaling
    }

    Vec2::ZERO
}

fn rule2_separation(boid: &Boid, boids: &[Boid], separation_radius: f32) -> Vec2 {
    let mut move_away = Vec2::ZERO;
    let mut count = 0;

    for other in boids {
        let distance = boid.position.distance(other.position);
        if distance > 0.0 && distance < separation_radius {
            move_away += (boid.position - other.position).normalize_or_zero();
            count += 1;
        }
    }

    if count > 0 {
        return move_away.normalize_or_zero() * 0.2; // Stronger separation force
    }

    Vec2::ZERO
}

fn rule3_alignment(boid: &Boid, boids: &[Boid], alignment_radius: f32) -> Vec2 {
    let mut avg_velocity = Vec2::ZERO;
    let mut count = 0;

    for other in boids {
        if boid.position.distance(other.position) < alignment_radius {
            avg_velocity += other.velocity;
            count += 1;
        }
    }

    if count > 0 {
        avg_velocity /= count as f32;
        return (avg_velocity - boid.velocity).normalize_or_zero() * 0.05; // Moderate scaling
    }

    Vec2::ZERO
}
#[macroquad::main("Boids with Egui")]
async fn main() {
    // Parameters for the simulation
    let mut max_speed = 6.0;
    let mut cohesion_radius = 100.0;
    let mut separation_radius = 30.0;
    let mut alignment_radius = 10.0;
    let mut num_boids = 100;
    let mut boid_size = 2.0;

    const FIXED_TIMESTEP: f32 = 1.0 / 60.0; // 60 updates per second
    const MAX_TIMESTEP_ACCUMULATION: f32 = 0.1;
    let mut accumulator = 0.0;

    // Create initial boids
    let mut boids: Vec<Boid> = (0..num_boids)
        .map(|_| {
            Boid::new(
                rand::gen_range(0.0, SCREEN_WIDTH),
                rand::gen_range(0.0, SCREEN_HEIGHT),
            )
        })
        .collect();

    loop {
        clear_background(BLACK);

        let frame_time = get_frame_time();
        accumulator += frame_time;

        if accumulator > MAX_TIMESTEP_ACCUMULATION {
            accumulator = MAX_TIMESTEP_ACCUMULATION;
        }

        while accumulator >= FIXED_TIMESTEP {
            let forces: Vec<Vec2> = boids
                .iter()
                .map(|boid| {
                    let cohesion = rule1_cohesion(boid, &boids, cohesion_radius);
                    let separation = rule2_separation(boid, &boids, separation_radius);
                    let alignment = rule3_alignment(boid, &boids, alignment_radius);
                    cohesion + separation + alignment
                })
                .collect();

            for (boid, force) in boids.iter_mut().zip(forces.iter()) {
                boid.velocity += *force;
                boid.update(max_speed);
            }

            accumulator -= FIXED_TIMESTEP;
        }

        // Draw the boids
        for (i, boid) in boids.iter().enumerate() {
            if i == 0 {
                draw_circle_lines(boid.position.x, boid.position.y, cohesion_radius, 1.0, RED);
                draw_circle_lines(
                    boid.position.x,
                    boid.position.y,
                    separation_radius,
                    1.0,
                    BLUE,
                );
                draw_circle_lines(
                    boid.position.x,
                    boid.position.y,
                    alignment_radius,
                    1.0,
                    GREEN,
                );
                draw_circle(boid.position.x, boid.position.y, boid_size, GOLD);
            } else {
                draw_circle(boid.position.x, boid.position.y, boid_size, WHITE);
            }
        }

        // Render egui
        new_egui_macroquad::ui(|egui_ctx| {
            egui::Window::new("Simulation Settings").show(egui_ctx, |ui| {
                ui.add(egui::Slider::new(&mut max_speed, 1.0..=10.0).text("Max Speed"));
                ui.add(
                    egui::Slider::new(&mut cohesion_radius, 0.0..=1000.0).text("Cohesion Radius"),
                );
                ui.add(
                    egui::Slider::new(&mut separation_radius, 0.0..=500.0)
                        .text("Separation Radius"),
                );
                ui.add(
                    egui::Slider::new(&mut alignment_radius, 0.0..=500.0).text("Alignment Radius"),
                );

                ui.add(egui::Slider::new(&mut boid_size, 1.0..=10.0).text("Boids size"));
                if ui.button("Reset Boids").clicked() {
                    boids = (0..num_boids)
                        .map(|_| {
                            Boid::new(
                                rand::gen_range(0.0, SCREEN_WIDTH),
                                rand::gen_range(0.0, SCREEN_HEIGHT),
                            )
                        })
                        .collect();
                }
                ui.add(egui::Slider::new(&mut num_boids, 10..=3_000).text("Number of Boids"));
            });
        });

        // Draw FPS
        draw_text(&format!("FPS: {}", get_fps()), 1800.0, 20.0, 20.0, GREEN);
        // Commit egui frame
        new_egui_macroquad::draw();

        match num_boids.cmp(&boids.len()) {
            std::cmp::Ordering::Less => {
                for _ in 0..boids.len() - num_boids {
                    boids.pop();
                }
            }
            std::cmp::Ordering::Greater => {
                for _ in 0..num_boids - boids.len() {
                    boids.push(Boid::new(
                        rand::gen_range(0.0, SCREEN_WIDTH),
                        rand::gen_range(0.0, SCREEN_HEIGHT),
                    ))
                }
            }
            _ => (),
        }

        next_frame().await;
    }
}
