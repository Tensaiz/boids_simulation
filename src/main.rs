use macroquad::prelude::*;
use rayon::prelude::*;

const SCREEN_WIDTH: f32 = 1920.0;
const SCREEN_HEIGHT: f32 = 1080.0;

#[derive(Clone, Debug)]
struct BoidData {
    positions: Vec<Vec2>,
    velocities: Vec<Vec2>,
}

impl BoidData {
    fn new(num_boids: usize) -> Self {
        Self {
            positions: Vec::with_capacity(num_boids),
            velocities: Vec::with_capacity(num_boids),
        }
    }
}

fn create_boid_data(num_boids: usize) -> BoidData {
    let mut data = BoidData::new(num_boids);
    for _ in 0..num_boids {
        let x = rand::gen_range(0.0, SCREEN_WIDTH);
        let y = rand::gen_range(0.0, SCREEN_HEIGHT);
        let angle = rand::gen_range(0.0, 5.0 * std::f32::consts::PI);

        data.positions.push(vec2(x, y));
        data.velocities
            .push(vec2(angle.cos(), angle.sin()) * rand::gen_range(0.0, 10.0));
    }
    data
}

fn rule1_cohesion(i: usize, data: &BoidData, interaction_radius: f32) -> Vec2 {
    let mut center_of_mass = Vec2::ZERO;
    let mut count = 0;

    let boid_pos = data.positions[i];

    for (j, &pos_j) in data.positions.iter().enumerate() {
        if j != i && boid_pos.distance(pos_j) < interaction_radius {
            center_of_mass += pos_j;
            count += 1;
        }
    }

    if count > 0 {
        center_of_mass /= count as f32;
        return (center_of_mass - boid_pos).normalize_or_zero() * 0.2;
    }

    Vec2::ZERO
}

fn rule2_separation(i: usize, data: &BoidData, separation_radius: f32) -> Vec2 {
    let mut move_away = Vec2::ZERO;
    let mut count = 0;

    let boid_pos = data.positions[i];

    for (j, &pos_j) in data.positions.iter().enumerate() {
        let distance = boid_pos.distance(pos_j);
        if j != i && distance > 0.0 && distance < separation_radius {
            move_away += (boid_pos - pos_j).normalize_or_zero();
            count += 1;
        }
    }

    if count > 0 {
        return move_away.normalize_or_zero() * 0.2;
    }

    Vec2::ZERO
}

fn rule3_alignment(i: usize, data: &BoidData, alignment_radius: f32) -> Vec2 {
    let mut avg_velocity = Vec2::ZERO;
    let mut count = 0;

    let boid_pos = data.positions[i];
    let boid_vel = data.velocities[i];

    for (j, &pos_j) in data.positions.iter().enumerate() {
        if j != i && boid_pos.distance(pos_j) < alignment_radius {
            avg_velocity += data.velocities[j];
            count += 1;
        }
    }

    if count > 0 {
        avg_velocity /= count as f32;
        return (avg_velocity - boid_vel).normalize_or_zero() * 0.05;
    }

    Vec2::ZERO
}

#[macroquad::main("Boids DoD")]
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
    let mut boid_data = create_boid_data(num_boids);

    loop {
        clear_background(BLACK);

        let frame_time = get_frame_time();
        accumulator += frame_time;

        if accumulator > MAX_TIMESTEP_ACCUMULATION {
            accumulator = MAX_TIMESTEP_ACCUMULATION;
        }

        let mut forces = vec![Vec2::ZERO; boid_data.positions.len()];
        while accumulator >= FIXED_TIMESTEP {
            forces.par_iter_mut().enumerate().for_each(|(i, force)| {
                let cohesion = rule1_cohesion(i, &boid_data, cohesion_radius);
                let separation = rule2_separation(i, &boid_data, separation_radius);
                let alignment = rule3_alignment(i, &boid_data, alignment_radius);
                *force = cohesion + separation + alignment;
            });

            boid_data
                .positions
                .par_iter_mut()
                .zip(boid_data.velocities.par_iter_mut())
                .zip(forces.par_iter())
                .for_each(|((pos, vel), &force)| {
                    *vel += force;
                    *vel = vel.clamp_length_max(max_speed);
                    *pos += *vel;

                    // Wrap around the screen
                    if pos.x > SCREEN_WIDTH {
                        pos.x = 0.0;
                    } else if pos.x < 0.0 {
                        pos.x = SCREEN_WIDTH;
                    }
                    if pos.y > SCREEN_HEIGHT {
                        pos.y = 0.0;
                    } else if pos.y < 0.0 {
                        pos.y = SCREEN_HEIGHT;
                    }
                });

            accumulator -= FIXED_TIMESTEP;
        }
        // Draw the boids
        for (i, (&pos, _)) in boid_data
            .positions
            .iter()
            .zip(&boid_data.velocities)
            .enumerate()
        {
            // Just an example: highlight first boid with circles
            if i == 0 {
                draw_circle_lines(pos.x, pos.y, cohesion_radius, 1.0, RED);
                draw_circle_lines(pos.x, pos.y, separation_radius, 1.0, BLUE);
                draw_circle_lines(pos.x, pos.y, alignment_radius, 1.0, GREEN);
                draw_circle(pos.x, pos.y, boid_size, GOLD);
            } else {
                draw_circle(pos.x, pos.y, boid_size, WHITE);
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
                    boid_data = create_boid_data(num_boids);
                }
                ui.add(egui::Slider::new(&mut num_boids, 10..=10_000).text("Number of Boids"));
            });
        });

        // Draw FPS
        draw_text(&format!("FPS: {}", get_fps()), 1800.0, 20.0, 20.0, GREEN);
        // Commit egui frame
        new_egui_macroquad::draw();

        match num_boids.cmp(&boid_data.velocities.len()) {
            std::cmp::Ordering::Less => {
                for _ in 0..boid_data.velocities.len() - num_boids {
                    boid_data.velocities.pop();
                    boid_data.positions.pop();
                }
            }
            std::cmp::Ordering::Greater => {
                for _ in 0..num_boids - boid_data.velocities.len() {
                    let x = rand::gen_range(0.0, SCREEN_WIDTH);
                    let y = rand::gen_range(0.0, SCREEN_HEIGHT);
                    let angle = rand::gen_range(0.0, 5.0 * std::f32::consts::PI);

                    boid_data.positions.push(vec2(x, y));
                    boid_data
                        .velocities
                        .push(vec2(angle.cos(), angle.sin()) * rand::gen_range(0.0, 10.0));
                }
            }
            _ => (),
        }

        next_frame().await;
    }
}
