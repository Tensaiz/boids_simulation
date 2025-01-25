use macroquad::prelude::*;
use rayon::prelude::*;

const SCREEN_WIDTH: f32 = 1920.0;
const SCREEN_HEIGHT: f32 = 1080.0;

#[derive(Clone, Debug)]
struct Rectangle {
    x: f32,
    y: f32,
    w: f32,
    h: f32,
}

#[derive(Clone, Debug)]
struct QuadTree {
    boundary: Rectangle, // The current quadtree's position
    capacity: usize,     // Amount of boids to contain before splitting
    boid_indices: Vec<usize>,
    divided: bool,
    northeast: Option<Box<QuadTree>>,
    northwest: Option<Box<QuadTree>>,
    southeast: Option<Box<QuadTree>>,
    southwest: Option<Box<QuadTree>>,
}

impl QuadTree {
    fn new(boundary: Rectangle, capacity: usize) -> Self {
        Self {
            boundary,
            capacity,
            boid_indices: Vec::new(),
            divided: false,
            northeast: None,
            northwest: None,
            southeast: None,
            southwest: None,
        }
    }

    fn contains(&self, pos: Vec2) -> bool {
        pos.x >= self.boundary.x
            && pos.x < self.boundary.x + self.boundary.w
            && pos.y >= self.boundary.y
            && pos.y < self.boundary.y + self.boundary.h
    }

    fn subdivide(&mut self) {
        let x = self.boundary.x;
        let y = self.boundary.y;
        let w = self.boundary.w / 2.0;
        let h = self.boundary.h / 2.0;

        self.northeast = Some(Box::new(QuadTree::new(
            Rectangle { x: x + w, y, w, h },
            self.capacity,
        )));
        self.northwest = Some(Box::new(QuadTree::new(
            Rectangle { x, y, w, h },
            self.capacity,
        )));
        self.southeast = Some(Box::new(QuadTree::new(
            Rectangle {
                x: x + w,
                y: y + h,
                w,
                h,
            },
            self.capacity,
        )));
        self.southwest = Some(Box::new(QuadTree::new(
            Rectangle { x, y: y + h, w, h },
            self.capacity,
        )));

        self.divided = true;
    }

    fn insert(&mut self, index: usize, position: Vec2) -> bool {
        if !self.contains(position) {
            return false;
        }

        if self.boid_indices.len() < self.capacity {
            self.boid_indices.push(index);
            return true;
        }

        if !self.divided {
            self.subdivide();
        }

        if let Some(ne) = self.northeast.as_mut() {
            if ne.insert(index, position) {
                return true;
            }
        }
        if let Some(se) = self.southeast.as_mut() {
            if se.insert(index, position) {
                return true;
            }
        }
        if let Some(nw) = self.northwest.as_mut() {
            if nw.insert(index, position) {
                return true;
            }
        }
        if let Some(sw) = self.southwest.as_mut() {
            if sw.insert(index, position) {
                return true;
            }
        }

        false
    }

    fn intersects(&self, range: &Rectangle) -> bool {
        // standard AABB intersection
        let no_x_overlap =
            range.x > self.boundary.x + self.boundary.w || range.x + range.w < self.boundary.x;
        let no_y_overlap =
            range.y > self.boundary.y + self.boundary.h || range.y + range.h < self.boundary.y;
        !no_x_overlap && !no_y_overlap
    }

    fn query(&self, range: &Rectangle, neighbors: &mut Vec<usize>) {
        if !self.intersects(range) {
            return;
        }

        for &index in &self.boid_indices {
            neighbors.push(index);
        }

        if self.divided {
            if let Some(ne) = &self.northeast {
                ne.query(range, neighbors);
            }
            if let Some(se) = &self.southeast {
                se.query(range, neighbors);
            }
            if let Some(nw) = &self.northwest {
                nw.query(range, neighbors);
            }
            if let Some(sw) = &self.southwest {
                sw.query(range, neighbors);
            }
        }
    }

    fn render(&self) {
        draw_rectangle_lines(
            self.boundary.x,
            self.boundary.y,
            self.boundary.w,
            self.boundary.h,
            1.0,
            GREEN,
        );

        if self.divided {
            if let Some(ref ne) = self.northeast {
                ne.render();
            }
            if let Some(ref nw) = self.northwest {
                nw.render();
            }
            if let Some(ref se) = self.southeast {
                se.render();
            }
            if let Some(ref sw) = self.southwest {
                sw.render();
            }
        }
    }
}

fn build_quadtree(boids: &[Boid], capacity: usize) -> QuadTree {
    // The entire simulation area as a boundary
    let boundary = Rectangle {
        x: 0.0,
        y: 0.0,
        w: SCREEN_WIDTH,
        h: SCREEN_HEIGHT,
    };
    let mut qt = QuadTree::new(boundary, capacity);

    // Insert each boid by index
    for (i, &boid) in boids.iter().enumerate() {
        qt.insert(i, boid.position);
    }
    qt
}

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

    let mut render_quadtree = false;
    let mut follow_single = false;

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

        let quadtree = build_quadtree(&boids, 10);
        while accumulator >= FIXED_TIMESTEP {
            let forces: Vec<Vec2> = boids
                .par_iter()
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
            if i == 0 && follow_single {
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
                ui.add(egui::Checkbox::new(&mut follow_single, "Mark single boid"));
                ui.add(egui::Checkbox::new(
                    &mut render_quadtree,
                    "Render the quad tree",
                ));
                ui.add(egui::Slider::new(&mut num_boids, 10..=10_000).text("Number of Boids"));
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

        // Draw QuadTree
        if render_quadtree {
            quadtree.render();
        }
        next_frame().await;
    }
}
