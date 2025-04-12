use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotPoints};
use reqwest::Client;
use serde::Deserialize;
use std::fs;
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use std::{collections::VecDeque, vec};

#[derive(Debug, Deserialize, Clone)]
pub struct Servo {
    pub id: usize,
    pub position: f32,
    #[serde(default)]
    pub history: Vec<ServoHistoryPoint>,
}

#[derive(Debug, Deserialize, Clone)]
pub struct ServoHistoryPoint {
    pub time: f32,
    pub target_angle: f32,
    pub actual_angle: f32,
}

#[derive(Debug, Deserialize)]
pub struct ApiResponse {
    pub servos: Vec<Servo>,
}

// Define a target pattern step
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct PatternStep {
    pub target_angle: f32,
    pub duration: f32, // Duration in seconds
}

// Define a looped pattern for a servo
#[derive(Debug, Clone, serde::Deserialize)]
pub struct LoopedPattern {
    pub steps: Vec<PatternStep>,
    #[serde(default)]
    pub total_duration: f32, // Total duration of the complete pattern
    #[serde(default)]
    pub enabled: bool,       // Whether the pattern is currently enabled
}

impl serde::Serialize for LoopedPattern {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;
        
        let mut s = serializer.serialize_struct("LoopedPattern", 3)?;
        s.serialize_field("steps", &self.steps)?;
        s.serialize_field("enabled", &self.enabled)?;
        // We don't need to serialize total_duration as it's calculated from steps
        s.end()
    }
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub struct TrackConfig {
    pub servo_id: usize,
    pub color: [u8; 3],
    #[serde(default = "default_history_duration")]
    pub history_duration: f64,
    #[serde(default = "default_max_points")]
    pub max_points: usize,
    pub pattern: LoopedPattern,
}

fn default_history_duration() -> f64 {
    10.0
}

fn default_max_points() -> usize {
    1000
}

impl LoopedPattern {
    pub fn new(steps: Vec<PatternStep>) -> Self {
        let total_duration = steps.iter().map(|step| step.duration).sum();
        Self {
            steps,
            total_duration,
            enabled: false,
        }
    }

    // Get the target angle at a specific time in the loop
    pub fn get_target_at_time(&self, time_in_loop: f32) -> f32 {
        let mut elapsed = 0.0;

        for step in &self.steps {
            if time_in_loop < elapsed + step.duration {
                return step.target_angle;
            }
            elapsed += step.duration;
        }

        // Fallback to the last step if there's any timing issue
        self.steps.last().map_or(90.0, |step| step.target_angle)
    }

    // Generate future pattern points for visualization
    pub fn generate_future_points(&self, start_time: f64, duration: f64) -> Vec<[f64; 2]> {
        if !self.enabled || self.steps.is_empty() {
            return Vec::new();
        }

        let mut points = Vec::new();
        let time_step = self.total_duration as f64 / 20.0; // 20 points per pattern for smooth lines

        let mut current_time = start_time;
        let max_time = start_time + duration;

        while current_time < max_time {
            let loop_time = (current_time - start_time).rem_euclid(self.total_duration as f64);
            let target = self.get_target_at_time(loop_time as f32);
            points.push([current_time, target as f64]);
            current_time += time_step;
        }

        // Add one more point at the end to complete the line
        let loop_time = (max_time - start_time).rem_euclid(self.total_duration as f64);
        let target = self.get_target_at_time(loop_time as f32);
        points.push([max_time, target as f64]);

        points
    }

    // Generate historical pattern points for trailing visualization
    pub fn generate_history_points(
        &self,
        end_time: f64,
        duration: f64,
        app_start_time: f64,
    ) -> Vec<[f64; 2]> {
        if !self.enabled || self.steps.is_empty() {
            return Vec::new();
        }

        let mut points = Vec::new();
        let time_step = self.total_duration as f64 / 20.0; // 20 points per pattern for smooth lines

        let start_time = end_time - duration;
        let mut current_time = start_time;

        while current_time <= end_time {
            let elapsed = current_time - app_start_time;
            let loop_time = elapsed.rem_euclid(self.total_duration as f64);
            let target = self.get_target_at_time(loop_time as f32);
            points.push([current_time, target as f64]);
            current_time += time_step;
        }

        points
    }
}

// Define the data structure for tracking a servo
pub struct ServoTrack {
    pub servo_id: usize,
    pub target_positions: VecDeque<(f64, f64)>, // (timestamp, value)
    pub actual_positions: VecDeque<(f64, f64)>, // (timestamp, value)
    pub current_target: f32,
    pub current_actual: f32,
    pub last_target: f32,
    pub max_points: usize,
    pub history_duration: f64, // Duration to show in history (seconds)
    pub color: egui::Color32,  // Color for this servo's plots
    pub looped_pattern: LoopedPattern, // Looped target pattern for this servo
    pub did_send: bool,        // Flag to indicate if a command was sent
}

impl ServoTrack {
    pub fn new(servo_id: usize, color: egui::Color32, pattern: LoopedPattern) -> Self {
        ServoTrack {
            servo_id,
            target_positions: VecDeque::new(),
            actual_positions: VecDeque::new(),
            current_target: 90.0,   // Default starting position
            last_target: 90.0,      // Default starting position
            current_actual: 90.0,   // Default starting position
            max_points: 1000,       // Maximum number of points to store
            history_duration: 10.0, // Show 10 seconds of history
            color,
            looped_pattern: pattern,
            did_send: false,
        }
    }
    
    pub fn from_config(config: &TrackConfig) -> Self {
        // Calculate the total duration for the pattern
        let mut pattern = config.pattern.clone();
        pattern.total_duration = pattern.steps.iter().map(|step| step.duration).sum();
        
        // Create the RGB color from the config
        let color = egui::Color32::from_rgb(config.color[0], config.color[1], config.color[2]);
        
        ServoTrack {
            servo_id: config.servo_id,
            target_positions: VecDeque::new(),
            actual_positions: VecDeque::new(),
            current_target: 90.0,
            last_target: 90.0,
            current_actual: 90.0,
            max_points: config.max_points,
            history_duration: config.history_duration,
            color,
            looped_pattern: pattern,
            did_send: false,
        }
    }

    // Add a new data point
    pub fn add_data_point(&mut self, timestamp: f64, target: f32, actual: f32) {
        self.current_target = target;
        self.current_actual = actual;

        // Add to history
        self.target_positions.push_back((timestamp, target as f64));
        self.actual_positions.push_back((timestamp, actual as f64));

        // Trim old data points beyond history duration
        let cutoff_time = timestamp - self.history_duration;
        while let Some((time, _)) = self.target_positions.front() {
            if *time < cutoff_time {
                self.target_positions.pop_front();
            } else {
                break;
            }
        }

        while let Some((time, _)) = self.actual_positions.front() {
            if *time < cutoff_time {
                self.actual_positions.pop_front();
            } else {
                break;
            }
        }

        // Keep the history at a reasonable size
        while self.target_positions.len() > self.max_points {
            self.target_positions.pop_front();
        }

        while self.actual_positions.len() > self.max_points {
            self.actual_positions.pop_front();
        }
    }

    // Process history from API
    pub fn process_history(&mut self, history: &[ServoHistoryPoint], current_time: f64) {
        // Clear existing history
        self.target_positions.clear();
        self.actual_positions.clear();

        // Process history in time order
        for point in history {
            let timestamp = current_time - (point.time as f64);
            self.add_data_point(timestamp, point.target_angle, point.actual_angle);
        }
    }

    // Get plot points for target positions
    pub fn get_target_plot_points(&self) -> Vec<[f64; 2]> {
        self.target_positions
            .iter()
            .map(|&(time, value)| [time, value])
            .collect()
    }

    // Get plot points for actual positions
    pub fn get_actual_plot_points(&self) -> Vec<[f64; 2]> {
        self.actual_positions
            .iter()
            .map(|&(time, value)| [time, value])
            .collect()
    }

    // Calculate the next target based on the looped pattern (if enabled)
    pub fn calculate_looped_target(&self, current_time: f64, start_time: f64) -> f32 {
        let elapsed = current_time - start_time;
        let loop_time = elapsed % self.looped_pattern.total_duration as f64;
        self.looped_pattern.get_target_at_time(loop_time as f32)
    }
}

// API client for fetching servo data
pub struct ApiClient {
    client: Client,
    api_url: String,
}

impl ApiClient {
    pub fn new(api_url: String) -> Self {
        ApiClient {
            client: Client::new(),
            api_url,
        }
    }

    pub async fn fetch_servo_data(&self) -> Result<Vec<Servo>, reqwest::Error> {
        let response = self
            .client
            .get(&self.api_url)
            .send()
            .await?
            .json::<ApiResponse>()
            .await?;

        Ok(response.servos)
    }

    pub async fn send_servo_command(
        &self,
        servo_id: usize,
        position: f32,
    ) -> Result<(), reqwest::Error> {
        let url = format!("{}/{}/position", self.api_url, servo_id);
        let command = serde_json::json!({
            "position": position,
            // "time": 600,
            "time": 400,
        });
        println!("Sending command to Servo {}: {:?}", servo_id, command);

        self.client.post(&url).json(&command).send().await?;
        Ok(())
    }
}

// Shared data between threads
pub struct SharedData {
    pub servo_data: Option<Vec<Servo>>,
    pub error_message: Option<String>,
}

// Main application state
// Function to load all track configurations from a directory
pub fn load_track_configs<P: AsRef<Path>>(tracks_dir: P) -> Result<Vec<TrackConfig>, Box<dyn std::error::Error>> {
    let mut configs = Vec::new();
    
    // Check if the directory exists
    let dir_path = tracks_dir.as_ref();
    if !dir_path.exists() || !dir_path.is_dir() {
        return Err(format!("Track directory not found: {}", dir_path.display()).into());
    }
    
    // Read all .toml files in the directory
    for entry in fs::read_dir(dir_path)? {
        let entry = entry?;
        let path = entry.path();
        
        if path.extension().map_or(false, |ext| ext == "toml") {
            // Read and parse the TOML file
            let content = fs::read_to_string(&path)?;
            let config: TrackConfig = toml::from_str(&content)?;
            configs.push(config);
        }
    }
    
    Ok(configs)
}

// Function to create a TrackConfig from a ServoTrack
pub fn track_to_config(track: &ServoTrack) -> TrackConfig {
    // Extract RGB components from Color32
    let r = track.color.r();
    let g = track.color.g();
    let b = track.color.b();
    
    TrackConfig {
        servo_id: track.servo_id,
        color: [r, g, b],
        history_duration: track.history_duration,
        max_points: track.max_points,
        pattern: track.looped_pattern.clone(),
    }
}

// Function to save a track configuration to a TOML file
pub fn save_track_config<P: AsRef<Path>>(
    tracks_dir: P, 
    track: &ServoTrack,
) -> Result<String, Box<dyn std::error::Error>> {
    // Create the directory if it doesn't exist
    let dir_path = tracks_dir.as_ref();
    if !dir_path.exists() {
        fs::create_dir_all(dir_path)?;
    }
    
    // Convert the track to a config
    let config = track_to_config(track);
    
    // Convert the config to TOML
    let toml_content = toml::to_string_pretty(&config)?;
    
    // Create a filename based on the servo ID
    let filename = format!("servo_{}.toml", track.servo_id);
    let file_path = dir_path.join(filename);
    
    // Write the TOML to the file
    fs::write(&file_path, toml_content)?;
    
    Ok(file_path.to_string_lossy().to_string())
}

pub struct ServoApp {
    pub tracks: Vec<ServoTrack>,
    pub client: Arc<ApiClient>,
    pub last_update_time: f64,
    pub update_interval: f64, // seconds
    pub start_time: f64,
    pub history_duration: f64, // Common history duration for all servos
    pub error_message: Option<String>,
    pub data_fetcher_running: Arc<Mutex<bool>>,
    pub shared_data: Arc<Mutex<SharedData>>,
    pub show_all_servos: bool,      // Flag to show all servos at once
    pub show_looped_patterns: bool, // Flag to show looped patterns visualization
    pub use_looped_targets: bool,   // Flag to use looped targets instead of API targets
    pub track_configs_path: Option<String>, // Path to track configurations
}

impl ServoApp {
    pub fn new(cc: &eframe::CreationContext<'_>, tracks_dir: &str, api_url: &str) -> Self {
        // Set up the API client
        let client = Arc::new(ApiClient::new(api_url.to_string()));

        // Define default colors for the tracks (only used if no TOML configs found)
        let default_colors = [
            egui::Color32::from_rgb(255, 0, 0),   // Red
            egui::Color32::from_rgb(0, 150, 255), // Blue
            egui::Color32::from_rgb(0, 200, 0),   // Green
        ];

        // Attempt to load tracks from TOML configuration files
        // let tracks_dir = "tracks";
        let mut tracks = Vec::new();
        
        match load_track_configs(tracks_dir) {
            Ok(configs) if !configs.is_empty() => {
                // Create tracks from configurations
                println!("Loaded {} track configurations from {}", configs.len(), tracks_dir);
                for config in &configs {
                    tracks.push(ServoTrack::from_config(config));
                }
            },
            Ok(_) => {
                // No config files found, create default tracks
                println!("No track configurations found in {}, using defaults", tracks_dir);
                let duration = 0.8;
                
                // Create a default track
                tracks.push(ServoTrack::new(
                    10,
                    default_colors[0],
                    LoopedPattern::new(vec![
                        PatternStep {
                            target_angle: 180.0,
                            duration: 2.0,
                        },
                        PatternStep {
                            target_angle: 220.0,
                            duration: 2.0,
                        },
                    ]),
                ));
            },
            Err(e) => {
                // Error loading configs, create default tracks
                println!("Error loading track configurations: {}", e);
                let duration = 0.8;
                
                // Create a default track
                tracks.push(ServoTrack::new(
                    10,
                    default_colors[0],
                    LoopedPattern::new(vec![
                        PatternStep {
                            target_angle: 180.0,
                            duration: 2.0,
                        },
                        PatternStep {
                            target_angle: 220.0,
                            duration: 2.0,
                        },
                    ]),
                ));
            }
        };

        let start_time = cc.egui_ctx.input(|i| i.time);
        let data_fetcher_running = Arc::new(Mutex::new(true));

        // Create shared data
        let shared_data = Arc::new(Mutex::new(SharedData {
            servo_data: None,
            error_message: None,
        }));

        // Clone references for the background task
        let ctx = cc.egui_ctx.clone();
        let client_clone = client.clone();
        let data_fetcher_running_clone = data_fetcher_running.clone();
        let shared_data_clone = shared_data.clone();

        // Spawn background task to fetch data
        std::thread::spawn(move || {
            let rt = tokio::runtime::Runtime::new().unwrap();
            rt.block_on(async {
                while *data_fetcher_running_clone.lock().unwrap() {
                    match client_clone.fetch_servo_data().await {
                        Ok(servo_data) => {
                            // Update shared data
                            {
                                let mut data = shared_data_clone.lock().unwrap();
                                data.servo_data = Some(servo_data);
                                data.error_message = None;
                            }
                            ctx.request_repaint();
                        }
                        Err(e) => {
                            let error_msg = format!("API Error: {}", e);
                            {
                                let mut data = shared_data_clone.lock().unwrap();
                                data.error_message = Some(error_msg);
                            }
                            ctx.request_repaint();
                        }
                    }
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
            });
        });

        ServoApp {
            tracks,
            client,
            last_update_time: start_time,
            update_interval: 0.1, // Update every 100ms
            start_time,
            history_duration: 10.0, // Default 10 seconds of history
            error_message: None,
            data_fetcher_running,
            shared_data,
            show_all_servos: true,      // Default to showing all servos
            show_looped_patterns: true, // Default to showing looped patterns
            use_looped_targets: false,  // Default to using looped targets instead of API targets
            track_configs_path: Some(tracks_dir.to_string()),
        }
    }

    pub fn update_from_api_data(&mut self) {
        // Lock shared data and process it
        let mut shared_data = self.shared_data.lock().unwrap();

        // Check if we have new servo data
        if let Some(ref servo_data) = shared_data.servo_data {
            let current_time = self.last_update_time;

            // Map servo IDs to their positions in the API response
            let servo_map: std::collections::HashMap<usize, &Servo> = servo_data
                .iter()
                .enumerate()
                .map(|(_, servo)| (servo.id, servo))
                .collect();

            // Update each track with its corresponding servo data
            for track in &mut self.tracks {
                if let Some(data) = servo_map.get(&track.servo_id) {
                    let target = track.calculate_looped_target(current_time, self.start_time);
                    let current = data.position;

                    if target != track.last_target {
                        println!(
                            "Servo {}: Target: {:.1}°, Last Target: {:.1}°",
                            track.servo_id, target, track.last_target
                        );

                        let rt = tokio::runtime::Runtime::new().unwrap();
                        rt.block_on(async {
                            let r = self.client.send_servo_command(track.servo_id, target).await;
                            if let Err(e) = r {
                                println!("Error sending command: {}", e);
                            } else {
                                println!(
                                    "Sent command to Servo {}: Target: {:.1}°",
                                    track.servo_id, target
                                );
                            }
                        })
                    }

                    let actual = current;

                    // Simple simulation of servo movement (would be replaced by actual data)
                    let new_actual = actual;
                    track.current_actual = new_actual;

                    // Add the latest data point
                    track.add_data_point(current_time, target, new_actual);

                    // Process history if available and not using looped targets
                    if !self.use_looped_targets && !data.history.is_empty() {
                        track.process_history(&data.history, current_time);
                    }

                    // Update the track's history duration from the app's global setting
                    track.history_duration = self.history_duration;

                    // update last_target
                    track.last_target = target;
                }
            }
        }

        // Check for errors
        if let Some(ref error_msg) = shared_data.error_message {
            self.error_message = Some(error_msg.clone());
            // Clear the error in shared data once we've processed it
            shared_data.error_message = None;
        }
    }

    pub fn draw_servo_plot(&self, ui: &mut egui::Ui, track: &ServoTrack, current_time: f64) {
        // Individual plot for a single servo
        let plot = Plot::new(format!("servo_plot_{}", track.servo_id))
            .legend(Legend::default().position(egui_plot::Corner::LeftTop))
            .height(200.0)
            .x_axis_label("Time (seconds)")
            .y_axis_label("Angle (degrees)")
            .allow_zoom(true)
            .allow_drag(true)
            .view_aspect(2.0);

        let min_x = current_time - track.history_duration;
        let max_x = current_time;
        let min_y = 0.0;
        let max_y = 270.0;

        plot.show(ui, |plot_ui| {
            // Manually set the bounds
            plot_ui.set_plot_bounds(egui_plot::PlotBounds::from_min_max(
                [min_x, min_y],
                [max_x, max_y],
            ));

            // Plot the past pattern (trailing)
            let past_pattern_color = track.color.linear_multiply(0.6); // Dimmer
            let past_pattern_points = track.looped_pattern.generate_history_points(
                current_time,
                track.history_duration,
                self.start_time,
            );

            if !past_pattern_points.is_empty() {
                let past_pattern_points: PlotPoints = past_pattern_points.into();
                plot_ui.line(
                    Line::new(past_pattern_points)
                        .name("Pattern History")
                        .width(2.0)
                        .color(past_pattern_color)
                        .style(egui_plot::LineStyle::dashed_loose()),
                );
            }

            // Plot the actual positions
            let actual_points = track.get_actual_plot_points();
            if !actual_points.is_empty() {
                let actual_points: PlotPoints = actual_points.into();
                plot_ui.line(
                    Line::new(actual_points)
                        .name("Actual")
                        .width(3.0)
                        .color(track.color),
                );
            }

            // Plot the future pattern
            let pattern_color = track.color.linear_multiply(0.7).gamma_multiply(1.2); // More visible
            let pattern_points = track
                .looped_pattern
                .generate_future_points(current_time, track.history_duration);

            if !pattern_points.is_empty() {
                let pattern_points: PlotPoints = pattern_points.into();
                plot_ui.line(
                    Line::new(pattern_points)
                        .name("Pattern Future")
                        .width(2.0)
                        .color(pattern_color)
                        .style(egui_plot::LineStyle::dashed_dense()),
                );
            }

            // Current position indicator (for actual)
            plot_ui.points(
                egui_plot::Points::new(PlotPoints::from_iter(vec![[
                    current_time,
                    track.current_actual as f64,
                ]]))
                .name("Current Position")
                .radius(6.0)
                .color(track.color)
                .shape(egui_plot::MarkerShape::Circle),
            );
        });
    }

    pub fn draw_combined_plot(&self, ui: &mut egui::Ui, current_time: f64) {
        // Combined plot for all servos
        let plot = Plot::new("all_servos_plot")
            .legend(Legend::default().position(egui_plot::Corner::LeftTop))
            .height(400.0)
            .x_axis_label("Time (seconds)")
            .y_axis_label("Angle (degrees)")
            .allow_zoom(true)
            .allow_drag(true);

        let min_x = current_time - self.history_duration;
        let max_x = current_time;
        let min_y = 0.0;
        let max_y = 270.0;

        plot.show(ui, |plot_ui| {
            // Manually set the bounds
            plot_ui.set_plot_bounds(egui_plot::PlotBounds::from_min_max(
                [min_x, min_y],
                [max_x, max_y],
            ));

            // Plot all servos on the same chart
            for track in &self.tracks {
                // Plot the past pattern (trailing)
                let past_pattern_color = track.color.linear_multiply(0.6); // Dimmer
                let past_pattern_points = track.looped_pattern.generate_history_points(
                    current_time,
                    self.history_duration,
                    self.start_time,
                );

                if !past_pattern_points.is_empty() {
                    let past_pattern_points: PlotPoints = past_pattern_points.into();
                    plot_ui.line(
                        Line::new(past_pattern_points)
                            .name(format!("Servo {} (Pattern History)", track.servo_id))
                            .width(2.0)
                            .color(past_pattern_color)
                            .style(egui_plot::LineStyle::dashed_loose()),
                    );
                }

                // Plot the actual positions
                let actual_points = track.get_actual_plot_points();
                if !actual_points.is_empty() {
                    let actual_points: PlotPoints = actual_points.into();
                    plot_ui.line(
                        Line::new(actual_points)
                            .name(format!("Servo {} (Actual)", track.servo_id))
                            .width(3.0)
                            .color(track.color),
                    );
                }

                // Plot the future pattern
                let pattern_color = track.color.linear_multiply(0.7).gamma_multiply(1.2); // More visible
                let pattern_points = track
                    .looped_pattern
                    .generate_future_points(current_time, self.history_duration);

                if !pattern_points.is_empty() {
                    let pattern_points: PlotPoints = pattern_points.into();
                    plot_ui.line(
                        Line::new(pattern_points)
                            .name(format!("Servo {} (Pattern Future)", track.servo_id))
                            .width(2.0)
                            .color(pattern_color)
                            .style(egui_plot::LineStyle::dashed_dense()),
                    );
                }

                // Current position indicators (for actual)
                plot_ui.points(
                    egui_plot::Points::new(PlotPoints::from_iter(vec![[
                        current_time,
                        track.current_actual as f64,
                    ]]))
                    .name(format!("Servo {} (Current)", track.servo_id))
                    .radius(6.0)
                    .color(track.color)
                    .shape(egui_plot::MarkerShape::Circle),
                );
            }
        });
    }

    pub fn reload_track_configs(&mut self) -> Result<(), String> {
        if let Some(tracks_dir) = &self.track_configs_path {
            match load_track_configs(tracks_dir) {
                Ok(configs) if !configs.is_empty() => {
                    // Clear existing tracks and create new ones from configurations
                    self.tracks.clear();
                    
                    for config in &configs {
                        self.tracks.push(ServoTrack::from_config(config));
                    }
                    
                    println!("Reloaded {} track configurations", configs.len());
                    Ok(())
                },
                Ok(_) => {
                    Err("No track configurations found".to_string())
                },
                Err(e) => {
                    Err(format!("Error loading track configurations: {}", e))
                }
            }
        } else {
            Err("No tracks directory configured".to_string())
        }
    }
    
    pub fn save_all_track_configs(&self) -> Result<Vec<String>, String> {
        if let Some(tracks_dir) = &self.track_configs_path {
            let mut saved_files = Vec::new();
            
            for track in &self.tracks {
                match save_track_config(tracks_dir, track) {
                    Ok(file_path) => {
                        saved_files.push(file_path);
                    },
                    Err(e) => {
                        return Err(format!("Failed to save track {}: {}", track.servo_id, e));
                    }
                }
            }
            
            if saved_files.is_empty() {
                return Err("No tracks to save".to_string());
            }
            
            Ok(saved_files)
        } else {
            Err("No tracks directory configured".to_string())
        }
    }
    
    pub fn draw_pattern_editor(&mut self, ui: &mut egui::Ui, track_index: usize) {
        let track = &mut self.tracks[track_index];

        ui.horizontal(|ui| {
            ui.checkbox(&mut track.looped_pattern.enabled, "Enable Pattern");

            if track.looped_pattern.enabled {
                ui.label(format!(
                    "Total cycle: {:.1} seconds",
                    track.looped_pattern.total_duration
                ));
            }
        });

        if track.looped_pattern.enabled {
            // Show pattern steps
            for (i, step) in track.looped_pattern.steps.iter_mut().enumerate() {
                ui.horizontal(|ui| {
                    ui.label(format!("Step {}:", i + 1));

                    ui.add(
                        egui::DragValue::new(&mut step.target_angle)
                            .speed(1.0)
                            .clamp_range(0.0..=270.0)
                            .suffix("°"),
                    );

                    ui.label("for");

                    ui.add(
                        egui::DragValue::new(&mut step.duration)
                            .speed(0.1)
                            .clamp_range(0.1..=10.0)
                            .suffix(" sec"),
                    );
                });
            }

            // Recalculate total duration when steps change
            track.looped_pattern.total_duration = track
                .looped_pattern
                .steps
                .iter()
                .map(|step| step.duration)
                .sum();
        }
    }
}

impl eframe::App for ServoApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Update time
        let current_time = ctx.input(|i| i.time);
        self.last_update_time = current_time;

        // Update from API data
        self.update_from_api_data();

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Servo Position Visualization");

            // Display any error messages
            if let Some(error) = &self.error_message {
                ui.colored_label(egui::Color32::RED, error);
                if ui.button("Clear Error").clicked() {
                    self.error_message = None;
                }
            }

            // Main control section
            ui.collapsing("Controls", |ui| {
                // Display mode toggles
                ui.checkbox(&mut self.show_all_servos, "Show all servos in one chart");
                ui.checkbox(
                    &mut self.use_looped_targets,
                    "Use looped targets (override API)",
                );

                // History duration control
                ui.horizontal(|ui| {
                    ui.label("History Duration (seconds):");
                    ui.add(egui::Slider::new(&mut self.history_duration, 1.0..=60.0));
                });
                
                // Track configuration buttons
                ui.horizontal(|ui| {
                    if ui.button("Reload Track Configurations").clicked() {
                        match self.reload_track_configs() {
                            Ok(_) => {
                                self.error_message = None;
                            },
                            Err(e) => {
                                self.error_message = Some(format!("Failed to reload tracks: {}", e));
                            }
                        }
                    }
                    
                    if ui.button("Save Track Configurations").clicked() {
                        match self.save_all_track_configs() {
                            Ok(saved_files) => {
                                self.error_message = None;
                                println!("Saved {} track configurations", saved_files.len());
                            },
                            Err(e) => {
                                self.error_message = Some(format!("Failed to save tracks: {}", e));
                            }
                        }
                    }
                    
                    if let Some(tracks_dir) = &self.track_configs_path {
                        ui.label(format!("in directory: {}", tracks_dir));
                    }
                });
            });

            // Pattern editor section
            ui.collapsing("Pattern Editor", |ui| {
                // Buttons to enable/disable all patterns at once
                ui.horizontal(|ui| {
                    if ui.button("Enable All Patterns").clicked() {
                        for track in &mut self.tracks {
                            track.looped_pattern.enabled = true;
                        }
                    }
                    if ui.button("Disable All Patterns").clicked() {
                        for track in &mut self.tracks {
                            track.looped_pattern.enabled = false;
                        }
                    }
                });

                for i in 0..self.tracks.len() {
                    ui.collapsing(format!("Servo {} Pattern", self.tracks[i].servo_id), |ui| {
                        self.draw_pattern_editor(ui, i);
                    });
                }
            });

            // Display current values for all servos
            ui.heading("Current Values");
            egui::Grid::new("servo_values_grid")
                .num_columns(4)
                .spacing([40.0, 4.0])
                .striped(true)
                .show(ui, |ui| {
                    ui.strong("Servo ID");
                    ui.strong("Actual Position");
                    ui.strong("Pattern Target");
                    ui.strong("Difference");
                    ui.end_row();

                    for track in &self.tracks {
                        let pattern_target =
                            track.calculate_looped_target(current_time, self.start_time);
                        let diff = (pattern_target - track.current_actual).abs();

                        ui.colored_label(track.color, format!("Servo {}", track.servo_id));
                        ui.label(format!("{:.1}°", track.current_actual));
                        ui.label(format!("{:.1}°", pattern_target));

                        let diff_color = if diff > 5.0 {
                            egui::Color32::RED
                        } else if diff > 2.0 {
                            egui::Color32::YELLOW
                        } else {
                            egui::Color32::GREEN
                        };

                        ui.colored_label(diff_color, format!("{:.1}°", diff));
                        ui.end_row();
                    }
                });

            ui.add_space(10.0);

            // Either show a single combined chart or individual charts
            if self.show_all_servos {
                ui.heading("All Servos");
                self.draw_combined_plot(ui, current_time);
            } else {
                // Show individual charts for each servo
                for track in &self.tracks {
                    ui.collapsing(format!("Servo {} Detail", track.servo_id), |ui| {
                        self.draw_servo_plot(ui, track, current_time);
                    });
                }
            }
        });

        // Keep requesting repaints for animation
        ctx.request_repaint_after(Duration::from_millis(16)); // ~60 FPS
    }

    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        // Stop the data fetcher thread
        *self.data_fetcher_running.lock().unwrap() = false;
    }
}
