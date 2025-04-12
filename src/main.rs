use eframe::{self, egui::ViewportBuilder};
use leg_studio::ServoApp;
use std::env;

fn main() -> Result<(), eframe::Error> {
    env_logger::init(); // Log to stderr

    // API endpoint for the servo controller
    let api_url = "http://192.168.1.154:3030/servos";

    // Parse command-line arguments for tracks directory
    let args: Vec<String> = env::args().collect();
    let tracks_dir = if args.len() > 1 {
        // Use the first argument as the tracks directory
        args[1].clone()
    } else {
        // Default to the current directory if no argument is provided
        "tracks".to_string()
    };

    let options = eframe::NativeOptions {
        viewport: ViewportBuilder {
            title: Some("Servo Visualization".to_owned()),
            min_inner_size: Some(egui::vec2(1200.0, 800.0)),
            ..Default::default()
        },
        ..Default::default()
    };

    println!("Using tracks directory: {:?}", tracks_dir);

    eframe::run_native(
        "Servo Visualization",
        options,
        Box::new(move |cc| {
            let mut app = ServoApp::new(cc, &tracks_dir, api_url);
            app.track_configs_path = Some(tracks_dir);
            Box::new(app)
        }),
    )
}
