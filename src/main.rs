use perception_fusion_rs::{Scenario, simuler};
use std::env;
use std::process::ExitCode;

fn usage(program: &str) {
    eprintln!("Usage: {program} <seed> <scenario: nominal|outliers|edge|dropout> <steps>");
}

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() != 4 {
        usage(&args[0]);
        return ExitCode::from(2);
    }
    let seed = match args[1].parse::<u64>() {
        Ok(value) => value,
        Err(_) => {
            eprintln!("seed invalide: {}", args[1]);
            return ExitCode::from(2);
        }
    };
    let scenario = match Scenario::parse(&args[2]) {
        Some(value) => value,
        None => {
            eprintln!("scenario invalide: {}", args[2]);
            return ExitCode::from(2);
        }
    };
    let steps = match args[3].parse::<usize>() {
        Ok(value) if value > 0 => value,
        _ => {
            eprintln!("steps doit etre un entier strictement positif");
            return ExitCode::from(2);
        }
    };

    let r = simuler(seed, scenario, steps);
    println!(
        "seed,scenario,steps,rmse,convergence_step,false_tracks,rejected_outliers,accepted_measurements"
    );
    println!(
        "{},{},{},{:.12},{},{},{},{}",
        r.seed,
        r.scenario.as_str(),
        r.steps,
        r.rmse,
        r.convergence_step,
        r.false_tracks,
        r.rejected_outliers,
        r.accepted_measurements
    );
    ExitCode::SUCCESS
}
