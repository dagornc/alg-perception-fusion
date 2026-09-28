//! Simulateur déterministe d'un EKF 2D réduit pour la fusion perception.
//!
//! Le modèle d'état est `[px, py, vx, vy]`. Les deux axes sont indépendants
//! et utilisent chacun un filtre position/vitesse à covariance 2×2. Le modèle
//! est linéaire (cas particulier exact de l'EKF), ce qui rend la référence
//! compacte et auditable. GNSS, odométrie et piste Edge sont fusionnés de
//! manière séquentielle ; un test d'innovation rejette les mesures aberrantes.

use crate::rng::PyRandom;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Scenario {
    Nominal,
    Outliers,
    Edge,
    Dropout,
}

impl Scenario {
    pub fn parse(value: &str) -> Option<Self> {
        match value {
            "nominal" => Some(Self::Nominal),
            "outliers" => Some(Self::Outliers),
            "edge" => Some(Self::Edge),
            "dropout" => Some(Self::Dropout),
            _ => None,
        }
    }

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Nominal => "nominal",
            Self::Outliers => "outliers",
            Self::Edge => "edge",
            Self::Dropout => "dropout",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Resultat {
    pub seed: u64,
    pub scenario: Scenario,
    pub steps: usize,
    pub rmse: f64,
    pub convergence_step: usize,
    pub false_tracks: usize,
    pub rejected_outliers: usize,
    pub accepted_measurements: usize,
}

#[derive(Clone, Copy)]
struct AxisFilter {
    pos: f64,
    vel: f64,
    p00: f64,
    p01: f64,
    p10: f64,
    p11: f64,
}

impl AxisFilter {
    fn new() -> Self {
        Self {
            pos: 0.0,
            vel: 0.0,
            p00: 100.0,
            p01: 0.0,
            p10: 0.0,
            p11: 25.0,
        }
    }

    fn predict(&mut self) {
        self.pos += self.vel;
        let n00 = self.p00 + self.p01 + self.p10 + self.p11 + 0.04;
        let n01 = self.p01 + self.p11;
        let n10 = self.p10 + self.p11;
        let n11 = self.p11 + 0.01;
        self.p00 = n00;
        self.p01 = n01;
        self.p10 = n10;
        self.p11 = n11;
    }

    fn update_position(&mut self, measurement: f64, variance: f64, gated: bool) -> bool {
        let residual = measurement - self.pos;
        let innovation_variance = self.p00 + variance;
        if gated && residual.abs() > 4.0 * innovation_variance.sqrt() {
            return false;
        }
        let k0 = self.p00 / innovation_variance;
        let k1 = self.p10 / innovation_variance;
        let old_p00 = self.p00;
        let old_p01 = self.p01;
        self.pos += k0 * residual;
        self.vel += k1 * residual;
        self.p00 = (1.0 - k0) * old_p00;
        self.p01 = (1.0 - k0) * old_p01;
        self.p10 -= k1 * old_p00;
        self.p11 -= k1 * old_p01;
        true
    }

    fn update_velocity(&mut self, measurement: f64, variance: f64) {
        let residual = measurement - self.vel;
        let innovation_variance = self.p11 + variance;
        let k0 = self.p01 / innovation_variance;
        let k1 = self.p11 / innovation_variance;
        let old_p10 = self.p10;
        let old_p11 = self.p11;
        self.pos += k0 * residual;
        self.vel += k1 * residual;
        self.p00 -= k0 * old_p10;
        self.p01 -= k0 * old_p11;
        self.p10 = (1.0 - k1) * old_p10;
        self.p11 = (1.0 - k1) * old_p11;
    }
}

fn centered(rng: &mut PyRandom, amplitude: f64) -> f64 {
    (rng.random() * 2.0 - 1.0) * amplitude
}

/// Exécute la fusion locale et, pour `edge`, une mesure de piste Edge.
///
/// `steps` est ramené à au moins un pas afin de conserver des métriques finies.
pub fn simuler(seed: u64, scenario: Scenario, steps: usize) -> Resultat {
    let steps = steps.max(1);
    let mut rng = PyRandom::new(seed);
    let mut true_x = centered(&mut rng, 12.0);
    let mut true_y = centered(&mut rng, 12.0);
    let mut true_vx = centered(&mut rng, 1.0);
    let mut true_vy = centered(&mut rng, 1.0);
    let mut fx = AxisFilter::new();
    let mut fy = AxisFilter::new();
    let mut squared_error_sum = 0.0;
    let mut convergence_step = 0;
    let mut false_tracks = 0;
    let mut rejected_outliers = 0;
    let mut accepted_measurements = 0;

    for step in 1..=steps {
        true_vx += centered(&mut rng, 0.08);
        true_vy += centered(&mut rng, 0.08);
        true_x += true_vx;
        true_y += true_vy;
        fx.predict();
        fy.predict();

        let odom_vx = true_vx + centered(&mut rng, 0.30);
        let odom_vy = true_vy + centered(&mut rng, 0.30);
        fx.update_velocity(odom_vx, 0.09);
        fy.update_velocity(odom_vy, 0.09);
        accepted_measurements += 2;

        let gnss_available = scenario != Scenario::Dropout || step % 4 == 0;
        if gnss_available {
            let injected_outlier = scenario == Scenario::Outliers && step % 9 == 0;
            let offset = if injected_outlier { 30.0 } else { 0.0 };
            let gnss_x = true_x + centered(&mut rng, 1.5) + offset;
            let gnss_y = true_y + centered(&mut rng, 1.5) - offset;
            let accepted_x = fx.update_position(gnss_x, 2.25, true);
            let accepted_y = fy.update_position(gnss_y, 2.25, true);
            accepted_measurements += usize::from(accepted_x) + usize::from(accepted_y);
            rejected_outliers += usize::from(!accepted_x) + usize::from(!accepted_y);
            if injected_outlier && (accepted_x || accepted_y) {
                false_tracks += 1;
            }
        }

        if scenario == Scenario::Edge && step % 4 == 0 {
            let edge_x = true_x + centered(&mut rng, 0.6);
            let edge_y = true_y + centered(&mut rng, 0.6);
            accepted_measurements += usize::from(fx.update_position(edge_x, 0.36, true));
            accepted_measurements += usize::from(fy.update_position(edge_y, 0.36, true));
        }

        let dx = fx.pos - true_x;
        let dy = fy.pos - true_y;
        let squared_error = dx * dx + dy * dy;
        squared_error_sum += squared_error;
        if convergence_step == 0 && squared_error.sqrt() <= 2.0 {
            convergence_step = step;
        }
    }

    Resultat {
        seed,
        scenario,
        steps,
        rmse: (squared_error_sum / steps as f64).sqrt(),
        convergence_step,
        false_tracks,
        rejected_outliers,
        accepted_measurements,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scenario_round_trip() {
        for scenario in [
            Scenario::Nominal,
            Scenario::Outliers,
            Scenario::Edge,
            Scenario::Dropout,
        ] {
            assert_eq!(Scenario::parse(scenario.as_str()), Some(scenario));
        }
        assert_eq!(Scenario::parse("inconnu"), None);
    }

    #[test]
    fn zero_step_est_normalise() {
        assert_eq!(simuler(1, Scenario::Nominal, 0).steps, 1);
    }
}
