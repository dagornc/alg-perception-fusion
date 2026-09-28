use perception_fusion_rs::{Scenario, simuler};

#[test]
fn nominal_converge_et_reduit_erreur() {
    for seed in 1000..1020 {
        let r = simuler(seed, Scenario::Nominal, 80);
        assert!(r.rmse < 2.0, "seed={seed}, rmse={}", r.rmse);
        assert!(r.convergence_step > 0 && r.convergence_step <= 80);
    }
}

#[test]
fn outliers_sont_rejetes_sans_fausse_piste() {
    for seed in 1000..1020 {
        let r = simuler(seed, Scenario::Outliers, 80);
        assert!(
            r.rejected_outliers >= 8,
            "seed={seed}, rejected={}",
            r.rejected_outliers
        );
        assert_eq!(r.false_tracks, 0, "seed={seed}");
        assert!(r.rmse < 2.5, "seed={seed}, rmse={}", r.rmse);
    }
}

#[test]
fn edge_ajoute_des_mesures() {
    let nominal = simuler(1234, Scenario::Nominal, 80);
    let edge = simuler(1234, Scenario::Edge, 80);
    assert!(edge.accepted_measurements > nominal.accepted_measurements);
}

#[test]
fn dropout_reste_borne() {
    for seed in 1000..1020 {
        let r = simuler(seed, Scenario::Dropout, 80);
        assert!(r.rmse < 4.0, "seed={seed}, rmse={}", r.rmse);
    }
}

#[test]
fn simulation_est_deterministe() {
    for scenario in [
        Scenario::Nominal,
        Scenario::Outliers,
        Scenario::Edge,
        Scenario::Dropout,
    ] {
        assert_eq!(simuler(42, scenario, 60), simuler(42, scenario, 60));
    }
}

#[test]
fn horizon_minimal_est_accepte() {
    let r = simuler(7, Scenario::Nominal, 1);
    assert!(r.rmse.is_finite());
    assert!(r.accepted_measurements > 0);
}
