use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct DroneEstimate {
    pub position: [f64; 3],
    pub covariance_diag: [f64; 3],
}

/// Decentralised uncertainty-weighted position fusion (information-form combiner).
/// Weight for each drone on axis k: w_k = 1 / max(covariance_diag[k], 1e-9)
/// Fused[k] = Σ(w_k × pos[k]) / Σ(w_k)
#[derive(Debug, Default)]
pub struct PoseConsensus {
    states: HashMap<usize, DroneEstimate>,
}

impl PoseConsensus {
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert or replace a drone's position estimate.
    pub fn update(&mut self, drone_id: usize, position: [f64; 3], covariance_diag: [f64; 3]) {
        self.states
            .insert(drone_id, DroneEstimate { position, covariance_diag });
    }

    /// Weighted-average fused position across all drones, or None if empty.
    pub fn fused_position(&self) -> Option<[f64; 3]> {
        if self.states.is_empty() {
            return None;
        }
        let mut num = [0.0f64; 3];
        let mut den = [0.0f64; 3];
        for est in self.states.values() {
            for k in 0..3 {
                let w = 1.0 / est.covariance_diag[k].max(1e-9);
                num[k] += w * est.position[k];
                den[k] += w;
            }
        }
        Some([num[0] / den[0], num[1] / den[1], num[2] / den[2]])
    }

    /// Simple unweighted mean of all drone positions.
    pub fn swarm_centroid(&self) -> Option<[f64; 3]> {
        if self.states.is_empty() {
            return None;
        }
        let n = self.states.len() as f64;
        let mut sum = [0.0f64; 3];
        for est in self.states.values() {
            for (s, &p) in sum.iter_mut().zip(est.position.iter()) {
                *s += p;
            }
        }
        Some([sum[0] / n, sum[1] / n, sum[2] / n])
    }

    pub fn drone_count(&self) -> usize {
        self.states.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    fn pc() -> PoseConsensus {
        PoseConsensus::new()
    }

    #[test]
    fn test_empty_returns_none() {
        let p = pc();
        assert!(p.fused_position().is_none());
        assert!(p.swarm_centroid().is_none());
    }

    #[test]
    fn test_single_drone_fused_equals_itself() {
        let mut p = pc();
        p.update(0, [3.0, 4.0, 5.0], [1.0, 1.0, 1.0]);
        let f = p.fused_position().unwrap();
        assert!((f[0] - 3.0).abs() < EPS);
        assert!((f[1] - 4.0).abs() < EPS);
        assert!((f[2] - 5.0).abs() < EPS);
    }

    #[test]
    fn test_equal_weights_gives_arithmetic_mean() {
        let mut p = pc();
        p.update(0, [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        p.update(1, [10.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        p.update(2, [20.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let f = p.fused_position().unwrap();
        assert!((f[0] - 10.0).abs() < 1e-6, "expected 10.0, got {}", f[0]);
    }

    #[test]
    fn test_high_certainty_dominates() {
        let mut p = pc();
        p.update(0, [0.0, 0.0, 0.0], [0.01, 0.01, 0.01]);
        p.update(1, [100.0, 0.0, 0.0], [10000.0, 10000.0, 10000.0]);
        let f = p.fused_position().unwrap();
        assert!(f[0] < 5.0, "High-certainty drone should dominate; got x={}", f[0]);
    }

    #[test]
    fn test_update_overwrites_previous() {
        let mut p = pc();
        p.update(0, [1.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        p.update(0, [9.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let f = p.fused_position().unwrap();
        assert!((f[0] - 9.0).abs() < EPS);
    }

    #[test]
    fn test_drone_count() {
        let mut p = pc();
        assert_eq!(p.drone_count(), 0);
        p.update(0, [0.0; 3], [1.0; 3]);
        assert_eq!(p.drone_count(), 1);
        p.update(1, [0.0; 3], [1.0; 3]);
        assert_eq!(p.drone_count(), 2);
        p.update(0, [0.0; 3], [1.0; 3]); // overwrite drone 0, count stays 2
        assert_eq!(p.drone_count(), 2);
    }

    #[test]
    fn test_swarm_centroid_is_unweighted_mean() {
        let mut p = pc();
        p.update(0, [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        p.update(1, [4.0, 0.0, 0.0], [0.01, 0.01, 0.01]); // different weights
        let c = p.swarm_centroid().unwrap();
        // Centroid is unweighted mean: (0+4)/2 = 2
        assert!((c[0] - 2.0).abs() < 1e-6);
    }
}
