use std::collections::HashMap;

use crate::conflict_solver::Waypoint3D;

#[derive(Debug, Clone)]
pub struct DroneState {
    pub id: usize,
    pub position: Waypoint3D,
    pub last_wp: Option<Waypoint3D>,
}

/// Greedy auction: for each goal (in index order) assign it to the drone with
/// minimum squared Euclidean distance from its current endpoint. Mutates
/// drones' `last_wp` to track cumulative assignment state.
pub fn assign(drones: &mut [DroneState], goals: &[Waypoint3D]) -> HashMap<usize, Vec<usize>> {
    let mut assignment: HashMap<usize, Vec<usize>> =
        drones.iter().map(|d| (d.id, vec![])).collect();

    for (goal_idx, &goal) in goals.iter().enumerate() {
        let best_id = drones
            .iter()
            .min_by_key(|d| {
                let from = d.last_wp.unwrap_or(d.position);
                let dx = (from.x - goal.x) as i64;
                let dy = (from.y - goal.y) as i64;
                let dz = (from.z - goal.z) as i64;
                dx * dx + dy * dy + dz * dz
            })
            .unwrap()
            .id;

        assignment.get_mut(&best_id).unwrap().push(goal_idx);
        let drone = drones.iter_mut().find(|d| d.id == best_id).unwrap();
        drone.last_wp = Some(goal);
    }

    assignment
}

/// Sum of Euclidean distances each drone travels to visit its assigned goals in order.
pub fn total_distance(
    drones: &[DroneState],
    goals: &[Waypoint3D],
    assignment: &HashMap<usize, Vec<usize>>,
) -> f64 {
    drones
        .iter()
        .map(|d| {
            let indices = assignment.get(&d.id).map(|v| v.as_slice()).unwrap_or(&[]);
            let mut pos = d.position;
            let mut dist = 0.0f64;
            for &idx in indices {
                let g = goals[idx];
                let dx = (pos.x - g.x) as f64;
                let dy = (pos.y - g.y) as f64;
                let dz = (pos.z - g.z) as f64;
                dist += (dx * dx + dy * dy + dz * dz).sqrt();
                pos = g;
            }
            dist
        })
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::{rngs::StdRng, seq::SliceRandom, SeedableRng};

    fn d(id: usize, x: i32, y: i32, z: i32) -> DroneState {
        DroneState {
            id,
            position: Waypoint3D::new(x, y, z),
            last_wp: None,
        }
    }

    fn w(x: i32, y: i32, z: i32) -> Waypoint3D {
        Waypoint3D::new(x, y, z)
    }

    #[test]
    fn test_single_drone_gets_all_goals() {
        let mut drones = vec![d(0, 0, 0, 0)];
        let goals = vec![w(1, 0, 0), w(2, 0, 0), w(3, 0, 0)];
        let a = assign(&mut drones, &goals);
        assert_eq!(a.get(&0).unwrap().len(), 3);
    }

    #[test]
    fn test_empty_goals() {
        let mut drones = vec![d(0, 0, 0, 0), d(1, 5, 0, 0)];
        let a = assign(&mut drones, &[]);
        assert!(a.get(&0).unwrap().is_empty());
        assert!(a.get(&1).unwrap().is_empty());
    }

    #[test]
    fn test_two_drones_split_by_proximity() {
        let mut drones = vec![d(0, 0, 0, 0), d(1, 10, 0, 0)];
        let goals = vec![w(1, 0, 0), w(9, 0, 0), w(2, 0, 0), w(8, 0, 0)];
        let a = assign(&mut drones, &goals);
        let d0 = a.get(&0).unwrap();
        let d1 = a.get(&1).unwrap();
        // Goals near x=0 (indices 0,2) should go to drone 0; goals near x=10 (1,3) to drone 1
        assert!(d0.contains(&0) || d0.contains(&2));
        assert!(d1.contains(&1) || d1.contains(&3));
    }

    #[test]
    fn test_last_wp_updated_after_assignment() {
        let mut drones = vec![d(0, 0, 0, 0)];
        let goals = vec![w(5, 0, 0), w(10, 0, 0)];
        assign(&mut drones, &goals);
        assert_eq!(drones[0].last_wp, Some(w(10, 0, 0)));
    }

    #[test]
    fn test_deterministic() {
        let mut drones_a = vec![d(0, 0, 0, 0), d(1, 5, 0, 0)];
        let mut drones_b = vec![d(0, 0, 0, 0), d(1, 5, 0, 0)];
        let goals = vec![w(1, 0, 0), w(4, 0, 0), w(2, 0, 0), w(6, 0, 0)];
        let a = assign(&mut drones_a, &goals);
        let b = assign(&mut drones_b, &goals);
        assert_eq!(a.get(&0).unwrap(), b.get(&0).unwrap());
        assert_eq!(a.get(&1).unwrap(), b.get(&1).unwrap());
    }

    #[test]
    fn test_greedy_beats_random_over_seeds() {
        let drones_base = vec![
            d(0, 0, 0, 0),
            d(1, 50, 0, 0),
            d(2, 25, 25, 0),
        ];
        let goals = vec![
            w(5, 0, 0),
            w(45, 0, 0),
            w(10, 0, 0),
            w(40, 0, 0),
            w(20, 10, 0),
            w(30, 10, 0),
        ];

        for seed in 0u64..20 {
            let mut drones = drones_base.clone();
            let a = assign(&mut drones, &goals);
            let greedy_dist = total_distance(&drones_base, &goals, &a);

            let mut rng = StdRng::seed_from_u64(seed);
            let mut random_total = 0.0f64;
            let n_samples = 1000usize;
            for _ in 0..n_samples {
                let mut order: Vec<usize> = (0..goals.len()).collect();
                order.shuffle(&mut rng);
                let per = goals.len() / drones_base.len();
                let mut rand_assign: HashMap<usize, Vec<usize>> = HashMap::new();
                for (di, drone) in drones_base.iter().enumerate() {
                    let start = di * per;
                    let end = if di == drones_base.len() - 1 {
                        goals.len()
                    } else {
                        start + per
                    };
                    rand_assign.insert(drone.id, order[start..end].to_vec());
                }
                random_total += total_distance(&drones_base, &goals, &rand_assign);
            }
            let random_mean = random_total / n_samples as f64;
            assert!(
                greedy_dist <= random_mean,
                "seed={seed}: greedy {greedy_dist:.1} > random mean {random_mean:.1}"
            );
        }
    }
}
