use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

use super::grid::Grid3D;
use super::types::{Agent, Constraint, EdgeConstraint, Waypoint3D};

// (f_cost, g_cost, x, y, z, time) — stored reversed for min-heap behaviour
type HeapEntry = Reverse<(u32, u32, i32, i32, i32, u32)>;

/// Time-extended A* for a single agent with vertex and edge constraints.
/// Returns the path as a Vec<Waypoint3D> from start to goal (inclusive),
/// or None if no path exists within max_time steps.
pub fn astar_single(
    agent: &Agent,
    constraints: &[Constraint],
    edge_constraints: &[EdgeConstraint],
    grid: &Grid3D,
    max_time: u32,
) -> Option<Vec<Waypoint3D>> {
    let mut open: BinaryHeap<HeapEntry> = BinaryHeap::new();
    // (point, time) -> (prev_point, prev_time)
    let mut came_from: HashMap<(Waypoint3D, u32), (Waypoint3D, u32)> = HashMap::new();
    let mut best_g: HashMap<(Waypoint3D, u32), u32> = HashMap::new();

    let start = agent.start;
    let goal = agent.goal;

    let h0 = start.manhattan(&goal);
    open.push(Reverse((h0, 0, start.x, start.y, start.z, 0)));
    best_g.insert((start, 0), 0);

    while let Some(Reverse((_, g, x, y, z, t))) = open.pop() {
        let cur = Waypoint3D::new(x, y, z);

        // Skip stale queue entries
        if g > *best_g.get(&(cur, t)).unwrap_or(&u32::MAX) {
            continue;
        }

        if cur == goal {
            return Some(reconstruct_path(&came_from, goal, t));
        }

        if t >= max_time {
            continue;
        }

        // 6-directional moves + wait-in-place
        let mut neighbors = grid.successors(cur);
        neighbors.push(cur); // wait

        for next in neighbors {
            let nt = t + 1;

            // Vertex constraint: agent blocked at `next` at time `nt`
            if constraints
                .iter()
                .any(|c| c.agent == agent.id && c.point == next && c.time == nt)
            {
                continue;
            }

            // Edge (swap) constraint: agent blocked on move cur→next at time t
            if edge_constraints.iter().any(|ec| {
                ec.agent == agent.id && ec.from == cur && ec.to == next && ec.time == t
            }) {
                continue;
            }

            let ng = g + 1;
            let entry = best_g.entry((next, nt)).or_insert(u32::MAX);
            if ng < *entry {
                *entry = ng;
                came_from.insert((next, nt), (cur, t));
                let h = next.manhattan(&goal);
                open.push(Reverse((ng + h, ng, next.x, next.y, next.z, nt)));
            }
        }
    }

    None
}

fn reconstruct_path(
    came_from: &HashMap<(Waypoint3D, u32), (Waypoint3D, u32)>,
    goal: Waypoint3D,
    t: u32,
) -> Vec<Waypoint3D> {
    let mut path = vec![goal];
    let mut state = (goal, t);
    while let Some(&(prev_point, prev_t)) = came_from.get(&state) {
        path.push(prev_point);
        state = (prev_point, prev_t);
    }
    path.reverse();
    path
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_agent(id: usize, sx: i32, sy: i32, sz: i32, gx: i32, gy: i32, gz: i32) -> Agent {
        Agent {
            id,
            start: Waypoint3D::new(sx, sy, sz),
            goal: Waypoint3D::new(gx, gy, gz),
        }
    }

    fn open_grid() -> Grid3D {
        Grid3D::new(20, 20, 10)
    }

    #[test]
    fn test_straight_path_length() {
        let agent = make_agent(0, 0, 0, 0, 5, 0, 0);
        let g = open_grid();
        let path = astar_single(&agent, &[], &[], &g, 100).unwrap();
        assert_eq!(path.len(), 6); // start + 5 steps
        assert_eq!(path[0], Waypoint3D::new(0, 0, 0));
        assert_eq!(*path.last().unwrap(), Waypoint3D::new(5, 0, 0));
    }

    #[test]
    fn test_start_equals_goal() {
        let agent = make_agent(0, 3, 3, 3, 3, 3, 3);
        let g = open_grid();
        let path = astar_single(&agent, &[], &[], &g, 100).unwrap();
        assert_eq!(path.len(), 1);
        assert_eq!(path[0], Waypoint3D::new(3, 3, 3));
    }

    #[test]
    fn test_detour_around_obstacle() {
        let mut g = Grid3D::new(10, 10, 5);
        g.block(Waypoint3D::new(3, 0, 2));
        let agent = make_agent(0, 0, 0, 2, 5, 0, 2);
        let path = astar_single(&agent, &[], &[], &g, 100).unwrap();
        assert_eq!(*path.last().unwrap(), Waypoint3D::new(5, 0, 2));
        assert!(!path.contains(&Waypoint3D::new(3, 0, 2)));
    }

    #[test]
    fn test_no_path_enclosed() {
        let mut g = Grid3D::new(10, 10, 5);
        let start = Waypoint3D::new(5, 5, 2);
        for &(dx, dy, dz) in &[
            (1, 0, 0),
            (-1, 0, 0),
            (0, 1, 0),
            (0, -1, 0),
            (0, 0, 1),
            (0, 0, -1),
        ] {
            g.block(Waypoint3D::new(5 + dx, 5 + dy, 2 + dz));
        }
        let agent = Agent {
            id: 0,
            start,
            goal: Waypoint3D::new(0, 0, 0),
        };
        assert!(astar_single(&agent, &[], &[], &g, 200).is_none());
    }

    #[test]
    fn test_vertex_constraint_forces_wait() {
        let agent = make_agent(0, 0, 0, 0, 2, 0, 0);
        let g = open_grid();
        let c = Constraint {
            agent: 0,
            point: Waypoint3D::new(1, 0, 0),
            time: 1,
        };
        let path = astar_single(&agent, &[c], &[], &g, 100).unwrap();
        assert_eq!(*path.last().unwrap(), Waypoint3D::new(2, 0, 0));
        assert_ne!(path.get(1), Some(&Waypoint3D::new(1, 0, 0)));
    }

    #[test]
    fn test_edge_constraint_prevents_move() {
        let agent = make_agent(0, 0, 0, 0, 2, 0, 0);
        let g = open_grid();
        let ec = EdgeConstraint {
            agent: 0,
            from: Waypoint3D::new(0, 0, 0),
            to: Waypoint3D::new(1, 0, 0),
            time: 0,
        };
        let path = astar_single(&agent, &[], &[ec], &g, 100).unwrap();
        assert_eq!(*path.last().unwrap(), Waypoint3D::new(2, 0, 0));
        assert_ne!(path.get(1), Some(&Waypoint3D::new(1, 0, 0)));
    }

    #[test]
    fn test_max_time_terminates() {
        let agent = make_agent(0, 0, 0, 0, 100, 0, 0);
        let g = open_grid();
        let result = astar_single(&agent, &[], &[], &g, 5);
        assert!(result.is_none());
    }
}
