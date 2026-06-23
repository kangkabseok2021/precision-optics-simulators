use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

use super::astar::astar_single;
use super::grid::Grid3D;
use super::types::{Agent, CbsError, Constraint, EdgeConstraint, Waypoint3D};

const MAX_TIME: u32 = 200;

// ── Internal conflict types ────────────────────────────────────────────────

struct VertexConflict {
    agent_a: usize,
    agent_b: usize,
    point: Waypoint3D,
    time: u32,
}

struct EdgeConflict {
    agent_a: usize,
    agent_b: usize,
    a_from: Waypoint3D,
    a_to: Waypoint3D,
    time: u32,
}

enum Conflict {
    Vertex(VertexConflict),
    Edge(EdgeConflict),
}

enum NewConstraint {
    Vertex(Constraint),
    Edge(EdgeConstraint),
}

// ── CBS constraint-tree node ───────────────────────────────────────────────

struct CbsNode {
    constraints: Vec<Constraint>,
    edge_constraints: Vec<EdgeConstraint>,
    paths: HashMap<usize, Vec<Waypoint3D>>,
    cost: u32,
    id: usize,
}

// ── Public API ─────────────────────────────────────────────────────────────

pub struct Cbs {
    max_nodes: usize,
}

impl Cbs {
    pub fn new(max_nodes: usize) -> Self {
        Self { max_nodes }
    }

    /// Returns conflict-free paths for all agents, or an error.
    pub fn plan(
        &self,
        agents: &[Agent],
        grid: &Grid3D,
    ) -> Result<HashMap<usize, Vec<Waypoint3D>>, CbsError> {
        let mut counter = 0usize;

        // Build root: each agent's unconstrained shortest path
        let mut root_paths = HashMap::new();
        for agent in agents {
            match astar_single(agent, &[], &[], grid, MAX_TIME) {
                Some(p) => {
                    root_paths.insert(agent.id, p);
                }
                None => return Err(CbsError::NoPathFound(agent.id)),
            }
        }
        let root_cost: u32 = root_paths.values().map(|p| p.len() as u32).sum();
        let root = CbsNode {
            constraints: vec![],
            edge_constraints: vec![],
            paths: root_paths,
            cost: root_cost,
            id: counter,
        };
        counter += 1;

        // Min-heap by (cost, id); id ensures deterministic tie-breaking
        let mut open: BinaryHeap<Reverse<(u32, usize)>> = BinaryHeap::new();
        let mut nodes: HashMap<usize, CbsNode> = HashMap::new();
        open.push(Reverse((root.cost, root.id)));
        nodes.insert(root.id, root);

        while let Some(Reverse((_, nid))) = open.pop() {
            let node = nodes.remove(&nid).unwrap();

            match first_conflict(&node.paths) {
                None => return Ok(node.paths),
                Some(conflict) => {
                    for new_c in expand(&conflict) {
                        let mut child_v = node.constraints.clone();
                        let mut child_e = node.edge_constraints.clone();

                        let constrained_id = match &new_c {
                            NewConstraint::Vertex(c) => {
                                let id = c.agent;
                                child_v.push(c.clone());
                                id
                            }
                            NewConstraint::Edge(ec) => {
                                let id = ec.agent;
                                child_e.push(ec.clone());
                                id
                            }
                        };

                        let mut child_paths = node.paths.clone();
                        let agent = agents.iter().find(|a| a.id == constrained_id).unwrap();
                        match astar_single(agent, &child_v, &child_e, grid, MAX_TIME) {
                            Some(p) => {
                                child_paths.insert(constrained_id, p);
                            }
                            None => continue, // infeasible branch
                        }

                        let child_cost: u32 = child_paths.values().map(|p| p.len() as u32).sum();
                        let child_id = counter;
                        counter += 1;

                        if counter > self.max_nodes {
                            return Err(CbsError::Timeout);
                        }

                        let child = CbsNode {
                            constraints: child_v,
                            edge_constraints: child_e,
                            paths: child_paths,
                            cost: child_cost,
                            id: child_id,
                        };
                        open.push(Reverse((child_cost, child_id)));
                        nodes.insert(child_id, child);
                    }
                }
            }
        }

        Err(CbsError::NoPathFound(0))
    }
}

// ── Conflict detection ─────────────────────────────────────────────────────

/// Returns position of agent at time t; if path is shorter, agent stays at goal.
fn pos_at(path: &[Waypoint3D], t: usize) -> Waypoint3D {
    *path.get(t).unwrap_or_else(|| path.last().unwrap())
}

fn first_conflict(paths: &HashMap<usize, Vec<Waypoint3D>>) -> Option<Conflict> {
    let mut ids: Vec<usize> = paths.keys().copied().collect();
    ids.sort_unstable();

    let max_t = paths.values().map(|p| p.len()).max().unwrap_or(0);

    for t in 0..max_t {
        for i in 0..ids.len() {
            for j in (i + 1)..ids.len() {
                let a = ids[i];
                let b = ids[j];
                let pa = pos_at(paths.get(&a).unwrap(), t);
                let pb = pos_at(paths.get(&b).unwrap(), t);

                // Vertex conflict
                if pa == pb {
                    return Some(Conflict::Vertex(VertexConflict {
                        agent_a: a,
                        agent_b: b,
                        point: pa,
                        time: t as u32,
                    }));
                }

                // Edge (swap) conflict
                if t + 1 < max_t {
                    let pa1 = pos_at(paths.get(&a).unwrap(), t + 1);
                    let pb1 = pos_at(paths.get(&b).unwrap(), t + 1);
                    if pa == pb1 && pa1 == pb {
                        return Some(Conflict::Edge(EdgeConflict {
                            agent_a: a,
                            agent_b: b,
                            a_from: pa,
                            a_to: pa1,
                            time: t as u32,
                        }));
                    }
                }
            }
        }
    }

    None
}

fn expand(conflict: &Conflict) -> Vec<NewConstraint> {
    match conflict {
        Conflict::Vertex(v) => vec![
            NewConstraint::Vertex(Constraint {
                agent: v.agent_a,
                point: v.point,
                time: v.time,
            }),
            NewConstraint::Vertex(Constraint {
                agent: v.agent_b,
                point: v.point,
                time: v.time,
            }),
        ],
        Conflict::Edge(e) => vec![
            NewConstraint::Edge(EdgeConstraint {
                agent: e.agent_a,
                from: e.a_from,
                to: e.a_to,
                time: e.time,
            }),
            NewConstraint::Edge(EdgeConstraint {
                agent: e.agent_b,
                from: e.a_to, // reverse direction for B
                to: e.a_from,
                time: e.time,
            }),
        ],
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn w(x: i32, y: i32, z: i32) -> Waypoint3D {
        Waypoint3D::new(x, y, z)
    }

    fn agent(id: usize, sx: i32, sy: i32, sz: i32, gx: i32, gy: i32, gz: i32) -> Agent {
        Agent {
            id,
            start: w(sx, sy, sz),
            goal: w(gx, gy, gz),
        }
    }

    fn open(width: i32, height: i32, depth: i32) -> Grid3D {
        Grid3D::new(width, height, depth)
    }

    fn cbs() -> Cbs {
        Cbs::new(10_000)
    }

    fn assert_no_collision(paths: &HashMap<usize, Vec<Waypoint3D>>) {
        let mut ids: Vec<usize> = paths.keys().copied().collect();
        ids.sort_unstable();
        let max_t = paths.values().map(|p| p.len()).max().unwrap_or(0);
        for t in 0..max_t {
            for i in 0..ids.len() {
                for j in (i + 1)..ids.len() {
                    let pa = pos_at(paths.get(&ids[i]).unwrap(), t);
                    let pb = pos_at(paths.get(&ids[j]).unwrap(), t);
                    assert_ne!(
                        pa, pb,
                        "Collision at t={t}: agent {} and {} both at {pa:?}",
                        ids[i], ids[j]
                    );
                }
            }
        }
    }

    #[test]
    fn test_single_agent_straight() {
        let agents = vec![agent(0, 0, 0, 0, 4, 0, 0)];
        let result = cbs().plan(&agents, &open(10, 10, 5)).unwrap();
        let path = result.get(&0).unwrap();
        assert_eq!(path[0], w(0, 0, 0));
        assert_eq!(*path.last().unwrap(), w(4, 0, 0));
        assert_eq!(path.len(), 5);
    }

    #[test]
    fn test_two_agents_no_conflict() {
        let agents = vec![agent(0, 0, 0, 0, 4, 0, 0), agent(1, 0, 4, 0, 4, 4, 0)];
        let result = cbs().plan(&agents, &open(10, 10, 5)).unwrap();
        assert_no_collision(&result);
    }

    #[test]
    fn test_two_agents_vertex_conflict_resolved() {
        let agents = vec![agent(0, 0, 0, 2, 4, 0, 2), agent(1, 4, 0, 2, 0, 0, 2)];
        let result = cbs().plan(&agents, &open(10, 10, 5)).unwrap();
        assert_no_collision(&result);
        assert_eq!(*result.get(&0).unwrap().last().unwrap(), w(4, 0, 2));
        assert_eq!(*result.get(&1).unwrap().last().unwrap(), w(0, 0, 2));
    }

    #[test]
    fn test_two_agents_edge_conflict_resolved() {
        let agents = vec![agent(0, 0, 0, 2, 1, 0, 2), agent(1, 1, 0, 2, 0, 0, 2)];
        let result = cbs().plan(&agents, &open(10, 10, 5)).unwrap();
        assert_no_collision(&result);
    }

    #[test]
    fn test_no_fly_zone_respected() {
        let mut g = Grid3D::new(10, 10, 5);
        g.block(w(2, 0, 2));
        let agents = vec![agent(0, 0, 0, 2, 4, 0, 2)];
        let result = cbs().plan(&agents, &g).unwrap();
        let path = result.get(&0).unwrap();
        assert!(!path.contains(&w(2, 0, 2)));
    }

    #[test]
    fn test_no_path_returns_error() {
        let mut g = Grid3D::new(10, 10, 5);
        for &(dx, dy, dz) in &[
            (1, 0, 0),
            (-1, 0, 0),
            (0, 1, 0),
            (0, -1, 0),
            (0, 0, 1),
            (0, 0, -1),
        ] {
            g.block(w(5 + dx, 5 + dy, 2 + dz));
        }
        let agents = vec![Agent {
            id: 0,
            start: w(5, 5, 2),
            goal: w(0, 0, 0),
        }];
        assert!(matches!(
            cbs().plan(&agents, &g),
            Err(CbsError::NoPathFound(_))
        ));
    }

    #[test]
    fn test_timeout_terminates() {
        // Very tight max_nodes forces early exit — must not hang
        let cbs_tight = Cbs::new(3);
        let agents = vec![agent(0, 0, 0, 2, 9, 0, 2), agent(1, 9, 0, 2, 0, 0, 2)];
        let _ = cbs_tight.plan(&agents, &open(10, 10, 5));
    }

    #[test]
    fn test_three_agents_no_collision() {
        let agents = vec![
            agent(0, 0, 0, 0, 4, 0, 0),
            agent(1, 0, 0, 2, 4, 0, 2),
            agent(2, 0, 4, 0, 4, 4, 0),
        ];
        let result = cbs().plan(&agents, &open(10, 10, 5)).unwrap();
        assert_no_collision(&result);
        assert_eq!(*result.get(&0).unwrap().last().unwrap(), w(4, 0, 0));
        assert_eq!(*result.get(&1).unwrap().last().unwrap(), w(4, 0, 2));
        assert_eq!(*result.get(&2).unwrap().last().unwrap(), w(4, 4, 0));
    }

    #[test]
    fn test_deterministic() {
        let agents = vec![agent(0, 0, 0, 2, 4, 0, 2), agent(1, 4, 0, 2, 0, 0, 2)];
        let g = open(10, 10, 5);
        let r1 = cbs().plan(&agents, &g).unwrap();
        let r2 = cbs().plan(&agents, &g).unwrap();
        assert_eq!(r1.get(&0).unwrap(), r2.get(&0).unwrap());
        assert_eq!(r1.get(&1).unwrap(), r2.get(&1).unwrap());
    }
}
