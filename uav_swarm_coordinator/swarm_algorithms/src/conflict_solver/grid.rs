use std::collections::HashSet;

use super::types::Waypoint3D;

pub struct Grid3D {
    pub width: i32,
    pub height: i32,
    pub depth: i32,
    blocked: HashSet<Waypoint3D>,
}

impl Grid3D {
    pub fn new(width: i32, height: i32, depth: i32) -> Self {
        Self {
            width,
            height,
            depth,
            blocked: HashSet::new(),
        }
    }

    pub fn block(&mut self, p: Waypoint3D) {
        self.blocked.insert(p);
    }

    pub fn is_blocked(&self, p: Waypoint3D) -> bool {
        p.x < 0
            || p.y < 0
            || p.z < 0
            || p.x >= self.width
            || p.y >= self.height
            || p.z >= self.depth
            || self.blocked.contains(&p)
    }

    pub fn successors(&self, p: Waypoint3D) -> Vec<Waypoint3D> {
        [
            Waypoint3D::new(p.x + 1, p.y, p.z),
            Waypoint3D::new(p.x - 1, p.y, p.z),
            Waypoint3D::new(p.x, p.y + 1, p.z),
            Waypoint3D::new(p.x, p.y - 1, p.z),
            Waypoint3D::new(p.x, p.y, p.z + 1),
            Waypoint3D::new(p.x, p.y, p.z - 1),
        ]
        .into_iter()
        .filter(|&s| !self.is_blocked(s))
        .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_in_bounds_not_blocked() {
        let g = Grid3D::new(10, 10, 5);
        assert!(!g.is_blocked(Waypoint3D::new(0, 0, 0)));
        assert!(!g.is_blocked(Waypoint3D::new(9, 9, 4)));
    }

    #[test]
    fn test_out_of_bounds_blocked() {
        let g = Grid3D::new(10, 10, 5);
        assert!(g.is_blocked(Waypoint3D::new(-1, 0, 0)));
        assert!(g.is_blocked(Waypoint3D::new(10, 0, 0)));
        assert!(g.is_blocked(Waypoint3D::new(0, 10, 0)));
        assert!(g.is_blocked(Waypoint3D::new(0, 0, 5)));
    }

    #[test]
    fn test_explicit_block() {
        let mut g = Grid3D::new(10, 10, 5);
        let p = Waypoint3D::new(3, 3, 2);
        assert!(!g.is_blocked(p));
        g.block(p);
        assert!(g.is_blocked(p));
    }

    #[test]
    fn test_successors_open_space() {
        let g = Grid3D::new(10, 10, 5);
        let s = g.successors(Waypoint3D::new(5, 5, 2));
        assert_eq!(s.len(), 6);
    }

    #[test]
    fn test_successors_corner() {
        let g = Grid3D::new(10, 10, 5);
        let s = g.successors(Waypoint3D::new(0, 0, 0));
        assert_eq!(s.len(), 3);
    }

    #[test]
    fn test_successors_exclude_blocked() {
        let mut g = Grid3D::new(10, 10, 5);
        g.block(Waypoint3D::new(6, 5, 2));
        let s = g.successors(Waypoint3D::new(5, 5, 2));
        assert_eq!(s.len(), 5);
        assert!(!s.contains(&Waypoint3D::new(6, 5, 2)));
    }
}
