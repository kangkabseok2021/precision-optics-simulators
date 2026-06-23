#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Waypoint3D {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl Waypoint3D {
    pub fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }

    pub fn manhattan(&self, other: &Self) -> u32 {
        (self.x - other.x).unsigned_abs()
            + (self.y - other.y).unsigned_abs()
            + (self.z - other.z).unsigned_abs()
    }
}

#[derive(Debug, Clone)]
pub struct Agent {
    pub id: usize,
    pub start: Waypoint3D,
    pub goal: Waypoint3D,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Constraint {
    pub agent: usize,
    pub point: Waypoint3D,
    pub time: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EdgeConstraint {
    pub agent: usize,
    pub from: Waypoint3D,
    pub to: Waypoint3D,
    pub time: u32,
}

#[derive(Debug)]
pub enum CbsError {
    NoPathFound(usize),
    Timeout,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_manhattan_same_point() {
        let p = Waypoint3D::new(3, 4, 5);
        assert_eq!(p.manhattan(&p), 0);
    }

    #[test]
    fn test_manhattan_axis_aligned() {
        let a = Waypoint3D::new(0, 0, 0);
        let b = Waypoint3D::new(3, 4, 0);
        assert_eq!(a.manhattan(&b), 7);
    }

    #[test]
    fn test_manhattan_3d() {
        let a = Waypoint3D::new(0, 0, 0);
        let b = Waypoint3D::new(1, 1, 1);
        assert_eq!(a.manhattan(&b), 3);
    }

    #[test]
    fn test_waypoint_equality() {
        assert_eq!(Waypoint3D::new(1, 2, 3), Waypoint3D::new(1, 2, 3));
        assert_ne!(Waypoint3D::new(1, 2, 3), Waypoint3D::new(1, 2, 4));
    }
}
