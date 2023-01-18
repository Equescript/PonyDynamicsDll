use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::utils::macros::IntEnum;
use crate::math::{Mat3, Mat4};
use crate::kinematics::{Pose, Transform, KinematicsState, Roatation};
use crate::dynamics::DynamicsState;
use crate::gaits::GaitType;

IntEnum!{
    pub enum LegType {
        Foreleg_L = 0,
        Foreleg_R,
        Backleg_L,
        Backleg_R,
    }
}

IntEnum!{
    pub enum LegStatus {
        Stance = 0,
        InitialSwing,
        Swing,
        Contact,
    }
}

pub trait Controller {
    fn calculate_motion(&self, leg_info: LegMotionInfo) -> Location;
    fn next(&self) -> Option<GaitType>;
    fn motion_solvers(&mut self) -> &mut [&mut Box<dyn LegMotionSolver>; 4];
}

pub trait LegMotionSolver {
    // allow the leg_info to influence the solver
    fn solve(&mut self, leg_info: &LegMotionInfo) -> Pose;
}

pub struct LegMotionInfo {
    pub leg: LegType,
    // leg_status: LegStatus,
    pub is_static: bool,
    pub start_time: usize,
    pub end_time: usize,
    pub time_length: usize,
    pub start_pose: Pose,
    pub target_pose: Pose,
    pub transform: Transform,
    pub factor: f64, // from 0 to 1, describe the moving process of hoove.
}

impl LegMotionInfo {
    fn calculate_motion(&mut self, motion_solver: &mut Box<dyn LegMotionSolver>, frame_current: usize) -> Pose {
        self.factor = (frame_current - self.start_time) as f64 / self.time_length as f64;
        if self.is_static {
            self.start_pose
        } else {
            motion_solver.solve(self)
        }
    }
}

pub struct LegKinematics {
    pub leg_type: LegType,
    // pub root_location: Location,
    pub leg_motion_info: LegMotionInfo,
    pub hoove: Pose,
}

pub struct LegDynamics {
    pub leg_type: LegType,
    pub forces: Vec<Force>,
}

pub struct LegRest {
    pub leg_type: LegType,
    pub root_location: Location,
    pub hoove: Pose,
}

pub struct ArmatureKinematics {
    pub root: Pose, // 延迟更新
    pub head: KinematicsState, // TODO: 加速度延迟更新
    pub legs: [LegKinematics; 4],
    pub center: KinematicsState,
    pub center_rotation: Mat3,
}

impl ArmatureKinematics {
    pub fn solve(&mut self, pridiction: Option<KinematicsState>, armature_rest: &ArmatureRest, motion_solvers: &mut [&mut Box<dyn LegMotionSolver>; 4], frame_current: usize) {
        match pridiction {
            Some(t) => { self.center = t; },
            None => { self.center.solve(); }
        }
        self.center_rotation = self.center.basis_matrix * glm::inverse(&armature_rest.center.basis_matrix);

        self.root.location = self.center.location + self.center_rotation * armature_rest.center_to_root;
        self.root.basis_matrix = self.center.basis_matrix;

        // self.head.solve();
        // TODO: Head Solver
        self.head.location = self.center.location + self.center_rotation * armature_rest.center_to_head;
        self.head.basis_matrix = self.center.basis_matrix;

        self.legs[0].hoove = self.legs[0].leg_motion_info.calculate_motion(motion_solvers[0], frame_current);
        self.legs[1].hoove = self.legs[1].leg_motion_info.calculate_motion(motion_solvers[1], frame_current);
        self.legs[2].hoove = self.legs[2].leg_motion_info.calculate_motion(motion_solvers[2], frame_current);
        self.legs[3].hoove = self.legs[3].leg_motion_info.calculate_motion(motion_solvers[3], frame_current);
    }
}

pub struct ArmatureDynamics {
    pub head: DynamicsState,
    pub center: DynamicsState,
    // pub legs: [LegDynamics; 4],
}

impl ArmatureDynamics {
    pub fn solve(&mut self, armature_kinematics: &ArmatureKinematics, accel: Accel, angular_accel: AngularAccel) {}
}

pub struct ArmatureRest {
    pub root: Pose,
    pub head: Pose,
    pub legs: [LegRest; 4],
    pub center: Pose,
    pub center_to_head: Location,
    pub center_to_root: Location,
}