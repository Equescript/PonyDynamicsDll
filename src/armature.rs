use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::utils::macros::{ImplCopy, IntEnum};
use crate::math::{Mat3, Mat4};
use crate::kinematics::{Pose, Transform, KinematicsState, Roatation};
use crate::dynamics::{EffectOfForce, MomentOfForce, DynamicsState, ForwardDynamicsSolver};
use crate::gaits::{GaitType, GaitLegInfo, Planners};
use crate::predictor::{Pridictions};
use crate::targets::Targets;
use crate::context::Ground;

IntEnum!{
    #[derive(PartialEq, Eq)]
    pub enum LegType {
        Foreleg_L = 0,
        Foreleg_R,
        Backleg_L,
        Backleg_R,
    }
}

macro_rules! UseControllerStructs {
    () => {
        use crate::armature::{ArmatureKinematics, ArmatureDynamics, ArmatureRest};
        use crate::dynamics::EffectOfForce;
        use crate::predictor::Pridictions;
        use crate::gaits::Planners;
        use crate::targets::Targets;
        use crate::context::Ground;
    };
}

pub(crate) use UseControllerStructs;

pub trait Controller {
    // fn period(&self) -> usize;
    // fn calculate_motion(&self, leg_info: LegMotionInfo) -> Location;
    // fn motion_solvers(&mut self) -> &mut [&mut Box<dyn LegMotionSolver>; 4];
    fn solve_legs(&mut self, armature: &mut ArmatureKinematics, armature_rest: &ArmatureRest, ground: &Ground, pridictions: &mut Pridictions,
        (targets,  results,                  planners,      frame_current):
        (&Targets, &Vec<ArmatureKinematics>, &mut Planners, usize)
    ) -> Result<(), ()> {
        Err(())
    }
}

#[derive(Clone, Copy)]
pub struct LegMotionInfo {
    pub leg_type: LegType,
    pub gait_data: GaitLegInfo,
    // leg_status: LegStatus,
    // pub is_stance: bool,
    pub start_time: usize,
    pub end_time: usize,
    pub time_length: usize,
    pub start_pose: Pose,
    pub target_pose: Pose,
    pub transform: Transform,
    pub factor: f64, // from 0 to 1, describe the moving process of hoove.
}


impl LegMotionInfo {
    pub fn new() -> Self {
        Self {
            leg_type: LegType::Foreleg_L,
            gait_data: GaitLegInfo::new(),
            // is_stance: true,
            start_time: 0,
            end_time: 0,
            time_length: 0,
            start_pose: Pose::new(),
            target_pose: Pose::new(),
            transform: Transform::new(),
            factor: 0.0
        }
    }
    pub fn set_factor(&mut self, frame_current: usize) {
        self.factor = (frame_current - self.start_time) as f64 / self.time_length as f64;
    }
    /* fn calculate_motion(&mut self, controller: &mut Box<dyn Controller>,/* motion_solver: &mut Box<dyn LegMotionSolver>, */ pridictions: &mut Pridictions,
        (targets,  results,                  planners,      frame_current):
        (&Targets, &Vec<ArmatureKinematics>, &mut Planners, usize)) -> Result<Pose, ()> {
        self.factor = (frame_current - self.start_time) as f64 / self.time_length as f64;
        // if self.is_stance {
            // self.start_pose
        // } else {
            // motion_solver.solve(self)
        // }
        controller.solve_leg(self, pridictions, (targets, results, planners, frame_current))
    } */
}
ImplCopy!{
    pub struct LegKinematics {
        pub leg_type: LegType,
        pub leg_motion_info: LegMotionInfo,
        pub hoove: Pose,
    }
}

impl LegKinematics {
    pub fn new(leg_type: LegType) -> Self {
        Self { leg_type, leg_motion_info: LegMotionInfo::new(), hoove: Pose::new() }
    }
}

ImplCopy!{
    pub struct ArmatureKinematics {
        pub root: Pose, // ????????????
        pub head: KinematicsState, // TODO: ?????????????????????
        pub legs: [LegKinematics; 4],
        pub center: KinematicsState,
        pub center_rotation: Mat3,
    }
}

impl ArmatureKinematics {
    pub fn new() -> Self {
        ArmatureKinematics {
            root: Pose::new(),
            head: KinematicsState::new(),
            legs: [
                LegKinematics::new(LegType::Foreleg_L),
                LegKinematics::new(LegType::Foreleg_R),
                LegKinematics::new(LegType::Backleg_L),
                LegKinematics::new(LegType::Backleg_R),
            ],
            center: KinematicsState::new(),
            center_rotation: Mat3::zeros()
        }
    }
    pub fn solve(&mut self, pridiction: Option<KinematicsState>, armature_rest: &ArmatureRest,
        controller: &mut Box<dyn Controller>, ground: &Ground, pridictions: &mut Pridictions,
        (targets,  results,                  planners,      frame_current):
        (&Targets, &Vec<ArmatureKinematics>, &mut Planners, usize)
                    ) -> Result<(), ()> {
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

        controller.solve_legs(self, armature_rest, ground, pridictions, (targets, results, planners, frame_current))?;
        Ok(())
    }
    pub fn leg_force_arm(&self, leg_type: LegType) -> Location {
        self.legs[leg_type as usize].hoove.location - self.center.location
    }
    pub fn leg_force_arms(&self) -> [Location; 4] {
        [
            self.legs[0].hoove.location - self.center.location,
            self.legs[1].hoove.location - self.center.location,
            self.legs[2].hoove.location - self.center.location,
            self.legs[3].hoove.location - self.center.location,
        ]
    }
}

pub struct LegDynamics {
    pub leg_type: LegType,
    pub forces: Vec<Force>,
    pub joint_force: MomentOfForce,
}

impl LegDynamics {
    pub fn new(leg_type: LegType) -> Self {
        Self { leg_type, forces: Vec::new(), joint_force: MomentOfForce { force: Force::zeros(), arm: Location::zeros() } }
    }
}

pub trait IDsolver {
    fn solve(&self, armature_dynamics: &mut ArmatureDynamics, effect_of_force: EffectOfForce) -> Result<(), EffectOfForce> {
        Ok(())
    }
}

pub struct ArmatureDynamics {
    pub head: DynamicsState,
    pub center: DynamicsState, // center.forces[0] is gravity
    pub legs: [LegDynamics; 4],
}

impl ArmatureDynamics {
    pub fn new() -> Self {
        Self { head: DynamicsState::new(), center: DynamicsState::new(), legs: [
            LegDynamics::new(LegType::Foreleg_L),
            LegDynamics::new(LegType::Foreleg_R),
            LegDynamics::new(LegType::Backleg_L),
            LegDynamics::new(LegType::Backleg_R),
        ], }
    }
    pub fn initialize(head_mass: Mass, head_rotational_inertia: RotationalInertia, center_mass: Mass, center_rotational_inertia: RotationalInertia) -> Self {
        Self {
            head: DynamicsState::initialize(head_mass, head_rotational_inertia),
            center: DynamicsState::initialize(center_mass, center_rotational_inertia),
            legs: [
                LegDynamics::new(LegType::Foreleg_L),
                LegDynamics::new(LegType::Foreleg_R),
                LegDynamics::new(LegType::Backleg_L),
                LegDynamics::new(LegType::Backleg_R),
            ],
        }
    }
    pub fn solve(&mut self, armature_kinematics: &ArmatureKinematics, ID_solver: &Box<dyn IDsolver>, g_accel: Accel, target_effect: EffectOfForce) -> Result<(), EffectOfForce> {
        self.legs[0].joint_force.arm = armature_kinematics.leg_force_arm(LegType::Foreleg_L);
        self.legs[1].joint_force.arm = armature_kinematics.leg_force_arm(LegType::Foreleg_R);
        self.legs[2].joint_force.arm = armature_kinematics.leg_force_arm(LegType::Backleg_L);
        self.legs[3].joint_force.arm = armature_kinematics.leg_force_arm(LegType::Backleg_R);
        // TODO: Head
        self.center.forces[0].force = g_accel * self.center.mass;
        let current_effect = ForwardDynamicsSolver(&self.center);
        // target_effect = self_effect + current_effect;
        ID_solver.solve(self, target_effect - current_effect)
    }
}

pub struct LegRest {
    pub leg_type: LegType,
    pub root_location: Location,
    pub hoove: Pose,
}

impl LegRest {
    pub fn new(leg_type: LegType) -> Self {
        Self { leg_type, root_location: Location::zeros(), hoove: Pose::new() }
    }
}

pub struct ArmatureRest {
    pub root: Pose,
    pub head: Pose,
    pub legs: [LegRest; 4],
    pub center: Pose,
    pub center_to_head: Location,
    pub center_to_root: Location,
}

impl ArmatureRest {
    pub fn new() -> Self {
        Self { root: Pose::new(), head: Pose::new(), legs: [
            LegRest::new(LegType::Foreleg_L),
            LegRest::new(LegType::Foreleg_R),
            LegRest::new(LegType::Backleg_L),
            LegRest::new(LegType::Backleg_R),
            ], center: Pose::new(), center_to_head: Location::zeros(), center_to_root: Location::zeros()
        }
    }
    pub fn initialize(root: Pose, head: Pose, legs: [(Location, Pose); 4], center: Pose) -> Self {
        Self { root, head, legs: [
                LegRest { leg_type: LegType::Foreleg_L, root_location: legs[0].0, hoove: legs[0].1 },
                LegRest { leg_type: LegType::Foreleg_R, root_location: legs[1].0, hoove: legs[1].1 },
                LegRest { leg_type: LegType::Backleg_L, root_location: legs[2].0, hoove: legs[2].1 },
                LegRest { leg_type: LegType::Backleg_R, root_location: legs[3].0, hoove: legs[3].1 },
            ],
            center,
            center_to_head: head.location - center.location,
            center_to_root: root.location - center.location
        }
    }
    pub fn leg_force_arm(&self, leg_type: LegType) -> Location {
        self.legs[leg_type as usize].hoove.location - self.center.location
    }
}