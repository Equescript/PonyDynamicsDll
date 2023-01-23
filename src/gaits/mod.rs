pub mod Walk;
pub mod InitiateWalk;
use crate::targets::Targets;
use crate::utils::macros::{ImplCopy, IntEnum, ImplIndex};
use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::predictor::{Planner, Pridictions, Pridiction};
use crate::armature::{Controller, ArmatureKinematics};
use crate::armature::IDsolver;
use crate::kinematics::{Pose, KinematicsState, ForwardKinematicsSolver};
use crate::dynamics::EffectOfForce;

IntEnum!{
    pub enum GaitType {
        Stance = 0,
        Walk,
        Trot,
        Gallop,
        InitiateWalk,
        InitiateTrot,
        InitiateGallop,
        TerminateWalk,
        TerminateTrot,
        TerminateGallop,
        WalkToTrot,
        TrotToGallop,
        GallopToTrot,
        TrotToWalk,
    }
}

#[derive(Clone, Copy)]
pub enum GaitPridiction {
    Stance(StancePridiction),
    Walk(Walk::WalkPridiction),
    Trot(TrotPridiction),
    Gallop(GallopPridiction),
    InitiateWalk(InitiateWalk::InitiateWalkPridiction),
    InitiateTrot(InitiateTrotPridiction),
    InitiateGallop(InitiateGallopPridiction),
    TerminateWalk(TerminateWalkPridiction),
    // TerminateTrot(TerminateTrotPridiction),
    // TerminateGallop(TerminateGallopPridiction),
    // WalkToTrot(WalkToTrotPridiction),
    // TrotToGallop(TrotToGallopPridiction),
    // GallopToTrot(GallopToTrotPridiction),
    // TrotToWalk(TrotToWalkPridiction),
}

#[derive(Clone, Copy)]
pub enum GaitLegInfo {
    Stance(StanceLegInfo),
    Walk(Walk::WalkLegInfo),
    Trot(TrotLegInfo),
    Gallop(GallopLegInfo),
    InitiateWalk(InitiateWalk::InitiateWalkLegInfo),
    InitiateTrot(InitiateTrotLegInfo),
    InitiateGallop(InitiateGallopLegInfo),
    TerminateWalk(TerminateWalkLegInfo),
    // TerminateTrot(TerminateTrotPridiction),
    // TerminateGallop(TerminateGallopPridiction),
    // WalkToTrot(WalkToTrotPridiction),
    // TrotToGallop(TrotToGallopPridiction),
    // GallopToTrot(GallopToTrotPridiction),
    // TrotToWalk(TrotToWalkPridiction),
}

impl GaitLegInfo {
    pub fn new() -> Self {
        Self::Stance(StanceLegInfo {  })
    }
}

/* macro_rules! GaitsUnwarp {
    ($gait:expr) => {
        {
            use crate::gaits::Gaits;
            match $gait {
                Gaits::Stance(t) => t,
                Gaits::Walk(t) => t,
                Gaits::Trot(t) => t,
                Gaits::Gallop(t) => t,
                Gaits::InitiateWalk(t) => t,
                Gaits::InitiateTrot(t) => t,
                Gaits::InitiateGallop(t) => t,
                Gaits::TerminateWalk(t) => t,
                Gaits::TerminateTrot(t) => t,
                Gaits::TerminateGallop(t) => t,
                Gaits::WalkToTrot(t) => t,
                Gaits::TrotToGallop(t) => t,
                Gaits::GallopToTrot(t) => t,
                Gaits::TrotToWalk(t) => t,
            }
        }
    };
} */

pub struct Planners {
    data: [Box<dyn Planner>; 2],
}

impl Planners {
    fn new() -> Self {
        Self { data: [
            Box::new(InitiateWalk::InitiateWalkPlanner::new()),
            Box::new(Walk::WalkPlanner::new()),
        ] }
    }
}

ImplIndex!(Planners, Box<dyn Planner>);

pub struct Controllers {
    data: [Box<dyn Controller>; 2],
}

impl Controllers {
    fn new() -> Self {
        Self { data: [
            Box::new(InitiateWalk::InitiateWalkController::new()),
            Box::new(Walk::WalkController::new()),
        ] }
    }
}

ImplIndex!(Controllers, Box<dyn Controller>);

pub struct IDsolvers {
    data: [Box<dyn IDsolver>; 2],
}

impl IDsolvers {
    fn new() -> Self {
        Self { data: [
            Box::new(InitiateWalk::InitiateWalkIDsolver{}),
            Box::new(Walk::WalkIDsolver{}),
        ] }
    }
}

ImplIndex!(IDsolvers, Box<dyn IDsolver>);

pub struct GaitInfo {
    pub gait_type: GaitType,
    // pub gaits: [Gaits; 14],
    // planners: [Box<dyn Planner>; 14],
    pub planners: Planners,
    pub controllers: Controllers,
    pub IDsolvers: IDsolvers,
}

impl GaitInfo {
    pub fn new() -> Self {
        Self { gait_type: GaitType::Stance, planners: Planners::new(), controllers: Controllers::new(), IDsolvers: IDsolvers::new() }
    }
}


macro_rules! GaitToDo {
    ($name1:ident, $name2:ident, $name3:ident, $name4:ident) => {
        #[derive(Clone, Copy)]
        pub struct $name1 {

        }

        /* impl Planner for $name1 {
            fn velocity_correction(&mut self, location_offset: Location) -> Velocity {
                todo!()
            }
            fn accel_correction(&mut self, velocity_offset: Velocity) -> Accel {
                todo!()
            }
            fn angular_velocity_correction(&mut self, rotation: Angle) -> AngularVelocity {
                todo!()
            }
            fn angular_accel_correction(&mut self, angular_velocity_offset: AngularVelocity) -> AngularAccel {
                todo!()
            }
        } */
        #[derive(Clone, Copy)]
        pub struct $name2 {

        }

        // impl Controller for $name2 {

        // }
        #[derive(Clone, Copy)]
        pub struct $name3 {

        }
        #[derive(Clone, Copy)]
        pub struct $name4 {

        }
    };
}

GaitToDo!(StancePlanner, StanceController, StancePridiction, StanceLegInfo);
GaitToDo!(TrotPlanner, TrotController, TrotPridiction, TrotLegInfo);
GaitToDo!(GallopPlanner, GallopController, GallopPridiction, GallopLegInfo);
GaitToDo!(InitiateTrotPlanner, InitiateTrotController, InitiateTrotPridiction, InitiateTrotLegInfo);
GaitToDo!(InitiateGallopPlanner, InitiateGallopController, InitiateGallopPridiction, InitiateGallopLegInfo);
GaitToDo!(TerminateWalkPlanner, TerminateWalkController, TerminateWalkPridiction, TerminateWalkLegInfo);
GaitToDo!(TerminateTrotPlanner, TerminateTrotController, TerminateTrotPridiction, TerminateTrotLegInfo);
GaitToDo!(TerminateGallopPlanner, TerminateGallopController, TerminateGallopPridiction, TerminateGallopLegInfo);
GaitToDo!(WalkToTrotPlanner, WalkToTrotController, WalkToTrotPridiction, WalkToTrotLegInfo);
GaitToDo!(TrotToGallopPlanner, TrotToGallopController, TrotToGallopPridiction, TrotToGallopLegInfo);
GaitToDo!(GallopToTrotPlanner, GallopToTrotController, GallopToTrotPridiction, GallopToTrotLegInfo);
GaitToDo!(TrotToWalkPlanner, TrotToWalkController, TrotToWalkPridiction, TrotToWalkLegInfo);
