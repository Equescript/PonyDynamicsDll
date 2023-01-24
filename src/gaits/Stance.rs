use crate::utils::macros::{ImplCopy, IntEnum, ImplIndex};
use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::math::{self, Vec3, Mat3};
use super::{GaitType, GaitLegInfo, GaitPridiction};
use crate::predictor::{Planner, Pridiction};
use crate::armature::{LegType, Controller, UseControllerStructs, IDsolver, LegKinematics, LegMotionInfo};
use crate::kinematics::{Pose, KinematicsState, ForwardKinematicsSolver, Roatation, Transform};
UseControllerStructs!();

#[derive(Clone, Copy)]
pub struct StancePridiction {}

#[derive(Clone, Copy)]
pub struct StancePlanner {}

impl Planner for StancePlanner {
    fn calculate_next_of(&mut self, targets: &Targets, results: &Vec<ArmatureKinematics>, pridictions: &mut Pridictions, frame_current: usize, frame_offset: usize) -> Result<(), ()> {
        todo!()
    }
}

#[derive(Clone, Copy)]
pub struct StanceLegInfo {}

#[derive(Clone, Copy)]
pub struct StanceController {}

impl Controller for StanceController {
    fn solve_legs(&mut self, armature: &mut ArmatureKinematics, armature_rest: &ArmatureRest, ground: &Ground, pridictions: &mut Pridictions,
        (targets,  results,                  planners,      frame_current):
        (&Targets, &Vec<ArmatureKinematics>, &mut Planners, usize)
    ) -> Result<(), ()> {
        todo!()
    }
}

#[derive(Clone, Copy)]
pub struct StanceIDsolver {}

impl IDsolver for StanceIDsolver {
    fn solve(&self, armature_dynamics: &mut ArmatureDynamics, effect_of_force: EffectOfForce) -> Result<(), EffectOfForce> {
        Ok(())
    }
}