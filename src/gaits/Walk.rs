use crate::utils::macros::{ImplCopy, IntEnum, ImplIndex};
use crate::targets::Targets;
use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::math::{self, Vec3};
use super::{GaitType};
use crate::predictor::{Planner, Pridictions, Pridiction};
use crate::armature::{Controller, IDsolver, ArmatureKinematics, LegMotionInfo, LegMotionSolver};
use crate::kinematics::{Pose, KinematicsState, ForwardKinematicsSolver, Roatation, Transform};

pub struct WalkPlanner {
    velocity_correction_factor: f64,
    accel_correction_factor: f64,
    angular_velocity_correction_factor: f64,
    angular_accel_correction_factor: f64,
}

/* impl Planner for WalkPlanner {
    fn velocity_correction(&mut self, location_offset: Location) -> Velocity {
        location_offset * self.velocity_correction_factor
    }
    fn accel_correction(&mut self, velocity_offset: Velocity) -> Accel {
        velocity_offset * self.accel_correction_factor
    }
    fn angular_velocity_correction(&mut self, rotation: Angle) -> AngularVelocity {
        rotation * self.angular_velocity_correction_factor
    }
    fn angular_accel_correction(&mut self, angular_velocity_offset: AngularVelocity) -> AngularAccel {
        angular_velocity_offset * self.angular_accel_correction_factor
    }
}
 */

 IntEnum!{
    pub enum WalkLegStatus {
        Stance = 0,
        InitialSwing,
        Swing,
        Contact,
    }
}

// impl Controller for Walk {

// }

#[derive(Clone, Copy)]
pub struct WalkLegInfo {

}

pub struct WalkSwingMotionSolver {
    upword_vec: Vec3,
    max_height: f64,
}

impl LegMotionSolver for WalkSwingMotionSolver {
    fn solve(&mut self, leg_info: &LegMotionInfo) -> Result<Pose, ()> {
        match leg_info.transform.rotation {
            Roatation::Angle(a) => {
                Ok(Pose::transform(&leg_info.start_pose, &Transform {
                    location: leg_info.transform.location * leg_info.factor + self.upword_vec * (self.max_height * math::parabola(leg_info.factor)),
                    rotation: Roatation::Angle(a * leg_info.factor),
                }))
            },
            Roatation::Matrix(_) => Err(())
        }
    }
}

ImplCopy!{
    pub struct WalkPridiction {}
}