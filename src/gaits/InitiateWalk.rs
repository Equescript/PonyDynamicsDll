use crate::utils::macros::{ImplCopy, IntEnum, ImplIndex};
use crate::targets::Targets;
use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::math;
use crate::predictor::{Planner, Pridictions, Pridiction};
use crate::armature::{LegType, LegMotionInfo, LegMotionSolver, Controller, IDsolver, ArmatureKinematics};
use crate::kinematics::{Pose, KinematicsState, ForwardKinematicsSolver};
use super::GaitLegInfo;
use super::{GaitType, GaitPridiction, StanceMotionSolver, Walk::{WalkLegStatus, WalkSwingMotionSolver}, Planners};

ImplCopy!{
    pub struct InitiateWalkPridiction {}
}

pub struct InitiateWalkPlanner {
    velocity_correction_factor: f64,
    accel_correction_factor: f64,
    angular_velocity_correction_factor: f64,
    angular_accel_correction_factor: f64,
    next_planner: GaitType,
}

impl Planner for InitiateWalkPlanner {
    fn calculate_next_of(&mut self, targets: &Targets, results: &Vec<ArmatureKinematics>, pridictions: &mut Pridictions, frame_current: usize, frame_offset: usize) -> Result<(), ()> {
        let velocity_correction = |location_offset: Location| -> Velocity {
            location_offset * self.velocity_correction_factor
        };
        let accel_correction = |velocity_offset: Velocity| -> Accel {
            velocity_offset * self.accel_correction_factor
        };
        let angular_velocity_correction = |rotation: Angle| -> AngularVelocity {
            rotation * self.angular_velocity_correction_factor
        };
        let angular_accel_correction = |angular_velocity_offset: AngularVelocity| -> AngularAccel {
            angular_velocity_offset * self.angular_accel_correction_factor
        };
        let accel = |location_offset: Location, velocity_offset: Velocity, target_accel: Accel| -> Accel {
            let velocity_correction: Velocity = velocity_correction(location_offset);
            accel_correction(velocity_correction + velocity_offset) + target_accel
        };
        let angular_accel = |rotation: Angle, angular_velocity_offset: AngularVelocity, target_angular_accel: AngularAccel| -> AngularAccel {
            let angular_velocity_correction: AngularVelocity = angular_velocity_correction(rotation);
            angular_accel_correction(angular_velocity_correction + angular_velocity_offset) + target_angular_accel
        };

        let p0 = &mut pridictions[frame_current];
        let t0 = &targets[frame_current + frame_current];
        let location_offset: Location = t0.center.location - p0.center.location;
        let velocity_offset: Velocity = t0.center.velocity - p0.center.velocity;
        p0.center.accel = accel(location_offset, velocity_offset, t0.center.accel);
        // t0.center.basis_matrix = rotation_matrix * p0.center.basis_matrix
        let rotation: Angle = math::angle_of_rotation_matrix(&(t0.center.basis_matrix * glm::inverse(&p0.center.basis_matrix)));
        let angular_velocity_offset: AngularVelocity = t0.center.angular_velocity - p0.center.angular_velocity;
        p0.center.angular_accel = angular_accel(rotation, angular_velocity_offset, t0.center.angular_accel);
        pridictions[frame_current + 1] = Pridiction {
            center: ForwardKinematicsSolver(&p0.center),
            gait_data: GaitPridiction::InitiateWalk(InitiateWalkPridiction {  }),
            planner_type: GaitType::InitiateWalk,
        };
        Ok(())
    }
    fn next(&self) -> GaitType {
        self.next_planner
    }
}

#[derive(Clone, Copy)]
pub struct InitiateWalkLegInfo {
    pub leg_status: WalkLegStatus,
}

pub struct InitiateWalkController {
    pub preferred_leg: LegType, // const
    pub initiate: bool,
    pub max_forward: f64,
    pub max_backward: f64,
    // pub legs: [(WalkLegStatus); 4],
}

impl Controller for InitiateWalkController {
    fn solve_leg(&mut self, leg_info: &mut LegMotionInfo, pridictions: &mut Pridictions,
        (targets,  results,                  planners,      frame_current):
        (&Targets, &Vec<ArmatureKinematics>, &mut Planners, usize)) -> Result<Pose, ()> {
        if self.initiate {
            let init_sequence = match self.preferred_leg {
                LegType::Foreleg_L => {
                    [LegType::Foreleg_L, LegType::Backleg_R, LegType::Foreleg_R, LegType::Backleg_L]
                },
                LegType::Foreleg_R => {
                    [LegType::Foreleg_R, LegType::Backleg_L, LegType::Foreleg_L, LegType::Backleg_R]
                },
                _ => {
                    [LegType::Foreleg_R, LegType::Backleg_L, LegType::Foreleg_L, LegType::Backleg_R]
                }
            };
            if leg_info.leg_type == init_sequence[0] {
                leg_info.gait_data = GaitLegInfo::InitiateWalk(InitiateWalkLegInfo { leg_status: WalkLegStatus::InitialSwing });
                leg_info.start_time = frame_current;
            } else if leg_info.leg_type == init_sequence[1] {
                leg_info.gait_data = GaitLegInfo::InitiateWalk(InitiateWalkLegInfo { leg_status: WalkLegStatus::Stance });
            } else if leg_info.leg_type == init_sequence[2] {
                leg_info.gait_data = GaitLegInfo::InitiateWalk(InitiateWalkLegInfo { leg_status: WalkLegStatus::Stance });
            } else if leg_info.leg_type == init_sequence[3] {
                leg_info.gait_data = GaitLegInfo::InitiateWalk(InitiateWalkLegInfo { leg_status: WalkLegStatus::Stance });
            }
        }
        todo!()
    }
}