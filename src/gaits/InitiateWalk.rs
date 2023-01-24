use crate::utils::macros::{IntEnum, ImplIndex};
use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::math::{self, Vec3};
use crate::predictor::{Planner, Pridiction};
use crate::armature::{LegType, LegMotionInfo, Controller, UseControllerStructs, IDsolver, LegKinematics};
UseControllerStructs!();
use crate::kinematics::{Pose, KinematicsState, ForwardKinematicsSolver};
use super::GaitLegInfo;
use super::{GaitType, GaitPridiction, Walk::{WalkLegStatus}};

#[derive(Clone, Copy)]
pub struct InitiateWalkPridiction {}

pub struct InitiateWalkPlanner {
    velocity_correction_factor: f64,
    accel_correction_factor: f64,
    angular_velocity_correction_factor: f64,
    angular_accel_correction_factor: f64,
    next_planner: GaitType,
}

impl InitiateWalkPlanner {
    pub fn new() -> Self {
        Self {
            velocity_correction_factor: 0.0,
            accel_correction_factor: 0.0,
            angular_velocity_correction_factor: 0.0,
            angular_accel_correction_factor: 0.0,
            next_planner: GaitType::Stance
        }
    }
    pub fn initialize(velocity_correction_factor: f64, accel_correction_factor: f64, angular_velocity_correction_factor: f64, angular_accel_correction_factor: f64) -> Self {
        Self { velocity_correction_factor, accel_correction_factor, angular_velocity_correction_factor, angular_accel_correction_factor, next_planner: GaitType::Walk }
    }
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

#[derive(Clone, Copy)]
pub struct InitiateWalkController {
    contact_percentage: f64, // walk步态两脚同时触地时间在周期中的占比
    preferred_leg: LegType, // const
    initiate: bool,
    legs_max_offset: [(Length, Length, Length); 4], // forward, backward, max_length
    // legs: [(WalkLegStatus); 4],
}

impl InitiateWalkController {
    pub fn new() -> Self {
        Self { contact_percentage: 0.0, preferred_leg: LegType::Foreleg_L, initiate: true, legs_max_offset: [(0.0, 0.0, 0.0); 4] }
    }
    pub fn initialize() -> Self {
        todo!()
    }
    fn hoove_offset(&self, leg_type: LegType, step_length: Length) -> Length {
        let (max, min, sum) = self.legs_max_offset[leg_type as usize];
        ((max - min) / 2.0) * (step_length / sum)
    }
}

impl Controller for InitiateWalkController {
    fn solve_legs(&mut self, armature: &mut ArmatureKinematics, armature_rest: &ArmatureRest, ground: &Ground, pridictions: &mut Pridictions,
        (targets,  results,                  planners,      frame_current):
        (&Targets, &Vec<ArmatureKinematics>, &mut Planners, usize)) -> Result<(), ()> {
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
            // armature.legs[init_sequence[3] as usize].leg_motion_info.gait_data
            let max_distance = self.legs_max_offset[init_sequence[3] as usize].1;
            let mut distance = 0.0;
            let mut next_distance = 0.0;
            let mut index = 0;
            let mut next_index = 0;
            for i in 1.. {
                let p = pridictions.get_pridiction(i, (targets,  results,                  planners,      frame_current))?;
                distance = next_distance;
                next_distance = distance + p.center.velocity.magnitude();
                if distance > max_distance {
                    index = i - 1;
                    next_index = i;
                    break;
                }
            }
            macro_rules! get_projection_vec {
                ($i:expr) => {
                    targets[$i].g_accel - pridictions[$i].center.accel
                };
            }
            let init_length = next_index - frame_current;
            let projection_vecs: [Vec3; 2] = [get_projection_vec!(index), get_projection_vec!(next_index)];
            // v = a1*v1 + a2*v2 + a3*v3
            // v = [v1, v2, v3] * [a1, a2, a3]
            armature.legs[init_sequence[0] as usize].leg_motion_info.gait_data = GaitLegInfo::InitiateWalk(InitiateWalkLegInfo { leg_status: WalkLegStatus::InitialSwing });
            armature.legs[init_sequence[0] as usize].leg_motion_info.start_pose = armature.legs[init_sequence[0] as usize].hoove;
            armature.legs[init_sequence[0] as usize].leg_motion_info.target_pose = armature.legs[init_sequence[0] as usize].hoove;
            armature.legs[init_sequence[1] as usize].leg_motion_info.gait_data = GaitLegInfo::InitiateWalk(InitiateWalkLegInfo { leg_status: WalkLegStatus::Stance });
            let leg = &mut armature.legs[init_sequence[1] as usize];
            leg.leg_motion_info.gait_data = GaitLegInfo::InitiateWalk(InitiateWalkLegInfo { leg_status: WalkLegStatus::Stance });
            leg.leg_motion_info.start_pose = leg.hoove;
            leg.leg_motion_info.start_time = frame_current;
            leg.leg_motion_info.time_length = ((init_length as f64) / 3.0).round() as usize;
            leg.leg_motion_info.end_time = frame_current + leg.leg_motion_info.time_length;

            let leg = &mut armature.legs[init_sequence[2] as usize];
            leg.leg_motion_info.gait_data = GaitLegInfo::InitiateWalk(InitiateWalkLegInfo { leg_status: WalkLegStatus::Stance });
            leg.leg_motion_info.start_pose = leg.hoove;
            leg.leg_motion_info.start_time = frame_current;
            leg.leg_motion_info.time_length = (((init_length * 2) as f64) / 3.0).round() as usize;
            leg.leg_motion_info.end_time = frame_current + leg.leg_motion_info.time_length;

            let leg = &mut armature.legs[init_sequence[3] as usize];
            leg.leg_motion_info.gait_data = GaitLegInfo::InitiateWalk(InitiateWalkLegInfo { leg_status: WalkLegStatus::Stance });
            leg.leg_motion_info.start_pose = leg.hoove;
            leg.leg_motion_info.start_time = frame_current;
            leg.leg_motion_info.time_length = init_length;
            leg.leg_motion_info.end_time = next_index;
            /* if leg_info.leg_type == init_sequence[0] {
                leg_info.gait_data = GaitLegInfo::InitiateWalk(InitiateWalkLegInfo { leg_status: WalkLegStatus::InitialSwing });
                leg_info.start_time = frame_current;
            } else if leg_info.leg_type == init_sequence[1] {
                leg_info.gait_data = GaitLegInfo::InitiateWalk(InitiateWalkLegInfo { leg_status: WalkLegStatus::Stance });
            } else if leg_info.leg_type == init_sequence[2] {
                leg_info.gait_data = GaitLegInfo::InitiateWalk(InitiateWalkLegInfo { leg_status: WalkLegStatus::Stance });
            } else if leg_info.leg_type == init_sequence[3] {
                leg_info.gait_data = GaitLegInfo::InitiateWalk(InitiateWalkLegInfo { leg_status: WalkLegStatus::Stance });
            } */
        }
        todo!()
    }
}

#[derive(Clone, Copy)]
pub struct InitiateWalkIDsolver {

}

impl IDsolver for InitiateWalkIDsolver {
    fn solve(&self, armature_dynamics: &mut ArmatureDynamics, effect_of_force: EffectOfForce) -> Result<(), EffectOfForce> {
        Ok(())
    }
}