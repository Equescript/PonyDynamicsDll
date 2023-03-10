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
pub struct WalkPridiction {}

pub struct WalkPlanner {
    velocity_correction_factor: f64,
    accel_correction_factor: f64,
    angular_velocity_correction_factor: f64,
    angular_accel_correction_factor: f64,
    next_planner: GaitType,
}

impl WalkPlanner {
    pub fn new() -> Self {
        Self {
            velocity_correction_factor: 0.0,
            accel_correction_factor: 0.0,
            angular_velocity_correction_factor: 0.0,
            angular_accel_correction_factor: 0.0,
            next_planner: GaitType::Stance
        }
    }
    pub fn initialize(velocity_correction_factor: f64, accel_correction_factor: f64, angular_velocity_correction_factor: f64, angular_accel_correction_factor: f64,) -> Self {
        Self { velocity_correction_factor, accel_correction_factor, angular_velocity_correction_factor, angular_accel_correction_factor, next_planner: GaitType::Walk }
    }
}

impl Planner for WalkPlanner {
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
            gait_data: GaitPridiction::Walk(WalkPridiction {  }),
            planner_type: GaitType::Walk,
        };
        Ok(())
    }
    fn next(&self) -> GaitType {
        self.next_planner
    }
}


 IntEnum!{
    pub enum WalkLegStatus {
        Stance = 0,
        InitialSwing,
        Swing,
        Contact,
    }
}

#[derive(Clone, Copy)]
pub struct WalkLegInfo {
    pub leg_status: WalkLegStatus,
}

#[derive(Clone, Copy)]
pub struct WalkController {
    legs_max_height: [Length; 4],
    legs_max_offset: [(Length, Length, Length); 4], // forward, backward, max_length
    contact_percentage: f64,
    stance_percentage: f64,
    swing_percentage: f64,
    period: f64, // unit: frame
    pattern_recovery_factor: f64,
}

impl WalkController {
    pub fn new() -> Self {
        Self {
            legs_max_height: [0.0; 4],
            legs_max_offset: [(0.0, 0.0, 0.0); 4],
            contact_percentage: 0.0,
            stance_percentage: 0.0,
            swing_percentage: 0.0,
            period: 0.0,
            pattern_recovery_factor: 0.0
        }
    }
    pub fn initialize(legs_max_height: [Length; 4], legs_max_offset: [(Length, Length); 4], contact_percentage: f64, pattern_recovery_factor: f64) -> Self {
        Self { legs_max_height, legs_max_offset: [
                (legs_max_offset[0].0, legs_max_offset[0].1, legs_max_offset[0].0 + legs_max_offset[0].1),
                (legs_max_offset[1].0, legs_max_offset[1].1, legs_max_offset[1].0 + legs_max_offset[1].1),
                (legs_max_offset[2].0, legs_max_offset[2].1, legs_max_offset[2].0 + legs_max_offset[2].1),
                (legs_max_offset[3].0, legs_max_offset[3].1, legs_max_offset[3].0 + legs_max_offset[3].1),
            ], contact_percentage,
            stance_percentage: 0.5 + contact_percentage / 2.0, swing_percentage: 0.5 - contact_percentage / 2.0,
            period: 0.0, pattern_recovery_factor
        }
    }
    fn step_length(&self, velocity: f64) -> Length {
        velocity * self.period
    }
    fn hoove_offset(&self, leg_type: LegType, step_length: Length) -> Length {
        let (max, min, sum) = self.legs_max_offset[leg_type as usize];
        ((max - min) / 2.0) * (step_length / sum)
    }
    fn stance_time(&self) -> usize {
        (self.period * ((1.0 + self.contact_percentage) / 2.0)).round() as usize
    }
    fn swing_time(&self) -> usize {
        (self.period * ((1.0 - self.contact_percentage) / 2.0)).round() as usize
    }
    fn solve_swing(&mut self, leg_info: &LegMotionInfo, upword_vec: Vec3) -> Result<Pose, ()> {
        let max_height = self.legs_max_height[leg_info.leg_type as usize];
        match leg_info.transform.rotation {
            Roatation::Angle(a) => {
                Ok(Pose::transform(&leg_info.start_pose, &Transform {
                    location: leg_info.transform.location * leg_info.factor + upword_vec * (max_height * math::parabola(leg_info.factor)),
                    rotation: Roatation::Angle(a * leg_info.factor),
                }))
            },
            Roatation::Matrix(_) => Err(())
        }
    }
    fn solve_leg(&mut self, leg_type: LegType, armature: &mut ArmatureKinematics, armature_rest: &ArmatureRest, ground: &Ground, pridictions: &mut Pridictions,
        (targets,  results,                  planners,      frame_current):
        (&Targets, &Vec<ArmatureKinematics>, &mut Planners, usize)
    ) -> Result<Pose, ()> {
        let phase_difference = |current: f64, target: f64| -> f64 {
            let difference = if current >= target {
                current - target
            } else {
                current + 1.0 - target
            };
            if difference > 0.5 {
                difference - 1.0
            } else {
                difference
            }
        };
        let phase_of = |leg_info: &LegMotionInfo| -> Result<f64, ()> {
            let phase = match match leg_info.gait_data {
                GaitLegInfo::Walk(t) => t.leg_status,
                GaitLegInfo::InitiateWalk(t) => t.leg_status,
                _ => return Err(())
            } {
                WalkLegStatus::InitialSwing | WalkLegStatus::Swing => {
                    leg_info.factor * self.swing_percentage
                },
                WalkLegStatus::Contact | WalkLegStatus::Stance => {
                    leg_info.factor * self.stance_percentage + self.swing_percentage
                },
            };
            if phase >= 1.0 {
                Ok(phase - 1.0)
            } else {
                Ok(phase)
            }
        };
        fn others(leg_type: LegType) -> [LegType; 3] {
            match leg_type {
                LegType::Foreleg_L => [LegType::Backleg_R, LegType::Foreleg_R, LegType::Backleg_L],
                LegType::Foreleg_R => [LegType::Backleg_L, LegType::Foreleg_L, LegType::Backleg_R],
                LegType::Backleg_L => [LegType::Foreleg_L, LegType::Backleg_R, LegType::Foreleg_R],
                LegType::Backleg_R => [LegType::Foreleg_R, LegType::Backleg_L, LegType::Foreleg_L],
            }
        }
        let FR_location: Location = armature.legs[LegType::Foreleg_R as usize].hoove.location;
        let FL_location: Location = armature.legs[LegType::Foreleg_L as usize].hoove.location;
        // let leg = &mut armature.legs[leg_type as usize];
        // let leg_info = &mut leg.leg_motion_info;
        // leg_info.factor has been refreshed
        // let mut projection_vec = |frame_current: usize, offset: usize| -> Result<Accel, ()> {Ok(targets[frame_current + offset].g_accel - pridictions.get_pridiction(offset, (targets,  results, planners, frame_current))?.center.accel)};
        match match {armature.legs[leg_type as usize].leg_motion_info.gait_data} {
            GaitLegInfo::Walk(t) => t.leg_status,
            GaitLegInfo::InitiateWalk(t) => {
                armature.legs[leg_type as usize].leg_motion_info.gait_data = GaitLegInfo::Walk(WalkLegInfo { leg_status: t.leg_status });
                t.leg_status
            },
            _ => return Err(())
        } {
            WalkLegStatus::Stance => {
                let leg_info = &mut armature.legs[leg_type as usize].leg_motion_info;
                if frame_current + 1 == leg_info.end_time {
                    leg_info.gait_data = GaitLegInfo::Walk(WalkLegInfo { leg_status: WalkLegStatus::InitialSwing });
                }
                Ok(leg_info.start_pose)
            },
            WalkLegStatus::InitialSwing => {
                let get_target_phase = || -> Result<f64, ()> {
                    let other_legs = others(leg_type);
                    let mut leg_phases = [
                        phase_of(&armature.legs[other_legs[0] as usize].leg_motion_info)?,
                        phase_of(&armature.legs[other_legs[1] as usize].leg_motion_info)?,
                        phase_of(&armature.legs[other_legs[2] as usize].leg_motion_info)?,
                    ];
                    loop {
                        if leg_phases[1] > leg_phases[0] {
                            leg_phases[1] = leg_phases[1] - 1.0;
                        } else {
                            break;
                        }
                    }
                    loop {
                        if leg_phases[2] > leg_phases[1] {
                            leg_phases[2] = leg_phases[2] - 1.0;
                        } else {
                            break;
                        }
                    }
                    // y = -0.25x + a   (-0.25x + a - y)^2 = (-0.25x - y)^2 + 2(-0.25x - y)a + a^2 => 2(-0.25x - y) + 2a
                    // variance = (-0.25 + a - leg_phases[0])^2 + (-0.5 + a - leg_phases[1])^2 + (-0.75 + a - leg_phases[2])^2
                    // d(variance) = 2( (-0.25 - leg_phases[0]) + (-0.5 - leg_phases[1]) + (-0.75 - leg_phases[2]) ) + 6a = 0
                    //             => a = -( (-0.25 - leg_phases[0]) + (-0.5 - leg_phases[1]) + (-0.75 - leg_phases[2]) ) / 3
                    let mut a = (leg_phases[0] + leg_phases[1] + leg_phases[2] + 1.5) / 3.0;
                    Ok(loop {
                        if a < 0.0 {
                            a = a + 1.0;
                        } else {
                            if a < 1.0 {
                                break a;
                            } else {
                                a = a - 1.0;
                            }
                        }
                    })
                };
                let target_phase = get_target_phase()?;
                let leg_info = &mut armature.legs[leg_type as usize].leg_motion_info;
                // let leg_info = &mut armature.legs[leg_type as usize].leg_motion_info;
                let current_phase = phase_of(&leg_info)?; // 0
                let difference = phase_difference(current_phase, target_phase);

                leg_info.gait_data = GaitLegInfo::Walk(WalkLegInfo { leg_status: WalkLegStatus::Swing });
                leg_info.start_time = frame_current;
                leg_info.factor = 0.0;
                leg_info.time_length = (self.swing_time() as i64 + (difference * self.period * self.pattern_recovery_factor).round() as i64) as usize;
                leg_info.end_time = leg_info.start_time + leg_info.end_time;
                // leg_info.start_pose = leg.hoove;
                match leg_info.leg_type {
                    LegType::Foreleg_L | LegType::Foreleg_R => {
                        // TODO: Calculate step by pridiction
                        let step_length = self.step_length(armature.center.velocity.magnitude());
                        let hoove_offset = self.hoove_offset(leg_info.leg_type, step_length) + step_length * 0.5;

                        let mut index = 0;
                        let mut distance = 0.0;
                        let pridiction = loop {
                            let p = pridictions.get_pridiction(index, (targets,  results, planners, frame_current))?;
                            index = index + 1;
                            distance = distance + p.center.velocity.magnitude();
                            if distance > hoove_offset {
                                break p;
                            }
                        };
                        // pridiction.center.basis_matrix = rotation_matrix * armature_rest.center.basis_matrix
                        let rotation_matrix: Mat3 = pridiction.center.basis_matrix * glm::inverse(&armature_rest.center.basis_matrix);
                        leg_info.target_pose.location = pridiction.center.location + rotation_matrix * armature_rest.leg_force_arm(leg_info.leg_type);
                        leg_info.target_pose.basis_matrix = pridiction.center.basis_matrix;

                    },
                    LegType::Backleg_L => {
                        let offset = (self.period * 0.5).round() as usize;
                        let pridiction = pridictions.get_pridiction(offset, (targets,  results, planners, frame_current))?;
                        let projection: Location = ground.intersection(armature.center.location, targets[frame_current + offset].g_accel - pridiction.center.accel);
                        leg_info.target_pose.location = projection * 2.0 - FR_location;
                        leg_info.target_pose.basis_matrix = pridiction.center.basis_matrix;
                    },
                    LegType::Backleg_R => {
                        let offset = (self.period * 0.5).round() as usize;
                        let pridiction = pridictions.get_pridiction(offset, (targets,  results, planners, frame_current))?;
                        let projection: Location = ground.intersection(armature.center.location, targets[frame_current + offset].g_accel - pridiction.center.accel);
                        leg_info.target_pose.location = projection * 2.0 - FL_location;
                        leg_info.target_pose.basis_matrix = pridiction.center.basis_matrix;
                    },
                }
                leg_info.transform = leg_info.start_pose.transform_with_rotation_angle_from(&leg_info.target_pose);
                Ok(leg_info.start_pose)
            },
            WalkLegStatus::Swing => {
                let leg_info = &mut armature.legs[leg_type as usize].leg_motion_info;
                if frame_current + 1 == leg_info.end_time {
                    leg_info.gait_data = GaitLegInfo::Walk(WalkLegInfo { leg_status: WalkLegStatus::Contact });
                }
                self.solve_swing(leg_info, Vec3::new(armature.center.basis_matrix.m11, armature.center.basis_matrix.m12, armature.center.basis_matrix.m13))
            }
            WalkLegStatus::Contact => {
                let get_target_phase = || -> Result<f64, ()> {
                    let other_legs = others(leg_type);
                    let mut leg_phases = [
                        phase_of(&armature.legs[other_legs[0] as usize].leg_motion_info)?,
                        phase_of(&armature.legs[other_legs[1] as usize].leg_motion_info)?,
                        phase_of(&armature.legs[other_legs[2] as usize].leg_motion_info)?,
                    ];
                    loop {
                        if leg_phases[1] > leg_phases[0] {
                            leg_phases[1] = leg_phases[1] - 1.0;
                        } else {
                            break;
                        }
                    }
                    loop {
                        if leg_phases[2] > leg_phases[1] {
                            leg_phases[2] = leg_phases[2] - 1.0;
                        } else {
                            break;
                        }
                    }
                    // y = -0.25x + a   (-0.25x + a - y)^2 = (-0.25x - y)^2 + 2(-0.25x - y)a + a^2 => 2(-0.25x - y) + 2a
                    // variance = (-0.25 + a - leg_phases[0])^2 + (-0.5 + a - leg_phases[1])^2 + (-0.75 + a - leg_phases[2])^2
                    // d(variance) = 2( (-0.25 - leg_phases[0]) + (-0.5 - leg_phases[1]) + (-0.75 - leg_phases[2]) ) + 6a = 0
                    //             => a = -( (-0.25 - leg_phases[0]) + (-0.5 - leg_phases[1]) + (-0.75 - leg_phases[2]) ) / 3
                    let mut a = (leg_phases[0] + leg_phases[1] + leg_phases[2] + 1.5) / 3.0;
                    Ok(loop {
                        if a < 0.0 {
                            a = a + 1.0;
                        } else {
                            if a < 1.0 {
                                break a;
                            } else {
                                a = a - 1.0;
                            }
                        }
                    })
                };
                let target_phase = get_target_phase()?;
                let leg_info = &mut armature.legs[leg_type as usize].leg_motion_info;
                // let leg_info = &mut armature.legs[leg_type as usize].leg_motion_info;
                let current_phase = phase_of(&leg_info)?; // 0.5(?)
                let difference = phase_difference(current_phase, target_phase);

                leg_info.gait_data = GaitLegInfo::Walk(WalkLegInfo { leg_status: WalkLegStatus::Stance });
                leg_info.start_time = frame_current;
                leg_info.factor = 0.0;
                leg_info.time_length = (self.stance_time() as i64 + (difference * self.period * self.pattern_recovery_factor).round() as i64) as usize;
                leg_info.end_time = leg_info.start_time + leg_info.end_time;
                leg_info.start_pose = leg_info.target_pose;
                Ok(leg_info.target_pose)
            },
        }
    }
}

impl Controller for WalkController {
    fn solve_legs(&mut self, armature: &mut ArmatureKinematics, armature_rest: &ArmatureRest, ground: &Ground, pridictions: &mut Pridictions,
        (targets,  results,                  planners,      frame_current):
        (&Targets, &Vec<ArmatureKinematics>, &mut Planners, usize)
    ) -> Result<(), ()> {

        armature.legs[0].leg_motion_info.set_factor(frame_current);
        armature.legs[1].leg_motion_info.set_factor(frame_current);
        armature.legs[2].leg_motion_info.set_factor(frame_current);
        armature.legs[3].leg_motion_info.set_factor(frame_current);
        armature.legs[0].hoove = self.solve_leg(LegType::Foreleg_L, armature, armature_rest, ground, pridictions, (targets,  results, planners, frame_current))?;
        armature.legs[1].hoove = self.solve_leg(LegType::Foreleg_R, armature, armature_rest, ground, pridictions, (targets,  results, planners, frame_current))?;
        armature.legs[2].hoove = self.solve_leg(LegType::Backleg_L, armature, armature_rest, ground, pridictions, (targets,  results, planners, frame_current))?;
        armature.legs[3].hoove = self.solve_leg(LegType::Backleg_R, armature, armature_rest, ground, pridictions, (targets,  results, planners, frame_current))?;
        Ok(())
    }
}

pub struct WalkIDsolver {

}

impl IDsolver for WalkIDsolver {
    fn solve(&self, armature_dynamics: &mut ArmatureDynamics, effect_of_force: EffectOfForce) -> Result<(), EffectOfForce> {
        Ok(())
    }
}