use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::math;
use crate::math::{Mat3, Mat4};
use crate::targets::Targets;
use crate::predictor::Pridictions;
use crate::kinematics::{Pose, KinematicsState};
use crate::dynamics::EffectOfForce;
use crate::armature::{ArmatureKinematics, ArmatureDynamics, ArmatureRest};
use crate::gaits::GaitInfo;

struct Output {
    root: Mat4,
    head: Mat4,
    legs: [Mat4; 4],
}

struct Context {
    targets: Targets,
    results: Vec<KinematicsState>,
    pridictions: Pridictions,
    match_pridiction: bool,
    tick: Time,
    frame_current: usize,
    armature_rest: ArmatureRest, // const
    armature_kinematics: ArmatureKinematics,
    armature_dynamics: ArmatureDynamics,
    output: Output,
    gait_info: GaitInfo,
}

impl Context {
    fn calculate(&mut self) {

        let planner = self.gait_info.planners.get_planner_by_velocity(&self.targets, &self.results, self.frame_current);
        // self.pridictions.get_pridiction(1, &self.targets, planner, self.frame_current);
        // self.pridictions.set_pridiction_to(index, targets, planners, frame_current)

        self.pridictions.pop_front();

        let controller = &mut self.gait_info.controllers[self.gait_info.gait_type as usize];

        if self.match_pridiction {
            self.armature_kinematics.center = self.pridictions.get_pridiction(0, &self.targets, planner, self.frame_current).center;
        } else {
            self.armature_kinematics.solve(None, &self.armature_rest, controller.motion_solvers(), self.frame_current);
            self.pridictions.refresh(self.armature_kinematics.center, self.gait_info.gait_type);
        }

        let pridiction = self.pridictions.get_pridiction(1, &self.targets, planner, self.frame_current);
        let ID_solver = &mut self.gait_info.IDsolvers[self.gait_info.gait_type as usize];

        match self.armature_dynamics.solve(
            &self.armature_kinematics, ID_solver, self.targets[self.frame_current].g_accel,
            EffectOfForce {
                accel: pridiction.center.accel,
                angular_accel: pridiction.center.angular_accel
            }
        ) {
            Ok(_) => { self.match_pridiction = true; },
            Err(e) => {
                self.match_pridiction = false;
                self.armature_kinematics.center.accel = e.accel;
                self.armature_kinematics.center.angular_accel = e.angular_accel;
            },
        }
        match controller.next() {
            Some(t) => { self.gait_info.gait_type = t; },
            None => {},
        }
        self.calculate_output()
    }
    fn calculate_output(&mut self) {
        self.output.root = math::make_transformation_matrix(self.armature_kinematics.root.location - self.armature_rest.root.location, self.armature_kinematics.center_rotation);
        self.output.head = Pose::from(self.armature_kinematics.head).transformation_matrix_from(&self.armature_rest.head);
        self.output.legs[0] = self.armature_kinematics.legs[0].hoove.transformation_matrix_from(&self.armature_rest.legs[0].hoove);
        self.output.legs[1] = self.armature_kinematics.legs[1].hoove.transformation_matrix_from(&self.armature_rest.legs[1].hoove);
        self.output.legs[2] = self.armature_kinematics.legs[2].hoove.transformation_matrix_from(&self.armature_rest.legs[2].hoove);
        self.output.legs[3] = self.armature_kinematics.legs[3].hoove.transformation_matrix_from(&self.armature_rest.legs[3].hoove);
    }
}