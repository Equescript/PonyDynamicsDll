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

pub struct Output {
    pub root: Mat4,
    pub head: Mat4,
    pub legs: [Mat4; 4],
}

pub struct Context {
    pub targets: Targets,
    pub results: Vec<ArmatureKinematics>,
    pub pridictions: Pridictions,
    pub match_pridiction: bool,
    pub tick: Time, // s/f
    pub frame_current: usize,
    pub armature_rest: ArmatureRest, // const
    pub armature_kinematics: ArmatureKinematics,
    pub armature_dynamics: ArmatureDynamics,
    pub output: Output,
    pub gait_info: GaitInfo,
}

impl Context {
    fn calculate(&mut self, frame_current: usize) -> Result<(), ()> {
        self.frame_current = frame_current;
        self.gait_info.gait_type = self.pridictions[0].planner_type;

        let controller = &mut self.gait_info.controllers[self.gait_info.gait_type as usize];

        if self.match_pridiction {
            let pridiction = self.pridictions[0].center;
            self.armature_kinematics.solve(Some(pridiction), &self.armature_rest, controller, &mut self.pridictions,
                (&self.targets, &self.results, &mut self.gait_info.planners, self.frame_current))?;
        } else {
            self.armature_kinematics.solve(None, &self.armature_rest, controller, &mut self.pridictions,
                (&self.targets, &self.results, &mut self.gait_info.planners, self.frame_current))?;
            self.pridictions.refresh(self.armature_kinematics.center, (&self.targets, &self.results, &mut self.gait_info.planners, self.frame_current))?;
            // self.pridictions[0] is ready
        }

        let ID_solver = &mut self.gait_info.IDsolvers[self.gait_info.gait_type as usize];

        match self.armature_dynamics.solve(
            &self.armature_kinematics, ID_solver, self.targets[self.frame_current].g_accel,
            self.pridictions[0].center.into()
        ) {
            Ok(_) => { self.match_pridiction = true; },
            Err(e) => {
                self.match_pridiction = false;
                self.armature_kinematics.center.accel = e.accel;
                self.armature_kinematics.center.angular_accel = e.angular_accel;
            },
        }
        self.pridictions.pop_front((&self.targets, &self.results, &mut self.gait_info.planners, self.frame_current))?;
        // if self.match_pridiction then self.pridictions[0] is ready

        /* match controller.next() {
            Some(t) => { self.gait_info.gait_type = t; },
            None => {},
        } */
        self.results.push(self.armature_kinematics);
        Ok(())
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