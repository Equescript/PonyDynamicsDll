use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::math;
use crate::math::{Vec3, Mat3, Mat4};
use crate::targets::Targets;
use crate::predictor::Pridictions;
use crate::kinematics::{Pose, KinematicsState};
use crate::dynamics::EffectOfForce;
use crate::armature::{ArmatureKinematics, ArmatureDynamics, ArmatureRest};
use crate::gaits::GaitInfo;

pub struct Ground {
    pub normal: Vec3, // normalized
    pub base_point: Location, // one of the point on the plane
    pub frictional_coeff: FrictionalCoeff,
}

impl Ground {
    pub fn initialize(normal: Vec3, base_point: Location, frictional_coeff: FrictionalCoeff) -> Self {
        Self { normal, base_point, frictional_coeff }
    }
    pub fn intersection(&self, o: Vec3, d: Vec3) -> Location {
        math::intersection(o, d, self.base_point, self.normal)
    }
}

pub struct Output {
    pub root: Mat4,
    pub head: Mat4,
    pub legs: [Mat4; 4],
}

impl Output {
    pub fn new() -> Self {
        Self { root: Mat4::zeros(), head: Mat4::zeros(), legs: [Mat4::zeros(); 4] }
    }
}

pub struct Context {
    pub targets: Targets,
    pub results: Vec<ArmatureKinematics>,
    pub pridictions: Pridictions,
    pub match_pridiction: bool,
    pub tick: Time, // s/f
    pub frame_current: usize,
    pub ground: Ground,
    pub armature_rest: ArmatureRest, // const
    pub armature_kinematics: ArmatureKinematics,
    pub armature_dynamics: ArmatureDynamics,
    pub output: Output,
    pub gait_info: GaitInfo,
}

impl Context {
    fn initialize(size: usize, centripetal_force_factor: f64, tick: Time, ground_normal: Vec3, base_point: Location, frictional_coeff: FrictionalCoeff,
        root: Pose, head: Pose, legs: [(Location, Pose); 4], center: Pose,
        head_mass: Mass, head_rotational_inertia: RotationalInertia, center_mass: Mass, center_rotational_inertia: RotationalInertia
    ) -> Self {
        Self {
            targets: Targets::initialize(size, centripetal_force_factor),
            results: Vec::with_capacity(size + 10),
            pridictions: Pridictions::new(),
            match_pridiction: false,
            tick: tick,
            frame_current: 0,
            ground: Ground::initialize(ground_normal, base_point, frictional_coeff),
            armature_rest: ArmatureRest::initialize(root, head, legs, center),
            armature_kinematics: ArmatureKinematics::new(), // uninitialized
            armature_dynamics: ArmatureDynamics::initialize(head_mass, head_rotational_inertia, center_mass, center_rotational_inertia),
            output: Output::new(), // uninitialized
            gait_info: GaitInfo::new() // uninitialized
        }
    }
    fn calculate(&mut self, frame_current: usize) -> Result<(), ()> {
        self.frame_current = frame_current;
        self.gait_info.gait_type = self.pridictions[0].planner_type;

        let controller = &mut self.gait_info.controllers[self.gait_info.gait_type as usize];

        if self.match_pridiction {
            self.armature_kinematics.solve(Some(self.pridictions[0].center), &self.armature_rest, controller, &self.ground, &mut self.pridictions,
                (&self.targets, &self.results, &mut self.gait_info.planners, self.frame_current))?;
        } else {
            self.armature_kinematics.solve(None, &self.armature_rest, controller, &self.ground, &mut self.pridictions,
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