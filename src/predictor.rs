use std::collections::VecDeque;
use std::ops::{Index, IndexMut};
use crate::math;
use crate::math::{Mat3};
use crate::targets::Targets;
use crate::utils::macros::ImplIndex;
use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::kinematics::{KinematicsState, ForwardKinematicsSolver};
use crate::gaits::{GaitType, Planners};

// pub struct Pridiction {
    // pub center: KinematicsState,
    // pub projection: Location,
// }

// impl Pridiction {
//     fn new() -> Self {
//         Self { center: KinematicsState::new(), projection: Location::zeros() }
//     }
// }

pub struct Pridiction {
    pub center: KinematicsState,
    pub planner: GaitType,
}

impl Pridiction {
    pub fn new() -> Self {
        Pridiction { center: KinematicsState::new(), planner: GaitType::Stance }
    }
}

pub struct Pridictions {
    length: usize,
    data: VecDeque<Pridiction>,
}

impl Pridictions {
    pub fn new() -> Self {
        Pridictions { length: 0, data: VecDeque::new() }
    }
    // pub fn renew_current(&mut self, center_current: &CenterTarget) {
    //     self.data.clear();
    //     // self.predictions[0].center = *center_current;
    //     self.data.push_back(Pridiction { center: *center_current, projection: Location::zeros() })
    // }
    pub fn pop_front(&mut self) -> Option<Pridiction> {
        self.data.pop_front()
    }
    pub fn refresh(&mut self, k: KinematicsState, planner: GaitType) {
        /* self.data.pop_front();
        if self.len() != 0 {
            self.data[0] = k;
        } else {
            self.data.push_back(k)
        } */
        self.data.clear();
        self.data.push_back(Pridiction { center: k, planner });
    }
    pub fn len(&self) -> usize {
        self.data.len()
    }
    pub fn get(&self, index: usize) -> Option<&Pridiction> {
        self.data.get(index)
    }
    pub fn get_mut(&mut self, index: usize) -> Option<&mut Pridiction> {
        self.data.get_mut(index)
    }
    fn set_pridiction_by_frame_offset(&mut self, targets: &Targets, planner: &mut Box<dyn Planner>, frame_current: usize, frame_offset: usize) {
        let t0 = &targets[frame_current+frame_offset].center;
        // let p1 = &mut ;
        self[frame_offset].center = ForwardKinematicsSolver(&self[frame_offset.saturating_sub(1)].center);
        let p0 = &mut self[frame_offset];
        p0.center.accel = planner.accel(
            t0.location - p0.center.location,
            t0.velocity - p0.center.velocity,
            t0.accel
        );
        p0.center.angular_accel = planner.angular_accel(
            // t0.basis_matrix = rotation_matrix * p0.basis_matrix
            math::angle_of_rotation_matrix(&(t0.basis_matrix * glm::inverse(&p0.center.basis_matrix))),
            t0.angular_velocity - p0.center.angular_velocity,
            t0.angular_accel
        );
    }
    pub fn set_pridiction_to(&mut self, index: usize, targets: &Targets, planners: &mut Planners, frame_current: usize) {
        todo!()
    }
    pub fn get_pridiction(&mut self, index: usize, targets: &Targets, planner: &mut Box<dyn Planner>, frame_current: usize) -> &Pridiction {
        let frame_offset = index;
        let length = self.len();
        if length <= frame_offset {
            for offset in length..=frame_offset {
                self.data.push_back(Pridiction::new());
                self.set_pridiction_by_frame_offset(targets, planner, frame_current, offset);
            }
        }
        self.get(index).unwrap()
    }
}

ImplIndex!(Pridictions, Pridiction);

pub trait Planner {
    fn velocity_correction(&mut self, location_offset: Location) -> Velocity;
    fn accel_correction(&mut self, velocity_offset: Velocity) -> Accel;
    fn angular_velocity_correction(&mut self, rotation: Angle) -> AngularVelocity;
    fn angular_accel_correction(&mut self, angular_velocity_offset: AngularVelocity) -> AngularAccel;
    fn accel(&mut self, location_offset: Location, velocity_offset: Velocity, target_accel: Accel) -> Accel {
        let velocity_correction: Velocity = self.velocity_correction(location_offset);
        self.accel_correction(velocity_correction + velocity_offset) + target_accel
    }
    fn angular_accel(&mut self, rotation: Angle, angular_velocity_offset: AngularVelocity, target_angular_accel: AngularAccel) -> AngularAccel {
        let angular_velocity_correction: AngularVelocity = self.angular_velocity_correction(rotation);
        self.angular_accel_correction(angular_velocity_correction + angular_velocity_offset) + target_angular_accel
    }
    fn next(&self) -> GaitType {
        todo!()
    }
}

