use std::collections::VecDeque;
use std::ops::{Index, IndexMut};
use crate::math;
use crate::math::{Mat3};
use crate::targets::Targets;
use crate::utils::macros::{ImplCopy, ImplIndex};
use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::kinematics::{KinematicsState, ForwardKinematicsSolver};
use crate::dynamics::EffectOfForce;
use crate::gaits::{GaitPridiction, GaitType, Planners, StancePridiction};
use crate::armature::ArmatureKinematics;

// pub struct Pridiction {
    // pub center: KinematicsState,
    // pub projection: Location,
// }

// impl Pridiction {
//     fn new() -> Self {
//         Self { center: KinematicsState::new(), projection: Location::zeros() }
//     }
// }
#[derive(Clone, Copy)]
pub struct Pridiction {
    pub center: KinematicsState,
    // pub armature_kinematics: ArmatureKinematics,
    pub gait_data: GaitPridiction,
    pub planner_type: GaitType,
}


impl Pridiction {
    pub fn new() -> Self {
        Pridiction { center: KinematicsState::new(), gait_data: GaitPridiction::Stance(StancePridiction {}), planner_type: GaitType::Stance }
    }
}

pub struct Pridictions {
    data: VecDeque<Pridiction>,
}

impl Pridictions {
    pub fn new() -> Self {
        Pridictions { data: VecDeque::new() }
    }
    // pub fn renew_current(&mut self, center_current: &CenterTarget) {
    //     self.data.clear();
    //     // self.predictions[0].center = *center_current;
    //     self.data.push_back(Pridiction { center: *center_current, projection: Location::zeros() })
    // }
    pub fn pop_front(&mut self, (targets,  results,                  planners,      frame_current):
                                (&Targets, &Vec<ArmatureKinematics>, &mut Planners, usize)
    ) -> Result<Pridiction, ()> {
        if self.len() <= 2 {
            self.calculate_pridiction_of(2, targets, results, planners, frame_current)?;
        }
        match self.data.pop_front() {
            Some(t) => Ok(t),
            None => Err(())
        }
    }
    pub fn refresh(&mut self, k: KinematicsState, (targets,  results,                  planners,      frame_current):
                                                  (&Targets, &Vec<ArmatureKinematics>, &mut Planners, usize)
    ) -> Result<(), ()> {
        self.data[0].center = k;
        let pridiction = self.data[0];
        self.data.clear();
        self.data.push_back(pridiction);
        self.calculate_pridiction_of(1, targets, results, planners, frame_current)
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
    pub fn calculate_pridiction_of(&mut self, index: usize,
        targets: &Targets, results: &Vec<ArmatureKinematics>,
        planners: &mut Planners, frame_current: usize
    ) -> Result<(), ()> {
        let last = self.len() - 1;
        if last >= index {
            return Ok(());
        }

        let mut planner = &mut planners[self[last].planner_type as usize];

        for i in last..index {
            planner.calculate_next_of(targets, results, self, frame_current, i)?;
            let next_type = planner.next();
            planner = &mut planners[next_type as usize];
        }
        Ok(())
    }
    pub fn get_pridiction(&mut self, index: usize, (targets,  results,                  planners,      frame_current):
                                                   (&Targets, &Vec<ArmatureKinematics>, &mut Planners, usize)
    ) -> Result<&Pridiction, ()> {
        let length = self.len();
        if length <= index {
            self.calculate_pridiction_of(index, targets, results, planners, frame_current)?;
        }
        match self.get(index) {
            Some(t) => Ok(t),
            None => Err(()),
        }
    }
}

ImplIndex!(Pridictions, Pridiction);

pub trait Planner {
    fn calculate_next_of(&mut self, targets: &Targets, results: &Vec<ArmatureKinematics>, pridictions: &mut Pridictions, frame_current: usize, frame_offset: usize) -> Result<(), ()> {
        todo!()
    }
    fn next(&self) -> GaitType {
        todo!()
    }
}

