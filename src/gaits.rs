use crate::kinematics::KinematicsState;
use crate::targets::Targets;
use crate::utils::macros::{IntEnum, ImplIndex};
use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::predictor::{Planner, Pridictions};
use crate::armature::Controller;
use crate::armature::IDsolver;

IntEnum!{
    pub enum GaitType {
        Stance = 0,
        Walk,
        Trot,
        Gallop,
        InitiateWalk,
        InitiateTrot,
        InitiateGallop,
        TerminateWalk,
        TerminateTrot,
        TerminateGallop,
        WalkToTrot,
        TrotToGallop,
        GallopToTrot,
        TrotToWalk,
    }
}
/* pub enum Gaits {
    Stance(Stance),
    Walk(Walk),
    Trot(Trot),
    Gallop(Gallop),
    InitiateWalk(InitiateWalk),
    InitiateTrot(InitiateTrot),
    InitiateGallop(InitiateGallop),
    TerminateWalk(TerminateWalk),
    TerminateTrot(TerminateTrot),
    TerminateGallop(TerminateGallop),
    WalkToTrot(WalkToTrot),
    TrotToGallop(TrotToGallop),
    GallopToTrot(GallopToTrot),
    TrotToWalk(TrotToWalk),
}

macro_rules! GaitsUnwarp {
    ($gait:expr) => {
        {
            use crate::gaits::Gaits;
            match $gait {
                Gaits::Stance(t) => t,
                Gaits::Walk(t) => t,
                Gaits::Trot(t) => t,
                Gaits::Gallop(t) => t,
                Gaits::InitiateWalk(t) => t,
                Gaits::InitiateTrot(t) => t,
                Gaits::InitiateGallop(t) => t,
                Gaits::TerminateWalk(t) => t,
                Gaits::TerminateTrot(t) => t,
                Gaits::TerminateGallop(t) => t,
                Gaits::WalkToTrot(t) => t,
                Gaits::TrotToGallop(t) => t,
                Gaits::GallopToTrot(t) => t,
                Gaits::TrotToWalk(t) => t,
            }
        }
    };
} */

// pub(crate) use GaitsUnwarp;

/* impl Gaits {
    pub fn planner(&self) -> &dyn Planner {
        GaitsUnwarp!(self)
    }
    pub fn controller(&self) -> &dyn Controller {
        GaitsUnwarp!(self)
    }
} */

/* pub struct Gait<T: Planner, U: Controller> {
    planner: Box<T>,
    controller: Box<U>,
} */

pub struct Planners {
    data: [Box<dyn Planner>; 14],
}

impl Planners {
    // 该函数会决定对于预测器应该使用什么样的规划器
    pub fn get_planner_by_velocity(&mut self, targets: &Targets, results: &Vec<KinematicsState>, frame_current: usize) -> &mut Box<dyn Planner> {
        // GaitsUnwarp!(&self.gaits[self.gait_type as usize])
        // self.planners[self.gait_type as usize];
        todo!()
    }
}

ImplIndex!(Planners, Box<dyn Planner>);

pub struct Controllers {
    data: [Box<dyn Controller>; 14],
}

ImplIndex!(Controllers, Box<dyn Controller>);

pub struct IDsolvers {
    data: [Box<dyn IDsolver>; 14],
}

ImplIndex!(IDsolvers, Box<dyn IDsolver>);

pub struct GaitInfo {
    pub gait_type: GaitType,
    // pub gaits: [Gaits; 14],
    // planners: [Box<dyn Planner>; 14],
    pub planners: Planners,
    pub controllers: Controllers,
    pub IDsolvers: IDsolvers,
}

impl GaitInfo {

}


pub struct InitiateWalk {
    velocity_correction_factor: f64,
    accel_correction_factor: f64,
    angular_velocity_correction_factor: f64,
    angular_accel_correction_factor: f64,
}

impl Planner for InitiateWalk {
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

// impl Controller for InitiateWalk {

// }

pub struct Walk {
    velocity_correction_factor: f64,
    accel_correction_factor: f64,
    angular_velocity_correction_factor: f64,
    angular_accel_correction_factor: f64,
}

impl Planner for Walk {
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

// impl Controller for Walk {

// }


macro_rules! GaitToDo {
    ($name1:ident, $name2:ident) => {
        pub struct $name1 {

        }

        impl Planner for $name1 {
            fn velocity_correction(&mut self, location_offset: Location) -> Velocity {
                todo!()
            }
            fn accel_correction(&mut self, velocity_offset: Velocity) -> Accel {
                todo!()
            }
            fn angular_velocity_correction(&mut self, rotation: Angle) -> AngularVelocity {
                todo!()
            }
            fn angular_accel_correction(&mut self, angular_velocity_offset: AngularVelocity) -> AngularAccel {
                todo!()
            }
        }

        pub struct $name2 {

        }

        // impl Controller for $name2 {

        // }
    };
}

GaitToDo!(StancePlanner, StanceController);
GaitToDo!(TrotPlanner, TrotController);
GaitToDo!(GallopPlanner, GallopController);
GaitToDo!(InitiateTrotPlanner, InitiateTrotController);
GaitToDo!(InitiateGallopPlanner, InitiateGallopController);
GaitToDo!(TerminateWalkPlanner, TerminateWalkController);
GaitToDo!(TerminateTrotPlanner, TerminateTrotController);
GaitToDo!(TerminateGallopPlanner, TerminateGallopController);
GaitToDo!(WalkToTrotPlanner, WalkToTrotController);
GaitToDo!(TrotToGallopPlanner, TrotToGallopController);
GaitToDo!(GallopToTrotPlanner, GallopToTrotController);
GaitToDo!(TrotToWalkPlanner, TrotToWalkController);