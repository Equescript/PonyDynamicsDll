use crate::targets::Targets;
use crate::utils::macros::{ImplCopy, IntEnum, ImplIndex};
use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::math;
use crate::predictor::{Planner, Pridictions, Pridiction};
use crate::armature::{Controller, ArmatureKinematics};
use crate::armature::IDsolver;
use crate::kinematics::{KinematicsState, ForwardKinematicsSolver};
use crate::dynamics::EffectOfForce;

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

ImplCopy!{
    pub enum GaitsPridiction {
        Stance(StancePridiction),
        Walk(WalkPridiction),
        Trot(TrotPridiction),
        Gallop(GallopPridiction),
        InitiateWalk(InitiateWalkPridiction),
        InitiateTrot(InitiateTrotPridiction),
        InitiateGallop(InitiateGallopPridiction),
        TerminateWalk(TerminateWalkPridiction),
        // TerminateTrot(TerminateTrotPridiction),
        // TerminateGallop(TerminateGallopPridiction),
        // WalkToTrot(WalkToTrotPridiction),
        // TrotToGallop(TrotToGallopPridiction),
        // GallopToTrot(GallopToTrotPridiction),
        // TrotToWalk(TrotToWalkPridiction),
    }
}

/* macro_rules! GaitsUnwarp {
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
    pub fn get_planner_by_velocity(&mut self, targets: &Targets, results: &Vec<ArmatureKinematics>, frame_current: usize) -> &mut Box<dyn Planner> {
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


pub struct InitiateWalkPlanner {
    velocity_correction_factor: f64,
    accel_correction_factor: f64,
    angular_velocity_correction_factor: f64,
    angular_accel_correction_factor: f64,
    next_planner: GaitType,
}

    /* fn velocity_correction(&mut self, location_offset: Location) -> Velocity;
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
    } */
/* impl Planner for InitiateWalkPlanner {
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
} */
impl Planner for InitiateWalkPlanner {
    fn calculate(&mut self, targets: &Targets, results: &Vec<ArmatureKinematics>, pridictions: &mut Pridictions, frame_current: usize, future_offset: usize) -> Result<(), ()> {
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

        let p0 = &mut pridictions[future_offset - 1];
        let t0 = &targets[frame_current + future_offset - 1];
        let location_offset: Location = t0.center.location - p0.center.location;
        let velocity_offset: Velocity = t0.center.velocity - p0.center.velocity;
        p0.center.accel = accel(location_offset, velocity_offset, t0.center.accel);
        // t0.center.basis_matrix = rotation_matrix * p0.center.basis_matrix
        let rotation: Angle = math::angle_of_rotation_matrix(&(t0.center.basis_matrix * glm::inverse(&p0.center.basis_matrix)));
        let angular_velocity_offset: AngularVelocity = t0.center.angular_velocity - p0.center.angular_velocity;
        p0.center.angular_accel = angular_accel(rotation, angular_velocity_offset, t0.center.angular_accel);
        pridictions[future_offset] = Pridiction {
            center: ForwardKinematicsSolver(&p0.center),
            gait_data: GaitsPridiction::InitiateWalk(InitiateWalkPridiction {  }),
            planner_type: GaitType::InitiateWalk,
        };
        Ok(())
    }
    fn next(&self) -> GaitType {
        self.next_planner
    }
}
// impl Controller for InitiateWalkController {

// }

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
// impl Controller for Walk {

// }

ImplCopy!{
    pub struct WalkPridiction {}
}
ImplCopy!{
    pub struct InitiateWalkPridiction {}
}

macro_rules! GaitToDo {
    ($name1:ident, $name2:ident, $name3:ident) => {
        pub struct $name1 {

        }

        /* impl Planner for $name1 {
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
        } */

        pub struct $name2 {

        }

        // impl Controller for $name2 {

        // }
        ImplCopy!{
            pub struct $name3 {

            }
        }
    };
}

GaitToDo!(StancePlanner, StanceController, StancePridiction);
GaitToDo!(TrotPlanner, TrotController, TrotPridiction);
GaitToDo!(GallopPlanner, GallopController, GallopPridiction);
GaitToDo!(InitiateTrotPlanner, InitiateTrotController, InitiateTrotPridiction);
GaitToDo!(InitiateGallopPlanner, InitiateGallopController, InitiateGallopPridiction);
GaitToDo!(TerminateWalkPlanner, TerminateWalkController, TerminateWalkPridiction);
GaitToDo!(TerminateTrotPlanner, TerminateTrotController, TerminateTrotPridiction);
GaitToDo!(TerminateGallopPlanner, TerminateGallopController, TerminateGallopPridiction);
GaitToDo!(WalkToTrotPlanner, WalkToTrotController, WalkToTrotPridiction);
GaitToDo!(TrotToGallopPlanner, TrotToGallopController, TrotToGallopPridiction);
GaitToDo!(GallopToTrotPlanner, GallopToTrotController, GallopToTrotPridiction);
GaitToDo!(TrotToWalkPlanner, TrotToWalkController, TrotToWalkPridiction);
