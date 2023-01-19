use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::utils::macros::ImplCopy;

ImplCopy!{
    pub struct EffectOfForce {
        pub accel: Accel,
        pub angular_accel: AngularAccel
    }
}

ImplCopy!{
    pub struct MomentOfForce {
        pub force: Option<Force>,
        pub arm: Option<Location>,
    }
}

pub struct DynamicsState {
    pub forces: Vec<(Force, Location)>,
    // pub force: Force,
    // pub joint_force: Force,
    pub mass: Mass,
    // pub accel: Accel,
    // pub torque: Torque,
    // pub joint_torque: Torque,
    pub rotational_inertia: RotationalInertia,
    // pub angular_accel: AngularAccel,
}

pub fn ForwardDynamicsSolver(d: &DynamicsState) -> (Accel, AngularAccel) {
    let mut joint_force: Force = Force::zeros();
    let mut joint_torque: Torque = Torque::zeros();
    // for moment_of_force in &d.forces {
    for (force, arm) in &d.forces {
        /* if let (Some(force), Some(arm)) = (moment_of_force.force, moment_of_force.arm) {
            joint_force = joint_force + force;
            joint_torque = joint_torque + arm.cross(&force);
        } */
        joint_force = joint_force + force;
        joint_torque = joint_torque + arm.cross(&force);
    }
    (joint_force / d.mass, joint_torque / d.rotational_inertia)
}

// InverseDynamicsSolver
pub trait IDsolver {
    fn solve(&self, forces: &mut Vec<(Force, Location)>, accel: Accel, angular_accel: AngularAccel);
}

// pub fn InverseDynamicsSolver(d: &mut DynamicsState, accel: Accel, angular_accel: AngularAccel) {}