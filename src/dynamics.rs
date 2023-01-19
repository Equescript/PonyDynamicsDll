use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::utils::macros::ImplCopy;

ImplCopy!{
    pub struct EffectOfForce {
        pub accel: Accel,
        pub angular_accel: AngularAccel
    }
}

impl std::ops::Add for EffectOfForce {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            accel: self.accel + rhs.accel,
            angular_accel: self.angular_accel + rhs.angular_accel,
        }
    }
}

impl std::ops::Sub for EffectOfForce {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            accel: self.accel - rhs.accel,
            angular_accel: self.angular_accel - rhs.angular_accel,
        }
    }
}

ImplCopy!{
    pub struct MomentOfForce {
        pub force: Force,
        pub arm: Location,
    }
}

impl MomentOfForce {
    pub fn torque(&self) -> Torque {
        self.arm.cross(&self.force)
    }
}

pub struct DynamicsState {
    pub forces: Vec<MomentOfForce>,
    // pub force: Force,
    // pub joint_force: Force,
    pub mass: Mass,
    // pub accel: Accel,
    // pub torque: Torque,
    // pub joint_torque: Torque,
    pub rotational_inertia: RotationalInertia,
    // pub angular_accel: AngularAccel,
}

impl DynamicsState {
    pub fn new() -> Self {
        Self { forces: Vec::new(), mass: 0.0, rotational_inertia: 0.0 }
    }
    pub fn initialize(mass: Mass, rotational_inertia: RotationalInertia) -> Self {
        Self { forces: Vec::new(), mass, rotational_inertia }
    }
}

pub fn ForwardDynamicsSolver(d: &DynamicsState) -> EffectOfForce {
    let mut joint_force: Force = Force::zeros();
    let mut joint_torque: Torque = Torque::zeros();
    for moment_of_force in &d.forces {
    // for (force, arm) in &d.forces {
        /* if let (Some(force), Some(arm)) = (moment_of_force.force, moment_of_force.arm) {
            joint_force = joint_force + force;
            joint_torque = joint_torque + arm.cross(&force);
        } */
        joint_force = joint_force + moment_of_force.force;
        joint_torque = joint_torque + moment_of_force.torque();
    }
    EffectOfForce { accel: joint_force / d.mass, angular_accel: joint_torque / d.rotational_inertia }
}

// InverseDynamicsSolver

// pub fn InverseDynamicsSolver(d: &mut DynamicsState, accel: Accel, angular_accel: AngularAccel) {}