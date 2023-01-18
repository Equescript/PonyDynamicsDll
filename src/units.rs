use super::math::Vec3;

macro_rules! UsePhysicsUnits {
    () => {
        use crate::units::{
            Time, Length,
            Location, Mass,           Force,  Momentum,        Velocity,        Accel,       ElasticCoeff, FrictionalCoeff,
            Angle, RotationalInertia, Torque, AngularMomentum, AngularVelocity, AngularAccel
        };
    };
}

pub(crate) use UsePhysicsUnits;

pub type Time = f64;
pub type Length = f64;

pub type Location = Vec3;
pub type Mass = f64;
pub type Force = Vec3;
pub type Momentum = Vec3;
pub type Velocity = Vec3;
pub type Accel = Vec3;
pub type ElasticCoeff = f64;
pub type FrictionalCoeff = f64;

pub type Angle = Vec3;
pub type RotationalInertia = f64;
pub type Torque = Vec3;
pub type AngularMomentum = Vec3;
pub type AngularVelocity = Vec3;
pub type AngularAccel = Vec3;

