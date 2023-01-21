use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::math;
use crate::math::{Mat3, Mat4, Qua};
use crate::utils::macros::ImplCopy;
use crate::dynamics::{DynamicsState, EffectOfForce};

ImplCopy!{
    pub struct Pose {
        pub location: Location,
        pub basis_matrix: Mat3,
    }
}

impl Pose {
    pub fn new() -> Self {
        Self { location: Location::zeros(), basis_matrix: Mat3::zeros() }
    }
    pub fn transformation_matrix_from(&self, pose: &Pose) -> Mat4 {
        math::make_transformation_matrix(self.location - pose.location, self.basis_matrix * glm::inverse(&pose.basis_matrix))
    }
    pub fn transform_with_rotation_angle_from(&self, pose: &Pose) -> Transform {
        Transform {
            location: self.location - pose.location,
            rotation: Roatation::Angle(math::angle_of_rotation_matrix(&(self.basis_matrix * glm::inverse(&pose.basis_matrix))))
        }
    }
    pub fn transform_with_rotation_matrix_from(&self, pose: &Pose) -> Transform {
        Transform {
            location: self.location - pose.location,
            rotation: Roatation::Matrix(self.basis_matrix * glm::inverse(&pose.basis_matrix))
        }
    }
    pub fn transform(p: &Pose, t: &Transform) -> Pose {
        Pose {
            location: p.location + t.location,
            basis_matrix: match t.rotation {
                Roatation::Angle(a) => math::rotation_mat3(a),
                Roatation::Matrix(m) => m,
            } * p.basis_matrix
        }
    }
    pub fn transformed_by(&mut self, t: &Transform) {
        self.location = self.location + t.location;
        self.basis_matrix = match t.rotation {
            Roatation::Angle(a) => math::rotation_mat3(a),
            Roatation::Matrix(m) => m,
        } * self.basis_matrix
        // TODO: Maybe normalize?
    }
}
ImplCopy!{
    pub enum Roatation {
        Angle(Angle),
        Matrix(Mat3),
    }
}

ImplCopy!{
    pub struct Transform {
        pub location: Location,
        pub rotation: Roatation,
    }
}

impl Transform {
    pub fn new() -> Self {
        Self { location: Location::zeros(), rotation: Roatation::Matrix(Mat3::zeros()) }
    }
    pub fn transformation_matrix(&self) -> Mat4 {
        match self.rotation {
            Roatation::Angle(t) => math::make_transformation_matrix(self.location, math::rotation_mat3(t)),
            Roatation::Matrix(t) => math::make_transformation_matrix(self.location, t),
        }
    }
    pub fn rotation_matrix(&self) -> Mat3 {
        match self.rotation {
            Roatation::Angle(t) => math::rotation_mat3(t),
            Roatation::Matrix(t) => t,
        }
    }
    pub fn rotation_angle(&self) -> Angle {
        match self.rotation {
            Roatation::Angle(t) => t,
            Roatation::Matrix(t) => math::angle_of_rotation_matrix(&t),
        }
    }
}

ImplCopy!{
    pub struct KinematicsState {
        pub location: Location,
        pub velocity: Velocity,
        pub accel: Accel,
        pub basis_matrix: Mat3,
        pub angular_velocity: AngularVelocity,
        pub angular_accel: AngularAccel,
    }
}

impl KinematicsState {
    pub fn new() -> Self {
        Self {
            location: Location::zeros(),
            velocity: Velocity::zeros(),
            accel: Accel::zeros(),
            basis_matrix: Mat3::zeros(),
            angular_velocity: AngularVelocity::zeros(),
            angular_accel: AngularAccel::zeros(),
        }
    }
    /* pub fn effect_of_force(&self) -> EffectOfForce {
        EffectOfForce { accel: self.accel, angular_accel: self.angular_accel }
    } */
    pub fn solve(&mut self) {
        self.location = self.location + self.velocity;
        self.velocity = self.velocity + self.accel;
        self.accel = Accel::zeros();
        self.basis_matrix = math::rotation_mat3(self.angular_velocity) * self.basis_matrix;
        self.angular_velocity = self.angular_velocity + self.angular_accel;
        self.angular_accel = AngularAccel::zeros()
    }
}

impl std::convert::From<KinematicsState> for EffectOfForce {
    fn from(k: KinematicsState) -> Self {
        Self { accel: k.accel, angular_accel: k.angular_accel }
    }
}

impl std::convert::From<KinematicsState> for Pose {
    fn from(k: KinematicsState) -> Self {
        Self { location: k.location, basis_matrix: k.basis_matrix }
    }
}

pub fn ForwardKinematicsSolver(k: &KinematicsState) -> KinematicsState {
    // k.accel = accel;
    // k.angular_accel = angular_accel;
    KinematicsState {
        location: k.location + k.velocity,
        velocity: k.velocity + k.accel,
        accel: Accel::zeros(),
        basis_matrix: math::rotation_mat3(k.angular_velocity) * k.basis_matrix,
        angular_velocity: k.angular_velocity + k.angular_accel,
        angular_accel: AngularAccel::zeros()
    }
}

pub fn InverseKinematicsSolver(k1: &mut KinematicsState, k2: &mut KinematicsState, k3: &mut KinematicsState) {
    k2.velocity = k3.location - k2.location;
    k1.accel = k2.velocity - k1.velocity;

    // k3.basis_matrix = rotation_matrix * k2.basis_matrix;
    let rotation_matrix: Mat3 = k3.basis_matrix * glm::inverse(&k2.basis_matrix);
    k2.angular_velocity = math::angle_of_rotation_matrix(&rotation_matrix);
    k1.angular_accel = k2.angular_velocity - k1.angular_velocity;
}