use std::ops::{Index, IndexMut};
use crate::units::UsePhysicsUnits;
UsePhysicsUnits!();
use crate::utils::macros::ImplCopy;
use crate::kinematics::{KinematicsState, InverseKinematicsSolver};
use crate::math;
use crate::math::{Vec3, Mat3};

struct ProjectionTarget {
    location: Location,
    velocity: Velocity,
    accel: Accel,
}

impl ProjectionTarget {
    fn new() -> Self {
        Self { location: Location::zeros(), velocity: Velocity::zeros(), accel: Accel::zeros() }
    }
}

pub struct Target {
    projection: ProjectionTarget,
    pub center: KinematicsState,
    virtual_g_accel: Accel, // (m/f^2)
    virtual_g_normal: Vec3,
    pub g_accel: Accel
}

impl Target {
    pub fn new(location: Location, g_accel: Accel) -> Self {
        Target {
            projection: ProjectionTarget { location: location, velocity: Vec3::zeros(), accel: Vec3::zeros() },
            center: KinematicsState::new(),
            virtual_g_accel: Accel::zeros(), // (m/f^2)
            virtual_g_normal: Vec3::zeros(),
            g_accel: g_accel
        }
    }
}

pub struct Targets {
    data: Vec<Target>,
    pub centripetal_force_factor: f64,
}

impl Targets {
    pub fn new() -> Self {
        Targets { data: Vec::new(), centripetal_force_factor: 0.0 }
    }
    pub fn initialize(&mut self, size: usize) {
        self.data = Vec::with_capacity(size + 10);
    }
    pub fn len(&self) -> usize {
        self.data.len()
    }
    pub fn get(&self, index: usize) -> &Target {
        let length = self.len();
        if index >= length {
            &self.data[length-1]
        } else {
            &self.data[index]
        }
    }
    pub fn get_mut(&mut self, index: usize) -> &mut Target {
        let length = self.len();
        if index >= length {
            &mut self.data[length-1]
        } else {
            &mut self.data[index]
        }
    }
}

impl Index<usize> for Targets {
    type Output = Target;
    fn index(&self, index: usize) -> &Self::Output {
        self.get(index)
    }
}
impl IndexMut<usize> for Targets {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        self.get_mut(index)
    }
}

pub fn set_target_by_frame(targets: &mut Targets, location: Location, _direction: Vec3, g_accel: Accel /* m/f^2 */, frame: usize, centripetal_force_factor: f64, center_height: f64) {
    let i_0 = frame;
    targets[i_0].projection.location = location;
    targets[i_0].g_accel /* m/f^2 */ = g_accel;
    targets[i_0] = Target::new(location, g_accel);
    let i_1 = frame.saturating_sub(1);
    let i_2 = frame.saturating_sub(2);
    let i_3 = frame.saturating_sub(3);
    let i_4 = frame.saturating_sub(4);
    targets[i_1].projection.velocity = targets[i_0].projection.location - targets[i_1].projection.location;
    targets[i_2].projection.accel = targets[i_1].projection.velocity - targets[i_2].projection.velocity;
    // TODO: controller correct the target.
    targets[i_2].virtual_g_accel = targets[i_2].g_accel - targets[i_2].projection.accel * centripetal_force_factor;
    let forward_vec: Vec3 = targets[i_2].projection.velocity.normalize();
    let sideward_vec: Vec3 = targets[i_2].virtual_g_accel.cross(&forward_vec).normalize();
    let upward_vec: Vec3 = forward_vec.cross(&sideward_vec).normalize();
    targets[i_2].center.basis_matrix = math::make_mat3_from_vec3(forward_vec, sideward_vec, upward_vec);
    // rotation_matrix * local_matrix = basis_matrix
    // rotation_matrix = basis_matrix * local_matrix_inverse
    // targets[i_2].center.rotation_matrix = targets[i_2].center.basis_matrix * global_data.armature.local_matrix_inverse;

    // i_2.basis_matrix = delta_rotation_matrix * i_3.basis_matrix
    let delta_rotation_matrix: Mat3 =  targets[i_2].center.basis_matrix * glm::inverse(&targets[i_3].center.basis_matrix);
    targets[i_3].center.angular_velocity = math::angle_of_rotation_matrix(&delta_rotation_matrix);
    targets[i_4].center.angular_accel = targets[i_3].center.angular_velocity - targets[i_4].center.angular_velocity;

    targets[i_2].virtual_g_normal = targets[i_2].virtual_g_accel.normalize();
    targets[i_2].center.location = targets[i_2].projection.location - targets[i_2].virtual_g_normal * center_height;
    targets[i_3].center.velocity = targets[i_2].center.location - targets[i_3].center.location;
    targets[i_4].center.accel = targets[i_3].center.velocity - targets[i_4].center.velocity;
    // InverseKinematicsSolver(&mut targets[i_4].center, &mut targets[i_3].center, &mut targets[i_2].center);
}

