extern crate nalgebra_glm as glm;

pub mod context;
pub mod dynamics;
pub mod gaits;
pub mod utils;
pub mod kinematics;
pub mod armature;
pub mod math;
pub mod predictor;
pub mod targets;
pub mod units;

use std::os::raw::{c_int, c_uint, c_ulonglong, c_double, c_void};
use context::Context;
use gaits::GaitType;
use utils::ffi::{vec3_from_ptr, mat3_from_ptr, load_legs_data};

use crate::math::{Vec3, Mat3};

static mut CONTEXT_PTR: Option<*mut Context> = None;
// static mut CONTEXT: Option<Context> = None;

#[inline]
fn context() -> &'static Context {
    unsafe {
        &*CONTEXT_PTR.unwrap()

    }
}
#[inline]
fn context_mut() -> &'static mut Context {
    unsafe {
        &mut *CONTEXT_PTR.unwrap()
    }
}

#[no_mangle]
pub extern fn initialize_context() {
    unsafe {
        if let None = CONTEXT_PTR {
            let layout = std::alloc::Layout::new::<Context>();
            CONTEXT_PTR = Some(std::alloc::alloc(layout) as *mut Context);
        }
    }
}

#[no_mangle]
pub extern fn initialize_path_data(targets_size: c_ulonglong, centripetal_force_factor: c_double, tick: c_double, frame_current: c_ulonglong,
    ground_normal: *const c_double, base_point: *const c_double, frictional_factor: c_double
) {
    context_mut().initialize_path_data(targets_size as usize, centripetal_force_factor, tick, frame_current as usize,
        vec3_from_ptr(ground_normal), vec3_from_ptr(base_point), frictional_factor
    );
}

#[no_mangle]
pub extern fn initialize_armature(root_location: *const c_double, root_basis_matrix: *const c_double, head_location: *const c_double, head_basis_matrix: *const c_double,
    FL_root_location: *const c_double, FL_hoove_location: *const c_double, FL_hoove_basis_matrix: *const c_double,
    FR_root_location: *const c_double, FR_hoove_location: *const c_double, FR_hoove_basis_matrix: *const c_double,
    BL_root_location: *const c_double, BL_hoove_location: *const c_double, BL_hoove_basis_matrix: *const c_double,
    BR_root_location: *const c_double, BR_hoove_location: *const c_double, BR_hoove_basis_matrix: *const c_double,
    center_location: *const c_double, center_basis_matrix: *const c_double,
    head_mass: c_double, head_rotational_inertia: c_double, center_mass: c_double, center_rotational_inertia: c_double
) {
    context_mut().initialize_armature(vec3_from_ptr(root_location), mat3_from_ptr(root_basis_matrix),
        vec3_from_ptr(head_location), mat3_from_ptr(head_basis_matrix),
        [
            (vec3_from_ptr(FL_root_location), (vec3_from_ptr(FL_hoove_location), mat3_from_ptr(FL_hoove_basis_matrix))),
            (vec3_from_ptr(FR_root_location), (vec3_from_ptr(FR_hoove_location), mat3_from_ptr(FR_hoove_basis_matrix))),
            (vec3_from_ptr(BL_root_location), (vec3_from_ptr(BL_hoove_location), mat3_from_ptr(BL_hoove_basis_matrix))),
            (vec3_from_ptr(BR_root_location), (vec3_from_ptr(BR_hoove_location), mat3_from_ptr(BR_hoove_basis_matrix))),
        ],
        vec3_from_ptr(center_location), mat3_from_ptr(center_basis_matrix),
        head_mass, head_rotational_inertia, center_mass, center_rotational_inertia
    );
}

#[no_mangle]
pub extern fn initialize_InitiateWalk(velocity_correction_factor: c_double, accel_correction_factor: c_double,
    angular_velocity_correction_factor: c_double, angular_accel_correction_factor: c_double, preferred_leg: c_uint,
    FL_max_height: c_double, FL_max_forward: c_double, FL_max_backward: c_double,
    FR_max_height: c_double, FR_max_forward: c_double, FR_max_backward: c_double,
    BL_max_height: c_double, BL_max_forward: c_double, BL_max_backward: c_double,
    BR_max_height: c_double, BR_max_forward: c_double, BR_max_backward: c_double, contact_percentage: c_double, pattern_recovery_factor: c_double
) {
    let gait_info = &mut context_mut().gait_info;
    gait_info.planners[GaitType::InitiateWalk as usize] = Box::new(gaits::InitiateWalk::InitiateWalkPlanner::initialize(velocity_correction_factor, accel_correction_factor, angular_velocity_correction_factor, angular_accel_correction_factor));
    gait_info.controllers[GaitType::InitiateWalk as usize] = Box::new(gaits::InitiateWalk::InitiateWalkController::initialize(
        (preferred_leg as usize).try_into().unwrap(),
        [ FL_max_height, FR_max_height, BL_max_height, BR_max_height],
        [
            (FL_max_forward, FL_max_backward),
            (FR_max_forward, FR_max_backward),
            (BL_max_forward, BL_max_backward),
            (BR_max_forward, BR_max_backward),
        ],
        contact_percentage, pattern_recovery_factor
    ));
    gait_info.IDsolvers[GaitType::InitiateWalk as usize] = Box::new(gaits::InitiateWalk::InitiateWalkIDsolver {});
}

#[no_mangle]
pub extern fn initialize_Walk(velocity_correction_factor: c_double, accel_correction_factor: c_double, angular_velocity_correction_factor: c_double, angular_accel_correction_factor: c_double,
    FL_max_height: c_double, FL_max_forward: c_double, FL_max_backward: c_double,
    FR_max_height: c_double, FR_max_forward: c_double, FR_max_backward: c_double,
    BL_max_height: c_double, BL_max_forward: c_double, BL_max_backward: c_double,
    BR_max_height: c_double, BR_max_forward: c_double, BR_max_backward: c_double, contact_percentage: c_double, pattern_recovery_factor: c_double
) {
    let gait_info = &mut context_mut().gait_info;
    gait_info.planners[GaitType::Walk as usize] = Box::new(gaits::Walk::WalkPlanner::initialize(velocity_correction_factor, accel_correction_factor, angular_velocity_correction_factor, angular_accel_correction_factor));
    gait_info.controllers[GaitType::Walk as usize] = Box::new(gaits::Walk::WalkController::initialize(
        [ FL_max_height, FR_max_height, BL_max_height, BR_max_height],
        [
            (FL_max_forward, FL_max_backward),
            (FR_max_forward, FR_max_backward),
            (BL_max_forward, BL_max_backward),
            (BR_max_forward, BR_max_backward),
        ],
        contact_percentage, pattern_recovery_factor
    ));
    gait_info.IDsolvers[GaitType::Walk as usize] = Box::new(gaits::Walk::WalkIDsolver {});
}

#[no_mangle]
pub extern fn set_target_by_frame(location: *const c_double, direction: *const c_double, gravity: *const c_double, frame: c_ulonglong) {
    // the tranform of gravity
    let context = context_mut();
    let centripetal_force_factor = context.targets.centripetal_force_factor;
    targets::set_target_by_frame(&mut context.targets, vec3_from_ptr(location), vec3_from_ptr(direction), vec3_from_ptr(gravity) / context.tick, frame as usize, centripetal_force_factor, context.armature_rest.center.location.z)
}

#[no_mangle]
pub extern fn calculate() {
    let context = context_mut();
    let length = context.length;
    for i in 0..length {
        context.calculate(i);
    }
}

#[no_mangle]
pub extern fn test() {
    println!("Hello, world!");
    let m1: Mat3 = math::make_mat3_from_vec3(Vec3::new(1.0, 2.0, 3.0), Vec3::new(4.0, 5.0, 6.0), Vec3::new(7.0, 8.0, 9.0));
    let v1: Vec3 = Vec3::new(3.0, 2.0, 1.0);
    println!("{}*{}={}", m1, v1, m1*v1)
}

