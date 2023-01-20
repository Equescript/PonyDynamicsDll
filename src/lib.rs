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
use utils::ffi::{vec3_from_ptr, mat3_from_ptr, load_legs_data};

static mut ContextPtr: Option<*mut Context> = None;

#[inline]
fn context() -> &'static Context {
    unsafe {
        &*ContextPtr.unwrap()
    }
}
#[inline]
fn context_mut() -> &'static mut Context {
    unsafe {
        &mut *ContextPtr.unwrap()
    }
}

#[no_mangle]
pub extern fn set_target_by_frame(location: *const c_double, direction: *const c_double, gravity: *const c_double, frame: c_ulonglong) {
    // the tranform of gravity
    let context = context_mut();
    let centripetal_force_factor = context.targets.centripetal_force_factor;
    targets::set_target_by_frame(&mut context.targets, vec3_from_ptr(location), vec3_from_ptr(direction), vec3_from_ptr(gravity) / context.tick, frame as usize, centripetal_force_factor, context.armature_rest.center.location.z)
}

#[no_mangle]
pub extern fn test() {
    println!("Hello, world!");
}

