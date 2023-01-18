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

// static mut GlobalDataPtr: Option<*mut GlobalData> = None;

#[no_mangle]
pub extern fn test() {
    println!("Hello, world!");
}

