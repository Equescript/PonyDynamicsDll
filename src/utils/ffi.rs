use std::mem::size_of;
use std::alloc;
use std::os::raw::{c_float, c_double};
use std::slice;
use std::ffi::c_void;
use crate::units::{Length, Location};
use crate::math::{Vec3, Vec4, Mat3, Mat4};
// use std::libc::free;
pub fn vec3_from_ptr(ptr: *const c_double) -> Vec3 {
    unsafe {
        glm::make_vec3(slice::from_raw_parts(ptr, size_of::<c_double>()))
    }
}

pub fn vec3_from_slice(ptr: &[c_double]) -> Vec3 {
    unsafe {
        glm::make_vec3(ptr)
    }
}

pub fn vec4_from_ptr(ptr: *const c_double) -> Vec4 {
    unsafe {
        glm::make_vec4(slice::from_raw_parts(ptr, size_of::<c_double>()))
    }
}

pub fn vec4_from_slice(ptr: &[c_double]) -> Vec4 {
    unsafe {
        glm::make_vec4(ptr)
    }
}

pub fn mat3_from_ptr(ptr: *const c_double) -> Mat3 {
    unsafe {
        glm::make_mat3(slice::from_raw_parts(ptr, size_of::<c_double>()))
    }
}

pub fn mat3_from_slice(ptr: &[c_double]) -> Mat3 {
    unsafe {
        glm::make_mat3(ptr)
    }
}

pub fn mat4_from_ptr(ptr: *const c_double) -> Mat4 {
    unsafe {
        glm::make_mat4(slice::from_raw_parts(ptr, size_of::<c_double>()))
    }
}

pub fn mat4_from_slice(ptr: &[c_double]) -> Mat4 {
    unsafe {
        glm::make_mat4(ptr)
    }
}

pub fn load_legs_data(legs_data: *const c_double) -> [(
    (/*max_length */Length, /*hoove_rest */Location, /*hoove_rest_offset */Location, /*root_rest */Location),
    (/*max_step_height_or_factor */f64, /*is_factor */bool)
); 4] {
    const offset1: isize = 12;
    const offset2: isize = 24;
    const offset3: isize = 36;
    unsafe {
    [
        ( ( *legs_data, vec3_from_ptr(legs_data.offset(1)), vec3_from_ptr(legs_data.offset(4)), vec3_from_ptr(legs_data.offset(7)) ), ( *legs_data.offset(10), *legs_data.offset(11) != 0.0 ) ),
        ( ( *legs_data.offset(offset1), vec3_from_ptr(legs_data.offset(offset1 + 1)), vec3_from_ptr(legs_data.offset(offset1 + 4)), vec3_from_ptr(legs_data.offset(offset1 + 7)) ), ( *legs_data.offset(offset1 + 10), *legs_data.offset(offset1 + 11) != 0.0 ) ),
        ( ( *legs_data.offset(offset2), vec3_from_ptr(legs_data.offset(offset2 + 1)), vec3_from_ptr(legs_data.offset(offset2 + 4)), vec3_from_ptr(legs_data.offset(offset2 + 7)) ), ( *legs_data.offset(offset2 + 10), *legs_data.offset(offset2 + 11) != 0.0 ) ),
        ( ( *legs_data.offset(offset3), vec3_from_ptr(legs_data.offset(offset3 + 1)), vec3_from_ptr(legs_data.offset(offset3 + 4)), vec3_from_ptr(legs_data.offset(offset3 + 7)) ), ( *legs_data.offset(offset3 + 10), *legs_data.offset(offset3 + 11) != 0.0 ) ),
    ]
    }
}

pub struct GlobalDataBufferPtrs {
    pub ground_normal: *const c_double,
    pub ground_base_point: *const c_double,
    pub local_matrix: *const c_double,
    pub body_mass_center_local_location: *const c_double,
    pub legs_data: *const c_double,
}

impl GlobalDataBufferPtrs {
    pub unsafe fn alloc(&mut self) {
        let vec3 = alloc::Layout::array::<c_double>(3).unwrap();
        let mat3 = alloc::Layout::array::<c_double>(9).unwrap();
        let legs_data = alloc::Layout::array::<c_double>(48).unwrap();
        self.ground_normal = alloc::alloc(vec3)  as *const c_double;
        self.ground_base_point = alloc::alloc(vec3)  as *const c_double;
        self.local_matrix = alloc::alloc(mat3)  as *const c_double;
        self.body_mass_center_local_location = alloc::alloc(vec3)  as *const c_double;
        self.legs_data = alloc::alloc(legs_data)  as *const c_double;
    }
    pub unsafe fn dealloc(&mut self) {
        let vec3 = alloc::Layout::array::<c_double>(3).unwrap();
        let mat3 = alloc::Layout::array::<c_double>(9).unwrap();
        let legs_data = alloc::Layout::array::<c_double>(48).unwrap();
        alloc::dealloc(self.ground_normal as *mut u8, vec3);
        alloc::dealloc(self.ground_base_point as *mut u8, vec3);
        alloc::dealloc(self.local_matrix as *mut u8, mat3);
        alloc::dealloc(self.body_mass_center_local_location as *mut u8, vec3);
        alloc::dealloc(self.legs_data as *mut u8, legs_data);
    }
}