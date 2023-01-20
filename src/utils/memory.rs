use crate::utils::macros::IntEnum;
use std::alloc;
use std::os::raw::{c_void, c_char, c_uchar, c_short, c_ushort, c_int, c_uint, c_longlong, c_ulonglong, c_float, c_double};

IntEnum!{
    pub enum Ctypes {
        c_void = 0, // c_void is ptr
        c_char,
        c_uchar,
        c_short,
        c_ushort,
        c_int,
        c_uint,
        c_longlong,
        c_ulonglong,
        c_float,
        c_double,
    }
}

impl Ctypes {
    pub fn layout(&self, n: usize) -> alloc::Layout {
        match self {
            Ctypes::c_void => alloc::Layout::array::<*const std::os::raw::c_void>(n).unwrap(),
            Ctypes::c_char => alloc::Layout::array::<std::os::raw::c_char>(n).unwrap(),
            Ctypes::c_uchar => alloc::Layout::array::<std::os::raw::c_uchar>(n).unwrap(),
            Ctypes::c_short => alloc::Layout::array::<std::os::raw::c_short>(n).unwrap(),
            Ctypes::c_ushort => alloc::Layout::array::<std::os::raw::c_ushort>(n).unwrap(),
            Ctypes::c_int => alloc::Layout::array::<std::os::raw::c_int>(n).unwrap(),
            Ctypes::c_uint => alloc::Layout::array::<std::os::raw::c_uint>(n).unwrap(),
            Ctypes::c_longlong => alloc::Layout::array::<std::os::raw::c_longlong>(n).unwrap(),
            Ctypes::c_ulonglong => alloc::Layout::array::<std::os::raw::c_ulonglong>(n).unwrap(),
            Ctypes::c_float => alloc::Layout::array::<std::os::raw::c_float>(n).unwrap(),
            Ctypes::c_double => alloc::Layout::array::<std::os::raw::c_double>(n).unwrap(),
        }
    }
}

struct Pointer<T> {
    is_null: bool,
    ptr: *const T,
    count: usize,
    layout: alloc::Layout,
}

impl<T> std::convert::TryFrom<Pointer<T>> for *const T {
    type Error = ();
    fn try_from(value: Pointer<T>) -> Result<Self, Self::Error> {
        if value.is_null() {
            Err(())
        } else {
            Ok(value.ptr)
        }
    }
}

impl<T> Pointer<T> {
    pub fn new() -> Self {
        Pointer {
            is_null: true,
            ptr: 0 as *const T,
            count: 0,
            layout: alloc::Layout::new::<T>()
        }
    }
    #[inline]
    pub fn is_null(&self) -> bool {
        self.is_null
    }
    pub fn ptr(&self) -> Option<*const T> {
        if self.is_null() {
            None
        } else {
            Some(self.ptr)
        }
    }
    pub unsafe fn alloc(&mut self, count: usize) {
        self.dealloc();
        self.count = count;
        self.layout = alloc::Layout::array::<T>(count).unwrap();
        self.ptr = alloc::alloc(self.layout) as *const T;
        self.is_null = false;
    }
    pub unsafe fn dealloc(&mut self) {
        if !self.is_null() {
            alloc::dealloc(self.ptr as *mut u8, self.layout);
            self.is_null = true;
        }
    }
}
