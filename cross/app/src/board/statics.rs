#![allow(unsafe_code)]
use core::cell::UnsafeCell;
use core::mem::MaybeUninit;

/// Wrapper for statics initialized in Board::new and used later.
pub struct SafelyInitializedStatic<T>(UnsafeCell<MaybeUninit<T>>);
unsafe impl<T> Sync for SafelyInitializedStatic<T> {}

impl<T> SafelyInitializedStatic<T> {
    /// Constructs a new instance of the wrapper.
    pub const fn new() -> Self {
        Self(UnsafeCell::new(MaybeUninit::uninit()))
    }

    /// Initialize the value returning the reference.
    pub fn write(&self, value: T) -> &T {
        unsafe { &mut *self.0.get() }.write(value)
    }
}
