//! Implementation of critical_section which is safe to use from both cores

use core::sync::atomic::{AtomicU8, Ordering};

use crate::multicore::CoreId;
use crate::pac;

struct RpSpinlockCs;
critical_section::set_impl!(RpSpinlockCs);

/// Marker value to indicate no-one has the lock.
///
/// Initialising `LOCK_OWNER` to 0 means cheaper static initialisation so it's the best choice
const LOCK_UNOWNED: u8 = 0;

/// Indicates which core owns the lock so that we can call critical_section recursively.
///
/// 0 = no one has the lock, 1 = core0 has the lock, 2 = core1 has the lock
static LOCK_OWNER: AtomicU8 = AtomicU8::new(LOCK_UNOWNED);

/// Marker value to indicate that we already owned the lock when we started the `critical_section`.
///
/// Since we can't take the spinlock when we already have it, we need some other way to keep track of `critical_section` ownership.
/// `critical_section` provides a token for communicating between `acquire` and `release` so we use that.
/// If we're the outermost call to `critical_section` we use the values 0 and 1 to indicate we should release the spinlock and set the interrupts back to disabled and enabled, respectively.
/// The value 2 indicates that we aren't the outermost call, and should not release the spinlock or re-enable interrupts in `release`
const LOCK_ALREADY_OWNED: u8 = 2;

unsafe impl critical_section::Impl for RpSpinlockCs {
    unsafe fn acquire() -> u8 {
        RpSpinlockCs::acquire()
    }

    unsafe fn release(token: u8) {
        RpSpinlockCs::release(token);
    }
}

impl RpSpinlockCs {
    unsafe fn acquire() -> u8 {
        // Store the initial interrupt state and current core id in stack variables
        let interrupts_active = cortex_m::register::primask::read().is_active();
        // We reserved 0 as our `LOCK_UNOWNED` value, so add 1 to core_id so we get 1 for core0, 2 for core1.
        let core = pac::SIO.cpuid().read() as u8 + 1;
        // Do we already own the spinlock?
        if LOCK_OWNER.load(Ordering::Acquire) == core {
            // We already own the lock, so we must have called acquire within a critical_section.
            // Return the magic inner-loop value so that we know not to re-enable interrupts in release()
            LOCK_ALREADY_OWNED
        } else {
            // Spin until we get the lock
            loop {
                // Need to disable interrupts to ensure that we will not deadlock
                // if an interrupt enters critical_section::Impl after we acquire the lock
                cortex_m::interrupt::disable();
                // Ensure the compiler doesn't re-order accesses and violate safety here
                core::sync::atomic::compiler_fence(Ordering::SeqCst);
                // Read the spinlock reserved for `critical_section`
                if let Some(lock) = Spinlock31::try_claim() {
                    // We just acquired the lock.
                    // 1. Forget it, so we don't immediately unlock
                    core::mem::forget(lock);
                    // 2. Store which core we are so we can tell if we're called recursively
                    LOCK_OWNER.store(core, Ordering::Relaxed);
                    break;
                }
                // We didn't get the lock, enable interrupts if they were enabled before we started
                if interrupts_active {
                    cortex_m::interrupt::enable();
                }
            }
            // If we broke out of the loop we have just acquired the lock
            // As the outermost loop, we want to return the interrupt status to restore later
            interrupts_active as _
        }
    }

    unsafe fn release(token: u8) {
        // Did we already own the lock at the start of the `critical_section`?
        if token != LOCK_ALREADY_OWNED {
            // No, it wasn't owned at the start of this `critical_section`, so this core no longer owns it.
            // Set `LOCK_OWNER` back to `LOCK_UNOWNED` to ensure the next critical section tries to obtain the spinlock instead
            LOCK_OWNER.store(LOCK_UNOWNED, Ordering::Relaxed);
            // Ensure the compiler doesn't re-order accesses and violate safety here
            core::sync::atomic::compiler_fence(Ordering::SeqCst);
            // Release the spinlock to allow others to enter critical_section again
            Spinlock31::release();
            // Re-enable interrupts if they were enabled when we first called acquire()
            // We only do this on the outermost `critical_section` to ensure interrupts stay disabled
            // for the whole time that we have the lock
            if token != 0 {
                cortex_m::interrupt::enable();
            }
        }
    }
}

/// Gets which core currently holds the spinlock. Useful in combination with
/// [`manually_release`] to determine if the current core needs to release the
/// critical section.
///
/// Returns `None` if neither core currenly holds the lock.
pub fn current_lock_owner() -> Option<CoreId> {
    match LOCK_OWNER.load(Ordering::Acquire) {
        1 => Some(CoreId::Core0),
        2 => Some(CoreId::Core1),
        _ => None,
    }
}

/// Allows the spinlock to be released manually. This is useful for example if
/// the panic handler is executed from within a critical section where there
/// would be no other way for the lock to be relased.
///
/// # Safety
///
/// Only call this function if you are sure that the current core holds
/// the lock and the critical section would never otherwise be released.
#[inline(always)]
pub unsafe fn manually_release(enable_interrupts: bool) {
    RpSpinlockCs::release(enable_interrupts as _);
}

pub struct Spinlock<const N: usize>(core::marker::PhantomData<()>)
where
    Spinlock<N>: SpinlockValid;

impl<const N: usize> Spinlock<N>
where
    Spinlock<N>: SpinlockValid,
{
    /// Try to claim the spinlock. Will return `Some(Self)` if the lock is obtained, and `None` if the lock is
    /// already in use somewhere else.
    pub fn try_claim() -> Option<Self> {
        let lock = pac::SIO.spinlock(N).read();
        if lock > 0 {
            Some(Self(core::marker::PhantomData))
        } else {
            None
        }
    }

    /// Clear a locked spin-lock.
    ///
    /// # Safety
    ///
    /// Only call this function if you hold the spin-lock.
    pub unsafe fn release() {
        // Write (any value): release the lock
        pac::SIO.spinlock(N).write_value(1);
    }
}

impl<const N: usize> Drop for Spinlock<N>
where
    Spinlock<N>: SpinlockValid,
{
    fn drop(&mut self) {
        // This is safe because we own the object, and hence hold the lock.
        unsafe { Self::release() }
    }
}

pub(crate) type Spinlock31 = Spinlock<31>;
pub trait SpinlockValid {}
impl SpinlockValid for Spinlock<31> {}
