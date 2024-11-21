#[doc = r" The RTIC application module"] pub mod app
{
    #[doc =
    r" Always include the device crate which contains the vector table"] use
    stm32f7 :: stm32f7x6 as
    you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml; pub use
    rtic :: Monotonic as _;
    #[doc = r" Holds static methods for each monotonic."] pub mod monotonics
    {
        pub use MyMono :: now;
        #[doc =
        "This module holds the static implementation for `MyMono::now()`"]
        #[allow(non_snake_case)] pub mod MyMono
        {
            #[doc = r" Read the current time from this monotonic"] pub fn
            now() -> < super :: super :: MyMono as rtic :: Monotonic > ::
            Instant
            {
                rtic :: export :: interrupt ::
                free(| _ |
                {
                    use rtic :: Monotonic as _; if let Some(m) = unsafe
                    {
                        & mut * super :: super ::
                        __rtic_internal_MONOTONIC_STORAGE_MyMono.get_mut()
                    } { m.now() } else
                    {
                        < super :: super :: MyMono as rtic :: Monotonic > :: zero()
                    }
                })
            }
        }
    } use super :: * ; #[doc = r" User code from within the module"] type
    MyMono = Systick < MONO_HZ > ; #[doc = r" User code end"]
    #[doc = " User provided init function"] #[inline(always)]
    #[allow(non_snake_case)] fn init(mut cx : init :: Context) ->
    (Shared, Local, init :: Monotonics)
    {
        rtt_init_print! (); rprintln! ("\n---init---\n"); let systick =
        cx.core.SYST; let mono = Systick :: new(systick, MONO_HZ);
        (Shared {}, Local {}, init :: Monotonics(mono))
    } #[doc = " User provided idle function"] #[allow(non_snake_case)] fn
    idle(_cx : idle :: Context) -> !
    {
        use rtic :: Mutex as _; use rtic :: mutex :: prelude :: * ; loop
        { cortex_m :: asm :: wfi(); }
    } #[doc = " RTIC shared resource struct"] struct Shared {}
    #[doc = " RTIC local resource struct"] struct Local {}
    #[doc = r" Monotonics used by the system"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct
    __rtic_internal_Monotonics(pub Systick < MONO_HZ >);
    #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct __rtic_internal_init_Context <
    'a >
    {
        #[doc = r" Core (Cortex-M) peripherals"] pub core : rtic :: export ::
        Peripherals, #[doc = r" Device peripherals"] pub device : stm32f7 ::
        stm32f7x6 :: Peripherals, #[doc = r" Critical section token for init"]
        pub cs : rtic :: export :: CriticalSection < 'a > ,
    } impl < 'a > __rtic_internal_init_Context < 'a >
    {
        #[doc(hidden)] #[inline(always)] pub unsafe fn
        new(core : rtic :: export :: Peripherals,) -> Self
        {
            __rtic_internal_init_Context
            {
                device : stm32f7 :: stm32f7x6 :: Peripherals :: steal(), cs :
                rtic :: export :: CriticalSection :: new(), core,
            }
        }
    } #[allow(non_snake_case)] #[doc = " Initialization function"] pub mod
    init
    {
        #[doc(inline)] pub use super :: __rtic_internal_Monotonics as
        Monotonics; #[doc(inline)] pub use super ::
        __rtic_internal_init_Context as Context;
    } #[doc = r" Execution context"] #[allow(non_snake_case)]
    #[allow(non_camel_case_types)] pub struct __rtic_internal_idle_Context < >
    {} impl < > __rtic_internal_idle_Context < >
    {
        #[doc(hidden)] #[inline(always)] pub unsafe fn
        new(priority : & rtic :: export :: Priority) -> Self
        { __rtic_internal_idle_Context {} }
    } #[allow(non_snake_case)] #[doc = " Idle loop"] pub mod idle
    {
        #[doc(inline)] pub use super :: __rtic_internal_idle_Context as
        Context;
    } #[doc = r" App module"] #[doc(hidden)] #[allow(non_upper_case_globals)]
    const __rtic_internal_MASK_CHUNKS : usize = rtic :: export ::
    compute_mask_chunks([]); #[doc(hidden)] #[allow(non_upper_case_globals)]
    const __rtic_internal_MASKS :
    [rtic :: export :: Mask < __rtic_internal_MASK_CHUNKS > ; 3] =
    [rtic :: export :: create_mask([]), rtic :: export :: create_mask([]),
    rtic :: export :: create_mask([])]; #[doc(hidden)]
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)] static
    __rtic_internal_TIMER_QUEUE_MARKER : rtic :: RacyCell < u32 > = rtic ::
    RacyCell :: new(0); #[doc(hidden)] #[allow(non_camel_case_types)]
    #[derive(Clone, Copy)] pub enum SCHED_T {} #[doc(hidden)]
    #[allow(non_camel_case_types)] #[allow(non_upper_case_globals)] static
    __rtic_internal_TQ_MyMono : rtic :: RacyCell < rtic :: export ::
    TimerQueue < Systick < MONO_HZ > , SCHED_T, 0 > > = rtic :: RacyCell ::
    new(rtic :: export ::
    TimerQueue(rtic :: export :: SortedLinkedList :: new_u16()));
    #[doc(hidden)] #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)] static
    __rtic_internal_MONOTONIC_STORAGE_MyMono : rtic :: RacyCell < Option <
    Systick < MONO_HZ > >> = rtic :: RacyCell :: new(None); #[no_mangle]
    #[allow(non_snake_case)] unsafe fn SysTick()
    {
        while let Some((task, index)) = rtic :: export :: interrupt ::
        free(| _ | if let Some(mono) =
        (& mut * __rtic_internal_MONOTONIC_STORAGE_MyMono.get_mut()).as_mut()
        {
            (& mut *
            __rtic_internal_TQ_MyMono.get_mut()).dequeue(|| core :: mem ::
            transmute :: < _, rtic :: export :: SYST >
            (()).disable_interrupt(), mono)
        } else { core :: hint :: unreachable_unchecked() }) { match task {} }
        rtic :: export :: interrupt ::
        free(| _ | if let Some(mono) =
        (& mut * __rtic_internal_MONOTONIC_STORAGE_MyMono.get_mut()).as_mut()
        { mono.on_interrupt(); });
    } #[doc(hidden)] mod rtic_ext
    {
        use super :: * ; #[no_mangle] unsafe extern "C" fn main() -> !
        {
            rtic :: export :: assert_monotonic :: < Systick < MONO_HZ > > ();
            const _CONST_CHECK : () =
            { if ! rtic :: export :: have_basepri() {} else {} }; let _ =
            _CONST_CHECK; rtic :: export :: interrupt :: disable(); let mut
            core : rtic :: export :: Peripherals = rtic :: export ::
            Peripherals :: steal().into(); const _ : () = if
            (1 << stm32f7 :: stm32f7x6 :: NVIC_PRIO_BITS) <
            (1 << stm32f7 :: stm32f7x6 :: NVIC_PRIO_BITS) as usize
            {
                :: core :: panic!
                ("Maximum priority used by monotonic 'MyMono' is more than supported by hardware");
            };
            core.SCB.set_priority(rtic :: export :: SystemHandler :: SysTick,
            rtic :: export ::
            logical2hw((1 << stm32f7 :: stm32f7x6 :: NVIC_PRIO_BITS), stm32f7
            :: stm32f7x6 :: NVIC_PRIO_BITS),); if ! < Systick < MONO_HZ > as
            rtic :: Monotonic > :: DISABLE_INTERRUPT_ON_EMPTY_QUEUE
            {
                core :: mem :: transmute :: < _, rtic :: export :: SYST >
                (()).enable_interrupt();
            } #[inline(never)] fn __rtic_init_resources < F > (f : F) where F
            : FnOnce() { f(); }
            __rtic_init_resources(||
            {
                let (shared_resources, local_resources, mut monotonics) =
                init(init :: Context :: new(core.into()));
                monotonics.0.reset();
                __rtic_internal_MONOTONIC_STORAGE_MyMono.get_mut().write(Some(monotonics.0));
                rtic :: export :: interrupt :: enable();
            });
            idle(idle :: Context ::
            new(& rtic :: export :: Priority :: new(0)))
        }
    }
}