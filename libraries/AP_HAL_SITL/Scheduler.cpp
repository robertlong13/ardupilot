#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_SITL.h"
#include <AP_HAL_SITL/I2CDevice.h>
#include "Scheduler.h"
#include "UARTDriver.h"
#include <sys/time.h>
#include <fenv.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#if defined (__clang__) || (defined (__APPLE__) && defined (__MACH__)) || defined (__OpenBSD__)
#include <stdlib.h>
#else
#include <malloc.h>
#endif
#include <AP_RCProtocol/AP_RCProtocol.h>
#ifdef UBSAN_ENABLED
#include <fcntl.h>
#include <sanitizer/asan_interface.h>
#endif
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <errno.h>
#if HAL_LOGGING_ENABLED
#include <AP_Logger/AP_Logger.h>
#endif

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

#ifndef SITL_STACK_CHECKING_ENABLED
//#define SITL_STACK_CHECKING_ENABLED !defined(__CYGWIN__) && !defined(__CYGWIN64__)
// stack checking is disabled until the memory corruption issues are
// fixed with pthread_attr_setstack.  These may be due to
// changes in the way guard pages are handled.
#define SITL_STACK_CHECKING_ENABLED 0
#endif

AP_HAL::Proc Scheduler::_failsafe = nullptr;

AP_HAL::MemberProc Scheduler::_timer_proc[SITL_SCHEDULER_MAX_TIMER_PROCS] = {nullptr};
uint8_t Scheduler::_num_timer_procs = 0;
bool Scheduler::_in_timer_proc = false;

AP_HAL::MemberProc Scheduler::_io_proc[SITL_SCHEDULER_MAX_TIMER_PROCS] = {nullptr};
uint8_t Scheduler::_num_io_procs = 0;
bool Scheduler::_in_io_proc = false;
bool Scheduler::_should_exit = false;

bool Scheduler::_in_semaphore_take_wait = false;

Scheduler::thread_attr *Scheduler::threads;
HAL_Semaphore Scheduler::_thread_sem;

volatile Scheduler::CheckpointOp Scheduler::_checkpoint_op = Scheduler::CheckpointOp::NONE;
volatile bool Scheduler::_barrier_engaged;
volatile uint32_t Scheduler::_threads_parked;
pid_t Scheduler::_savepoint_pid = -1;
int Scheduler::_restore_pipe[2] = { -1, -1 };

Scheduler::Scheduler(SITL_State *sitlState) :
    _sitlState(sitlState),
    _stopped_clock_usec(0)
{
}

#ifdef UBSAN_ENABLED
/*
  catch ubsan errors and append to a log file
 */
extern "C" {
void __ubsan_get_current_report_data(const char **OutIssueKind,
                                     const char **OutMessage,
                                     const char **OutFilename, unsigned *OutLine,
                                     unsigned *OutCol, char **OutMemoryAddr);

void __ubsan_on_report();
void __ubsan_on_report()
{
    static int fd = -1;
    if (fd == -1) {
        const char *ubsan_log_path = getenv("UBSAN_LOG_PATH");
        if (ubsan_log_path == nullptr) {
            ubsan_log_path = "ubsan.log";
        }
        if (ubsan_log_path != nullptr) {
            fd = open(ubsan_log_path, O_APPEND|O_CREAT|O_WRONLY, 0644);
        }
    }
    if (fd != -1) {
        const char *OutIssueKind = nullptr;
        const char *OutMessage = nullptr;
        const char *OutFilename = nullptr;
        unsigned OutLine=0;
        unsigned OutCol=0;
        char *OutMemoryAddr=nullptr;
        __ubsan_get_current_report_data(&OutIssueKind, &OutMessage, &OutFilename,
                                        &OutLine, &OutCol, &OutMemoryAddr);
        dprintf(fd, "ubsan error: %s:%u:%u %s:%s\n",
                OutFilename, OutLine, OutCol,
                OutIssueKind, OutMessage);
    }
}
}
#endif

void Scheduler::init()
{
    _main_ctx = pthread_self();
}

bool Scheduler::in_main_thread() const
{
    if (!_in_timer_proc && !_in_io_proc && pthread_self() == _main_ctx) {
        return true;
    }
    return false;
}

/*
 * semaphore_wait_hack_required - possibly move time input step
 * forward even if we are currently pretending to be the IO or timer
 * threads.
 *
 * Without this, if another thread has taken a semaphore (e.g. the
 * Object Avoidance thread), and an "IO process" tries to take that
 * semaphore with a timeout specified, then we end up not advancing
 * time (due to the logic in SITL_State::wait_clock) and thus taking
 * the semaphore never times out - meaning we essentially deadlock.
 */
bool Scheduler::semaphore_wait_hack_required() const
{
    if (pthread_self() != _main_ctx) {
        // only the main thread ever moves stuff forwards
        return false;
    }

    return _in_semaphore_take_wait;
}

void Scheduler::delay_microseconds(uint16_t usec)
{
    if (_sitlState->_sitl == nullptr) {
        // this allows examples to run
        hal.scheduler->stop_clock(AP_HAL::micros64()+usec);
        return;
    }
    uint64_t start = AP_HAL::micros64();
    do {
        uint64_t dtime = AP_HAL::micros64() - start;
        if (dtime >= usec) {
            break;
        }
        _sitlState->wait_clock(start + usec);
    } while (true);
}

void Scheduler::delay(uint16_t ms)
{
    uint32_t start = AP_HAL::millis();
    uint32_t now = start;
    do {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= (ms - (now - start))) {
            if (in_main_thread()) {
                call_delay_cb();
            }
        }
        now = AP_HAL::millis();
    } while (now - start < ms);
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < SITL_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    }
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < SITL_SCHEDULER_MAX_TIMER_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    }
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::set_system_initialized() {
    if (_initialized) {
        AP_HAL::panic(
            "PANIC: scheduler system initialized called more than once");
    }
    int exceptions = FE_OVERFLOW | FE_DIVBYZERO;
#ifndef __i386__
    // i386 with gcc doesn't work with FE_INVALID
    exceptions |= FE_INVALID;
#endif
#if !defined(HAL_BUILD_AP_PERIPH)
    if (_sitlState->_sitl == nullptr || _sitlState->_sitl->float_exception) {
        feenableexcept(exceptions);
    } else {
        feclearexcept(exceptions);
    }
#else
    feclearexcept(exceptions);
#endif
    _initialized = true;
}

void Scheduler::sitl_end_atomic() {
    if (_nested_atomic_ctr == 0) {
        hal.serial(0)->printf("NESTED ATOMIC ERROR\n");
    } else {
        _nested_atomic_ctr--;
    }
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    HAL_SITL::actually_reboot();
    abort();
}

void Scheduler::_run_timer_procs()
{
    if (_in_timer_proc) {
        // the timer calls took longer than the period of the
        // timer. This is bad, and may indicate a serious
        // driver failure. We can't just call the drivers
        // again, as we could run out of stack. So we only
        // call the _failsafe call. It's job is to detect if
        // the drivers or the main loop are indeed dead and to
        // activate whatever failsafe it thinks may help if
        // need be.  We assume the failsafe code can't
        // block. If it does then we will recurse and die when
        // we run out of stack
        if (_failsafe != nullptr) {
            _failsafe();
        }
        return;
    }
    _in_timer_proc = true;

    // now call the timer based drivers
    for (int i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i]) {
            _timer_proc[i]();
        }
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }

    _in_timer_proc = false;
}

void Scheduler::_run_io_procs()
{
    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;

    // now call the IO based drivers
    for (int i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i]) {
            _io_proc[i]();
        }
    }

    _in_io_proc = false;

    for (uint8_t i=0; i<hal.num_serial; i++) {
        hal.serial(i)->_timer_tick();
    }
    hal.storage->_timer_tick();

    // in lieu of a thread-per-bus:
    ((HALSITL::I2CDeviceManager*)(hal.i2c_mgr))->_timer_tick();

#if SITL_STACK_CHECKING_ENABLED
    check_thread_stacks();
#endif

#if AP_RCPROTOCOL_ENABLED
    AP::RC().update();
#endif
}

/*
  set simulation timestamp
 */
void Scheduler::stop_clock(uint64_t time_usec)
{
    _stopped_clock_usec = time_usec;
    if (_sitlState->_sitl != nullptr && time_usec - _last_io_run > 10000) {
        _last_io_run = time_usec;
        _run_io_procs();
    }
}

/*
  SITL quicksave/quickload implementation.

  A checkpoint is a fork() taken while every non-main (helper) thread is
  parked at a known lock-free point. fork() copies the whole address space
  but only the calling (main) thread, so on restore we deliberately respawn
  the helper threads from the registry rather than relying on fork to carry
  them - this is what makes it portable (e.g. Cygwin) where carrying threads
  across fork is fragile.
 */

// number of registered (non-main) threads in the list
uint32_t Scheduler::count_threads()
{
    WITH_SEMAPHORE(_thread_sem);
    uint32_t n = 0;
    for (struct thread_attr *a = threads; a != nullptr; a = a->next) {
        n++;
    }
    return n;
}

// cooperative park point, called by non-main threads from wait_clock(). This
// is the existing 10us poll loop, so parking here holds no locks.
void Scheduler::checkpoint_park_point()
{
    if (!_barrier_engaged) {
        return;
    }
    __atomic_add_fetch(&_threads_parked, 1, __ATOMIC_SEQ_CST);
    while (_barrier_engaged) {
        usleep(50);
    }
    __atomic_sub_fetch(&_threads_parked, 1, __ATOMIC_SEQ_CST);
}

// serviced on the main thread from the top-level loop (a lock-free point)
void Scheduler::service_checkpoint()
{
    const CheckpointOp op = _checkpoint_op;
    if (op == CheckpointOp::NONE) {
        return;
    }
    _checkpoint_op = CheckpointOp::NONE;
    if (op == CheckpointOp::SAVE) {
        do_quicksave();
    } else {
        do_quickload();
    }
}

void Scheduler::do_quicksave()
{
    // single quicksave slot: discard any previous savepoint
    if (_savepoint_pid > 0) {
        kill(_savepoint_pid, SIGKILL);
        waitpid(_savepoint_pid, nullptr, 0);
        _savepoint_pid = -1;
    }
    if (_restore_pipe[0] != -1) { close(_restore_pipe[0]); _restore_pipe[0] = -1; }
    if (_restore_pipe[1] != -1) { close(_restore_pipe[1]); _restore_pipe[1] = -1; }
    if (pipe(_restore_pipe) != 0) {
        return;
    }

    // quiesce every helper thread at its cooperative park point
    const uint32_t n = count_threads();
    _threads_parked = 0;
    _barrier_engaged = true;
    while (__atomic_load_n(&_threads_parked, __ATOMIC_SEQ_CST) < n) {
        usleep(50);
    }

    // only the main thread is live now; helpers spin in checkpoint_park_point()
    // holding no locks. Fork a frozen copy.
    const pid_t pid = fork();
    if (pid < 0) {
        _barrier_engaged = false;
        return;
    }
    if (pid > 0) {
        // parent = the live process; keeps flying
        _savepoint_pid = pid;
        close(_restore_pipe[0]);
        _restore_pipe[0] = -1;
        _barrier_engaged = false;       // release helpers
        return;
    }

    // child = frozen savepoint. Only the calling thread exists here.
    close(_restore_pipe[1]);
    _restore_pipe[1] = -1;

    // Re-arm the savepoint on every quickload so the same save can be loaded
    // repeatedly (until the next quicksave). When woken we first fork a fresh
    // frozen copy of this saved state - still single-threaded, before any
    // respawn or semaphore re-base, so it is identical to the original
    // savepoint - to serve the next quickload, then this process goes live.
    while (true) {
        freeze_until_restore();         // block until a quickload wakes us

        if (pipe(_restore_pipe) != 0) {
            _restore_pipe[0] = -1;
            _restore_pipe[1] = -1;
            _savepoint_pid = -1;
            break;                      // can't re-arm; go live without a savepoint
        }
        const pid_t child_pid = fork();
        if (child_pid < 0) {
            close(_restore_pipe[0]); _restore_pipe[0] = -1;
            close(_restore_pipe[1]); _restore_pipe[1] = -1;
            _savepoint_pid = -1;
            break;                      // fork failed; go live without a savepoint
        }
        if (child_pid == 0) {
            // the replacement savepoint: keep the read end, loop back to freeze
            close(_restore_pipe[1]);
            _restore_pipe[1] = -1;
            continue;
        }
        // we are the one going live; keep the write end to wake the
        // replacement on the next quickload
        _savepoint_pid = child_pid;
        close(_restore_pipe[0]);
        _restore_pipe[0] = -1;
        break;
    }

    // become the live process at the saved state. Re-base the semaphores first:
    // this thread has a new TID, so any mutex held across the fork (e.g. the
    // scheduler loop sem) must be reset before we touch any lock.
    Semaphore::reinit_after_fork();
    _barrier_engaged = false;           // clear before respawning
    // do the fd surgery (sockets, log rewind) while still single-threaded so
    // it can't race the respawned log_io thread writing to the same log fd
    reinit_io_after_restore();
    respawn_threads();
}

void Scheduler::freeze_until_restore()
{
    // block until the parent writes a wake byte (quickload) or closes the
    // pipe (parent exited without loading -> discard this stale savepoint)
    uint8_t b;
    ssize_t r;
    do {
        r = read(_restore_pipe[0], &b, 1);
    } while (r < 0 && errno == EINTR);
    if (r <= 0) {
        _exit(0);
    }
    close(_restore_pipe[0]);
    _restore_pipe[0] = -1;
}

void Scheduler::do_quickload()
{
    if (_savepoint_pid <= 0 || _restore_pipe[1] == -1) {
        return;                         // nothing saved
    }
    // wake the child and disappear; the child's inherited listen sockets stay
    // bound, so GCS/MAVProxy reconnect to it once we exit.
    const uint8_t b = 1;
    IGNORE_RETURN(write(_restore_pipe[1], &b, 1));
    _exit(0);
}

void Scheduler::respawn_threads()
{
    // re-create a live pthread for each registered thread; each re-enters its
    // MemberProc loop. Heap/member state is intact (copied by fork); only
    // per-thread C-stack state is reset.
    for (struct thread_attr *a = threads; a != nullptr; a = a->next) {
        if (pthread_create(&a->thread, &a->attr, thread_create_trampoline, a) != 0) {
            AP_HAL::panic("checkpoint: failed to respawn thread %s", a->name);
        }
#if !defined(__APPLE__) && !defined(__OpenBSD__)
        pthread_setname_np(a->thread, a->name);
#endif
    }
}

void Scheduler::reinit_io_after_restore()
{
    // the original launcher is gone now that the live parent has exited;
    // stop the parent-death watchdog from killing this resumed savepoint
    _sitlState->checkpoint_orphan();

    // drop the stale client connections inherited from the parent so
    // GCS/MAVProxy reconnect to the (still-bound, inherited) listen socket
    for (uint8_t i=0; i<hal.num_serial; i++) {
        ((HALSITL::UARTDriver*)hal.serial(i))->checkpoint_reset_connection();
    }
#if HAL_LOGGING_ENABLED
    // rewind the dataflash log to the quicksave point and drop the tail the
    // parent appended (we shared its write fd offset)
    AP::logger().checkpoint_rewind();
#endif
}

/*
  trampoline for thread create
*/
void *Scheduler::thread_create_trampoline(void *ctx)
{
    struct thread_attr *a = (struct thread_attr *)ctx;
    a->thread = pthread_self();
    a->f[0]();
    
    WITH_SEMAPHORE(_thread_sem);
    if (threads == a) {
        threads = a->next;
    } else {
        for (struct thread_attr *p=threads; p->next; p=p->next) {
            if (p->next == a) {
                p->next = p->next->next;
                break;
            }
        }
    }
    free(a->stack);
    free(a->f);
    delete a;
    return nullptr;
}

#ifndef PTHREAD_STACK_MIN
#define PTHREAD_STACK_MIN 16384U
#endif

/*
  create a new thread
*/
bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char *name, uint32_t stack_size, priority_base base, int8_t priority)
{
    WITH_SEMAPHORE(_thread_sem);

    // even an empty thread takes 2500 bytes on Linux, so always add 2300, giving us 200 bytes
    // safety margin
    stack_size += 2300;
    
    pthread_t thread {};
    const uint32_t alloc_stack = MAX(size_t(PTHREAD_STACK_MIN),stack_size);

    struct thread_attr *a = NEW_NOTHROW struct thread_attr;
    if (!a) {
        return false;
    }
    // take a copy of the MemberProc, it is freed after thread exits
    a->f = (AP_HAL::MemberProc *)malloc(sizeof(proc));
    if (!a->f) {
        goto failed;
    }
    if (posix_memalign(&a->stack, 4096, alloc_stack) != 0) {
        goto failed;
    }
    if (!a->stack) {
        goto failed;
    }
    memset(a->stack, stackfill, alloc_stack);
    a->stack_min = (const uint8_t *)((((uint8_t *)a->stack) + alloc_stack) - stack_size);

    a->stack_size = stack_size;
    a->f[0] = proc;
    a->name = name;

    if (pthread_attr_init(&a->attr) != 0) {
        goto failed;
    }
#if SITL_STACK_CHECKING_ENABLED
    if (pthread_attr_setstack(&a->attr, a->stack, alloc_stack) != 0) {
        AP_HAL::panic("Failed to set stack of size %u for thread %s", alloc_stack, name);
    }
#endif
    if (pthread_create(&thread, &a->attr, thread_create_trampoline, a) != 0) {
        goto failed;
    }

#if !defined(__APPLE__) && !defined(__OpenBSD__)
    pthread_setname_np(thread, name);
#endif

    a->next = threads;
    threads = a;
    return true;

failed:
    if (a->stack) {
        free(a->stack);
    }
    if (a->f) {
        free(a->f);
    }
    delete a;
    return false;
}

/*
  check for stack overflow
 */
void Scheduler::check_thread_stacks(void)
{
    WITH_SEMAPHORE(_thread_sem);
    for (struct thread_attr *p=threads; p; p=p->next) {
        const uint8_t ncheck = 8;
        for (uint8_t i=0; i<ncheck; i++) {
            if (p->stack_min[i] != stackfill) {
                AP_HAL::panic("stack overflow in thread %s", p->name);
            }
        }
    }
}

// get the name of the current thread, or nullptr if not known
const char *Scheduler::get_current_thread_name(void) const
{
    const pthread_t self = pthread_self();
    for (struct thread_attr *a=threads; a; a=a->next) {
        if (a->thread == self) {
            return a->name;
        }
    }
    return nullptr;
}
