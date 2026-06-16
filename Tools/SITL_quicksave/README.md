# SITL quicksave / quickload

Fork-based state checkpoint for **headless** SITL (built-in physics only — not
RealFlight or other external FDMs). `quicksave` forks a frozen copy of the
running sim; `quickload` discards the live process and resumes from the frozen
copy. The same save can be loaded **repeatedly** until the next `quicksave`.

Branch: `sitl-quicksave-checkpoint` (off `master`).

## Trigger

Reuses the `PREFLIGHT_REBOOT_SHUTDOWN` developer magic sequence. SITL-only,
handled *before* the armed check so it works in flight:

    MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN  param1=42 param2=24 param3=71
        param4=110  -> quicksave
        param4=111  -> quickload

From MAVProxy:

    long PREFLIGHT_REBOOT_SHUTDOWN 42 24 71 110 0 0 0   # quicksave
    long PREFLIGHT_REBOOT_SHUTDOWN 42 24 71 111 0 0 0   # quickload

## Helper scripts

`quicksave.py` / `quickload.py` send those commands over MAVLink. They listen on
`udpin:0.0.0.0:14553`, so point a SITL serial at that port:

    --serial2 udpclient:127.0.0.1:14553

Then `python3 Tools/SITL_quicksave/quicksave.py` (and `quickload.py`).

## How it works

- `quicksave` quiesces the helper threads at the `wait_clock` barrier, then
  `fork()`s. The child freezes (blocked on a pipe); the parent flies on.
- `quickload` writes the pipe and `_exit()`s the parent. The child wakes,
  re-arms a fresh savepoint (forks again at the same saved state so the save
  stays loadable), then goes live: re-bases the semaphores (new thread TID),
  rewinds/truncates the dataflash log to the save point, drops stale GCS
  sockets, and respawns its helper threads from the registry.
- Lua: the script VM is **resumed**, not rebuilt, so all script state is kept.

## Commits

Four base commits plus fixups — fold them in with
`git rebase -i --autosquash`:

- `AP_Logger:` checkpoint_rewind — rewind/truncate the log to the save point
- `AP_Scripting:` resume VM — re-enter the script loop on the preserved VM
- `AP_HAL_SITL:` checkpoint — barrier + fork + resume machinery
- `GCS_MAVLink:` trigger — the magic-number command

## Gotchas / TODO

- Verified working headless, including Lua state restore across repeated loads
  (counter + path-dependent checksum jump back, VM not rebuilt). **Cygwin
  standalone is unverified** — fork+threads there is the main risk.
- Resumed children are **immortal** (the parent-death watchdog is pointed at
  self so they survive quickload). Trade-off: orphaned sims when a session ends.
  Clean up with `pkill -x arduplane`. Frozen savepoints self-clean via pipe EOF;
  only the live resumed process lingers.
- Debugger: set `follow-fork-mode child` so it follows across the fork (already
  in the quadplane launch config). Without it, the live process exiting on
  quickload ends the debug session and orphans the resumed child.
- Not done yet: a lock-free-park assert in the barrier (tripwire if a future
  thread ever parks holding a HAL semaphore).
