# The `cdl/main` branch

**If you arrived here from a binary or a source offer, this is the branch that
binary was built from.**

This is a fork of [IBM3705_R5](https://github.com/snhstq/IBM3705_R5) by Henk
Stegeman and Edwin Freekenhorst — a SIMH-based simulator of the **IBM 3705
communications controller**, the programmable box that front-ended a
mainframe's terminal and network links. `cdl/main` carries changes made by
**Carrier Detect Labs** to run it on **[Genode](https://genode.org)**, an OS
framework in which every program is a component with no ambient authority,
running on a microkernel rather than a general-purpose OS. It runs there beside
a Genode port of the [Hercules](https://github.com/SDL-Hercules-390/hyperion)
mainframe emulator, which is what the 3705 is attached to.

`main` is upstream, untouched. The base of this branch is `73994e8`.

Licence is unchanged: **GPL-3.0**, as upstream.

## Reading a commit here

The messages carry the reasoning and the measurements, and were written for
people working on this port. Terms that recur:

* **Genode session** — a connection to a service, granted by policy; the
  substitute for opening a socket. **"over IPC"** means carried that way
  instead of over TCP.
* **`terminal_crosslink`** — a Genode component joining exactly two clients so
  each reads what the other writes. A cable, not a connection.
* **NCP** (Network Control Program) — the software the 3705 runs. It is *loaded
  into* the controller by the mainframe, over the channel, at startup.
* **CCW** — Channel Command Word, one I/O instruction; a channel program is a
  chain of them. **L2/L3** are the simulated controller's program levels,
  roughly interrupt priorities.
* **BSC** and **SDLC** — the two link protocols on the lines. **3271/3274** are
  terminal cluster controllers; **3270** is the block-mode terminal itself.
* **DLSw** — Data Link Switching, SNA carried over IP. **Trunk** — the link
  between two 3705s.
* **genherc** — the target machine these changes were measured on; a small
  x86-64 box booting Genode/NOVA with no general-purpose OS.

## Two commit messages here are wrong

Kept rather than rebased away, because what a message got wrong is worth
knowing when you weigh the others.

* **`43a94b7`** replaced eight hardcoded time-stamp-counter divisors with a
  calibrated rate, and describes those literals as though they were found in
  somebody else's code. They were this branch's. `7509178` added the
  calibration routine *and* the first literal in the same commit; the
  reporting subsystem they belong to does not exist upstream at all. So its
  "generally applicable, not specific to any platform" is wrong — there is no
  upstream code for it to apply to — and "the calibration has been in this
  file all along" means since we added it, not before. `fb0b7b3` carries the
  correction.
* **`455cdec`**, the third of its three defects, says the adapter's
  read-modify-write "brackets a wait: it loads the register, waits half a
  millisecond or more, and then stores what it loaded". It does not. That
  statement sits *after* the wait loop, and `wait()` is an opaque call across
  which no compiler may keep a global in a register, so the window is the same
  two or three instructions as every other site. The race is real and the lock
  it added is right; its width was overstated, and the consequence is that the
  *second* defect, the latch, is the one that fits the observed failure. All
  three went in together, so which one parked a given load was never isolated.

**And the commit hashes in this file were stale for three weeks.** Every one
of them named a commit from before this branch was last rebased, so none
resolved in a fresh clone — in a file whose whole purpose is to orient someone
who cannot ask us. They are re-anchored now. If you cite a commit in prose,
cite its subject line too: subjects survive a rebase and hashes do not.

## What is on this branch, and which parts might interest you

Most of it is only meaningful on Genode. Five commits are defects in common
code that affect **every** platform, and those are worth a look regardless.

### Bugs that affect every platform — unguarded, upstream candidates

| commit | what |
|---|---|
| `01b826c` | **The BSC record check reads PAD through a signed `char`**, so `BSC_tbuf[n-1] == PAD` is false for every real block and every one is NAKed. The function took `char*` while the caller passes a `uint8_t` array — gcc's signedness warning was the bug, not noise. |
| `a034af4` | **The 100 ms timer handler assigns to register `0x7F`**, which is not the register it means. |
| `96a1ee5` | **DLSw `accept()`s into the configured peer address struct**, overwriting the configuration with whoever connected. |
| `455cdec` | **Three wrong orderings in the handshake between the channel adapter and the emulated processor.** A level-3 request flag stored outside the lock that guards the register it is read with, on both sides. Two control-word-address valid bits that are latches — upstream's own comment calls one of them that, and register X'56' exists for nothing but resetting them — which a whole-register `OUT X'55'` was clearing. And seven unsynchronised read-modify-writes of that register while two threads do them. Together they can leave the adapter in an unbounded wait for a bit nothing will set, with the processor idle, no interrupt raised and nothing printed: one load in five on a fast host, and the mainframe's missing-interrupt handler is the only thing that reports it. The latch is the one that fires — 180 of 192 image stores in a single load carried an image omitting a bit that was armed at the time. |
| `891cfe5` | **The level-3 idle gate tests request flags, not progress.** A flag nothing will service stays set, so a controller with no host on the channel spins: measured at 12.75 million level-3 passes a second, and 200% of a core on pristine upstream. Counting requests *cleared* rather than *pending* fixes it. |

### Genode platform support — guarded on `__GENODE__`

The sockets between the simulator's parts become Genode sessions: the channel
adapter (`0438858`), the lines (`f79477b`), the control panel
(`e5f8ffa`), `comm3705` — the Hercules end of the channel (`0e78f49`),
the 3274 (`652220b`), the 3271 (`21fa0c4`), the Trunk (`659aed1`) and
DLSw (`b9c7aa0`). Supporting changes: a portable thread id
(`177cbbe`), trace files when there is nowhere to put them (`5694d81`),
and terminal release when a client's socket dies (`54ec2cb`) — needed here
because Genode's libc `poll()` sets only `POLLIN`/`POLLOUT`, so a failed write
is the only way the controller learns a client has gone.

Then the timing work, which is where most of the effort went. A microkernel
gives no slack to a busy-wait: **one spinning component stretches every timed
wait in the system**, so loops that were harmless on Linux were not here.
`7509178` paces the scanner through Genode's Timer session rather than the
libc (whose `usleep` floors at 1 ms); `ab67d24` blocks on the L2 handshake
on a condition variable instead of polling it — which took a remote NCP load
from 404 s to 174 s, against 256 s for the same load on native Linux;
`75b96c0`, `7518fc8` and `6deee7e` rest the CPU, trunk and channel
threads when there is nothing to do; `60c6994` grants CTS when RTS is
raised rather than on the next sweep. `3d1b0c1` and `c97f3e9` are
instrumentation, as are `b84f773`, which names which of the channel adapter's
twenty-two unbounded interlocks a stalled adapter is spinning in and reports
it after two seconds, and `29bb9d5` with `081d82b`, which count how often the
latch fix in `455cdec` actually fires and then stop printing that count on
every sample — printing it per sample cost 2.6% of the scanner's rate beside a
core that was saturated, while the counting itself cost nothing measurable.

### Housekeeping

`a244e17` normalises line endings on two files. `ef31f4b` removes a
checked-in precompiled header, `sim_console.h.gch`: on a fresh clone it is
newer than `sim_console.h`, so gcc prefers it and edits to the header silently
have no effect.

## What this fork is not

It is **not** a proposed upstream series and has not been submitted. Prose
reports of the four platform-independent defects were drafted for the
maintainers but not filed. Issues and pull requests here are welcome, but this
is not upstream — file 3705 simulator bugs with
[snhstq/IBM3705_R5](https://github.com/snhstq/IBM3705_R5).
