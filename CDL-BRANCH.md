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

## What is on this branch, and which parts might interest you

Most of it is only meaningful on Genode. Four commits are defects in common
code that affect **every** platform, and those are worth a look regardless.

### Bugs that affect every platform — unguarded, upstream candidates

| commit | what |
|---|---|
| `dbc19e9dfb` | **The BSC record check reads PAD through a signed `char`**, so `BSC_tbuf[n-1] == PAD` is false for every real block and every one is NAKed. The function took `char*` while the caller passes a `uint8_t` array — gcc's signedness warning was the bug, not noise. |
| `13830b4a6f` | **The 100 ms timer handler assigns to register `0x7F`**, which is not the register it means. |
| `33548cd293` | **DLSw `accept()`s into the configured peer address struct**, overwriting the configuration with whoever connected. |
| `ffed317236` | **The level-3 idle gate tests request flags, not progress.** A flag nothing will service stays set, so a controller with no host on the channel spins: measured at 12.75 million level-3 passes a second, and 200% of a core on pristine upstream. Counting requests *cleared* rather than *pending* fixes it. |

### Genode platform support — guarded on `__GENODE__`

The sockets between the simulator's parts become Genode sessions: the channel
adapter (`f37b2daa00`), the lines (`1e051d5ecb`), the control panel
(`f575225b70`), `comm3705` — the Hercules end of the channel (`65f1bdb389`),
the 3274 (`9210b34693`), the 3271 (`ecb686d853`), the Trunk (`47edaf0c12`) and
DLSw (`5e454a5127`). Supporting changes: a portable thread id
(`b73df6dc74`), trace files when there is nowhere to put them (`3adca0280a`),
and terminal release when a client's socket dies (`9d0c2f213a`) — needed here
because Genode's libc `poll()` sets only `POLLIN`/`POLLOUT`, so a failed write
is the only way the controller learns a client has gone.

Then the timing work, which is where most of the effort went. A microkernel
gives no slack to a busy-wait: **one spinning component stretches every timed
wait in the system**, so loops that were harmless on Linux were not here.
`eb54edf2c0` paces the scanner through Genode's Timer session rather than the
libc (whose `usleep` floors at 1 ms); `bd44094111` blocks on the L2 handshake
on a condition variable instead of polling it — which took a remote NCP load
from 404 s to 174 s, against 256 s for the same load on native Linux;
`eab1f5c74b`, `ef2316aace` and `915d307a0b` rest the CPU, trunk and channel
threads when there is nothing to do; `baa8d6ea1e` grants CTS when RTS is
raised rather than on the next sweep. `c2e1a50121` and `af0c250cfb` are
instrumentation.

### Housekeeping

`d8d2ce5e3b` normalises line endings on two files. `e7e2e3b1e8` removes a
checked-in precompiled header, `sim_console.h.gch`: on a fresh clone it is
newer than `sim_console.h`, so gcc prefers it and edits to the header silently
have no effect.

## What this fork is not

It is **not** a proposed upstream series and has not been submitted. Prose
reports of the four platform-independent defects were drafted for the
maintainers but not filed. Issues and pull requests here are welcome, but this
is not upstream — file 3705 simulator bugs with
[snhstq/IBM3705_R5](https://github.com/snhstq/IBM3705_R5).
