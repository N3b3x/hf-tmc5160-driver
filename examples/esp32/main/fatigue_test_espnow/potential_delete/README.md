## potential_delete/

Deprecated app-level bounds finding implementation.

The fatigue test unit now uses the **driver library built-in** homing/bounds subsystem:

- `g_driver->homing.FindBounds(...)` (StallGuard / Encoder / Switch)

These files are kept temporarily for reference (and for porting any extra debug logging):

- `bounds_finder.hpp`
- `bounds_finder_stallguard.cpp`
- `bounds_finder_encoder.cpp`


