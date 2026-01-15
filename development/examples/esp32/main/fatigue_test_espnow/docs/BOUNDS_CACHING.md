# Bounds Finding Cache System

## Overview

The Bounds Caching System allows independent bounds finding, keeping the motor ready for immediate test starts. This improves workflow efficiency by eliminating redundant bounds finding when running multiple tests in succession.

## Key Features

1. **Independent Bounds Finding**: Run `bounds` command separately from test start
2. **Time-Based Validity**: Bounds remain valid for a configurable window (default: 2 minutes)
3. **Motor State Management**: Motor stays energized during validity window, de-energizes after timeout
4. **Automatic De-energization**: Prevents motor overheating during idle periods
5. **Skip Redundant Finding**: `start` command uses cached bounds if still valid

---

## Commands

### `bounds`

Run bounds finding independently without starting the test.

```
> bounds
╔══════════════════════════════════════════════════════════════════════════════╗
║                              BOUNDS FINDING                                   ║
╠══════════════════════════════════════════════════════════════════════════════╣
║ Bounds finding started...                                                    ║
║ Motor will stay energized after completion.                                  ║
║ Validity window: 2 minutes                                                   ║
║                                                                              ║
║ Run 'start' within validity window to skip bounds finding.                   ║
╚══════════════════════════════════════════════════════════════════════════════╝
```

### `start` (with cache)

When bounds are cached and valid:

```
> start
[FatigueTestUnit] Using cached bounds (valid for 85 more seconds)
[FatigueTestUnit] Test started immediately (no bounds finding)
```

When bounds are expired or not found:

```
> start
[FatigueTestUnit] Bounds not cached or expired - running bounds finding
[FatigueTestUnit] Starting bounds finding...
```

### `status` (shows cache info)

```
> status
╔══════════════════════════════════════════════════════════════════════════════╗
║                              MOTION STATUS                                    ║
╠══════════════════════════════════════════════════════════════════════════════╣
║ State:              IDLE                                                     ║
║ Bounded:            YES                                                      ║
║ Frequency:          0.50 Hz                                                  ║
║ ...                                                                          ║
║                                                                              ║
║ BOUNDS CACHE:                                                                ║
║   Status:           Valid (85 sec remaining)                                 ║
║   Motor:            Energized (ready for start)                              ║
╚══════════════════════════════════════════════════════════════════════════════╝
```

---

## State Diagram

```
                          ┌─────────────────────┐
                          │ Motor De-energized  │
                          │  Bounds Invalid     │
                          └──────────┬──────────┘
                                     │
           ┌─────────────────────────┼─────────────────────────┐
           │                         │                         │
    bounds command             start command           config change
           │                         │                         │
           ▼                         ▼                         │
    ┌──────────────┐          ┌──────────────┐                 │
    │ BOUNDS       │          │ BOUNDS       │                 │
    │ FINDING      │          │ FINDING      │                 │
    └──────┬───────┘          └──────┬───────┘                 │
           │                         │                         │
    bounds found              bounds found                     │
           │                         │                         │
           ▼                         ▼                         │
    ┌──────────────────┐      ┌──────────────┐                 │
    │ IDLE             │      │ RUNNING      │                 │
    │ Motor Energized  │      │ Test Active  │                 │
    │ Timer Started    │      │ Timer Paused │                 │
    └────────┬─────────┘      └──────┬───────┘                 │
             │                       │                         │
     ┌───────┴───────┐        test complete                    │
     │               │               │                         │
 timeout          start              │                         │
   2min          command             │                         │
     │               │               │                         │
     ▼               ▼               ▼                         │
┌────────────┐  ┌──────────┐  ┌──────────────┐                 │
│ Motor      │  │ RUNNING  │  │ Motor        │                 │
│ De-energized│  │ (no      │  │ De-energized │                 │
│ Bounds     │  │ bounds   │  │ Bounds Valid │                 │
│ Expired    │  │ needed)  │  │ (for next    │                 │
└────────────┘  └──────────┘  │ quick start) │                 │
                              └──────┬───────┘                 │
                                     │                         │
                                timeout or─────────────────────┘
                                next start
```

---

## Configuration

### Default Validity Window

```cpp
namespace BoundsCache {
    static constexpr uint32_t DEFAULT_VALIDITY_MINUTES = 2;
}
```

### Changing Validity at Runtime

```cpp
// Set validity to 5 minutes
BoundsCache::SetValidityMinutes(5);
```

### Compile-Time Configuration

Modify `DEFAULT_VALIDITY_MINUTES` in `main.cpp` to change the default:

```cpp
static constexpr uint32_t DEFAULT_VALIDITY_MINUTES = 5; // 5 minutes
```

---

## API Reference

### BoundsCache Namespace

| Function | Description |
|----------|-------------|
| `Init()` | Initialize the bounds cache system (create timer) |
| `AreBoundsValid()` | Check if cached bounds are still valid |
| `GetRemainingValiditySec()` | Get seconds remaining in validity window |
| `MarkBoundsFound()` | Mark bounds as found, start de-energize timer |
| `CancelDeenergizeTimer()` | Cancel timer (e.g., when test starts) |
| `InvalidateBounds()` | Force invalidation (e.g., on config change) |
| `SetValidityMinutes(uint32_t)` | Change validity window |

### Global Variables

| Variable | Description |
|----------|-------------|
| `g_bounds_validity_us` | Validity window in microseconds |
| `g_bounds_timestamp_us` | Timestamp when bounds were last found |
| `g_motor_energized_for_bounds` | Whether motor is energized from bounds finding |
| `g_deenergize_timer` | ESP timer handle for de-energize callback |

---

## Workflow Examples

### Quick Iteration Workflow

For testing different settings rapidly:

```bash
# 1. Find bounds once
> bounds

# 2. Run first test
> start

# 3. Wait for completion or stop
> stop

# 4. Adjust settings
> set -f 0.7

# 5. Start again immediately (bounds still valid)
> start
```

### Standard Test Workflow

For production testing:

```bash
# 1. Configure test parameters
> set -f 0.5 -c 1000 -b -60 60

# 2. Start test (bounds found automatically)
> start

# 3. Monitor progress
> status
```

### Pre-Flight Check Workflow

For verification before actual testing:

```bash
# 1. Run bounds to verify mechanism works
> bounds

# 2. Check status shows valid bounds
> status

# 3. If satisfied, start test
> start
```

---

## Timing Details

### De-energize Timer

- **Type**: One-shot ESP timer
- **Resolution**: Microseconds (from `esp_timer_get_time()`)
- **Callback context**: ESP timer task (not ISR safe, can call motor functions)

### Critical Timing Scenarios

| Scenario | Timer Behavior |
|----------|----------------|
| `bounds` completes | Timer started for 2 minutes |
| `start` with valid cache | Timer cancelled, test runs |
| Timer expires (idle) | Motor de-energized, bounds marked expired |
| `stop` during test | Timer NOT restarted (bounds stay valid) |
| Config change | Timer cancelled, bounds invalidated |

---

## Safety Considerations

### Motor Heating Prevention

The de-energize timer ensures the motor doesn't remain energized indefinitely:

1. After bounds finding, motor stays energized (holding torque applied)
2. If no test starts within validity window, motor is disabled
3. This prevents motor and driver overheating during extended idle periods

### Power-On State

At boot:
- Motor starts disabled
- Bounds are invalid
- First `start` or `bounds` command will energize and find bounds

### Error Recovery

If bounds finding fails:
- Motor is disabled immediately
- Bounds remain invalid
- Next `start` will retry bounds finding

---

## Troubleshooting

### Bounds Keep Expiring Too Fast

**Cause**: Validity window too short for your workflow.

**Solution**: Increase validity window:
```cpp
BoundsCache::SetValidityMinutes(5); // 5 minutes
```

### Motor De-energizes Unexpectedly

**Cause**: Validity timer expired.

**Solution**: Run `bounds` again or start test before timeout.

### Start Runs Bounds Even Though Recently Found

**Possible Causes**:
1. Config was changed (invalidates bounds)
2. Timer expired
3. Motor was manually disabled

**Solution**: Check `status` command for bounds cache state.

### Motor Stays Energized After Stop

**Expected Behavior**: After `stop`, bounds remain valid but motor stays energized for the remaining validity window to allow quick restart.

**To force motor disable**: Wait for timeout, or change configuration.

