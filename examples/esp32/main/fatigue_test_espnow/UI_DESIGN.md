# UI Design Documentation - Fatigue Test ESP-NOW Controller

## Overview

This document describes the user interface design for the Fatigue Test ESP-NOW remote controller, featuring a 2.9" ThinkInk Tricolor E-Ink display and three-button navigation.

## Hardware Specifications

### Display
- **Model**: Adafruit 2.9" ThinkInk FeatherWing Tricolor E-Ink Display
- **Physical Dimensions**: 296 x 128 pixels (horizontal orientation)
- **Portrait Mode**: 128 x 296 pixels (rotated 90°)
- **Controller**: IL0373
- **Colors**: Black, White, Red (tricolor support)
- **Refresh Rate**: Optimized for e-ink (2+ seconds for full refresh)

### Buttons
Three buttons positioned on the side of the display:
- **Top Button (UP)**: Navigate up, increase values
- **Middle Button (SELECT)**: Confirm, enter menus
- **Bottom Button (DOWN)**: Navigate down, decrease values, stop test

## UI States and Navigation Flow

```
┌─────────────────────────────────────┐
│         MAIN SCREEN                 │
│  [Fatigue Tester]                   │
│  Cycles: 1000                       │
│  Time: 1s                           │
│  Dwell: 1s                          │
│  Method: SG                         │
│                                     │
│  START (UP)                         │
│  SET (SELECT)                       │
│                                     │
│  Cycle: 0/1000                      │
└─────────────────────────────────────┘
         │
         ├─ SELECT → Settings Menu
         ├─ UP → Start Test → Running Screen
         └─ DOWN → Confirm Stop → Stop Screen
```

## Screen Mockups

### 1. Main Screen (Idle)

```
┌──────────────────────────────┐
│     Fatigue                   │
│      Tester                   │
│                               │
│ Cycles: 1000                  │
│ Time: 1s                      │
│ Dwell: 1s                     │
│ Method: SG                    │
│                               │
│ START                         │
│ SET                           │
│                               │
│ Ready                         │
└──────────────────────────────┘
```

**Button Actions:**
- **UP**: Start test
- **SELECT**: Enter settings menu
- **DOWN**: Show stop confirmation

### 2. Main Screen (Running)

```
┌──────────────────────────────┐
│     Fatigue                   │
│      Tester                   │
│                               │
│ Cycles: 1000                  │
│ Time: 1s                      │
│ Dwell: 1s                     │
│ Method: SG                    │
│                               │
│ [RUNNING]                     │
│ PAUSE (SEL)                   │
│                               │
│ Cycle: 542/1000               │
└──────────────────────────────┘
```

**Button Actions:**
- **SELECT**: Pause test
- **DOWN**: Show stop confirmation

### 3. Main Screen (Paused)

```
┌──────────────────────────────┐
│     Fatigue                   │
│      Tester                   │
│                               │
│ Cycles: 1000                  │
│ Time: 1s                      │
│ Dwell: 1s                     │
│ Method: SG                    │
│                               │
│ [PAUSED]                      │
│ RESUME (UP)                   │
│                               │
│ Cycle: 542/1000               │
└──────────────────────────────┘
```

**Button Actions:**
- **UP**: Resume test
- **SELECT**: Resume test (alternative)
- **DOWN**: Show stop confirmation

### 4. Settings Menu

```
┌──────────────────────────────┐
│      Settings                 │
│                               │
│ >Cycles        1000           │
│  Time/Cycle    1s             │
│  Dwell         1s              │
│  Method        SG             │
│  Orientation   Norm           │
│                               │
│ UP/DOWN: Navigate             │
│ SEL: Edit  BACK: Exit         │
└──────────────────────────────┘
```

**Button Actions:**
- **UP**: Navigate to previous item
- **DOWN**: Navigate to next item
- **SELECT**: Enter edit mode for selected item

**Note**: Selected item is highlighted in RED with white text.

### 5. Settings Edit Screen (Example: Cycles)

```
┌──────────────────────────────┐
│      Cycles                   │
│                               │
│                               │
│           1000                │
│                               │
│                               │
│ Range: 1 - 100000             │
│                               │
│ UP: Increase                  │
│ DOWN: Decrease                │
│ SEL: Save                     │
└──────────────────────────────┘
```

**Button Actions:**
- **UP**: Increase value by step (100 for cycles)
- **DOWN**: Decrease value by step
- **SELECT**: Save and return to settings menu

### 6. Settings Edit Screen (Example: Method)

```
┌──────────────────────────────┐
│      Method                   │
│                               │
│                               │
│      StallGuard               │
│                               │
│                               │
│ Range: 0 - 1                  │
│                               │
│ UP: Increase                  │
│ DOWN: Decrease                │
│ SEL: Save                     │
└──────────────────────────────┘
```

**Values:**
- 0 = StallGuard
- 1 = Encoder

### 7. Confirm Stop Screen

```
┌──────────────────────────────┐
│                               │
│                               │
│          Stop                 │
│          Test?                │
│                               │
│                               │
│                               │
│ DOWN = Confirm                │
│ Other = Cancel                │
└──────────────────────────────┘
```

**Button Actions:**
- **DOWN**: Confirm stop (sends stop command)
- **UP/SELECT**: Cancel (return to main screen)

### 8. Error Screen

```
┌──────────────────────────────┐
│                               │
│                               │
│          ERROR                │
│                               │
│        Code: 1                │
│                               │
│                               │
│     Press any                 │
│     button...                 │
└──────────────────────────────┘
```

**Button Actions:**
- **Any button**: Return to main screen

### 9. Complete Screen

```
┌──────────────────────────────┐
│                               │
│                               │
│       Complete!               │
│                               │
│     Cycles: 1000/1000         │
│                               │
│                               │
│     Press any                 │
│     button                    │
└──────────────────────────────┘
```

**Button Actions:**
- **Any button**: Return to main screen

## Settings Menu Items

### 1. Cycles
- **Range**: 1 - 100,000
- **Step**: 100
- **Default**: 1000
- **Description**: Target number of fatigue test cycles

### 2. Time per Cycle
- **Range**: 1 - 3600 seconds
- **Step**: 1 second
- **Default**: 1 second
- **Description**: Time duration for one complete cycle

### 3. Dwell Time
- **Range**: 0 - 60 seconds
- **Step**: 1 second
- **Default**: 1 second
- **Description**: Time to pause at each bound before reversing direction

### 4. Method
- **Range**: 0 (StallGuard) or 1 (Encoder)
- **Step**: 1
- **Default**: StallGuard (0)
- **Description**: Bounds detection method

### 5. Orientation
- **Range**: 0 (Normal) or 1 (Flipped)
- **Step**: 1
- **Default**: Normal (0)
- **Description**: Display orientation (affects rotation)

## Navigation Flow Diagram

```
                    MAIN SCREEN
                         │
        ┌────────────────┼────────────────┐
        │                │                │
    UP (Start)      SELECT          DOWN (Stop)
        │                │                │
        │                │                │
   RUNNING SCREEN  SETTINGS MENU   CONFIRM STOP
        │                │                │
        │                │                │
    SELECT (Pause)   SELECT          DOWN (Confirm)
        │            (Edit)               │
        │                │                │
   PAUSED SCREEN   EDIT SCREEN      MAIN SCREEN
        │                │
    UP (Resume)      SELECT (Save)
        │                │
        │                │
   RUNNING SCREEN   SETTINGS MENU
```

## Color Usage

### Black (EPD_BLACK)
- Primary text
- Normal menu items
- Status information

### White (EPD_WHITE)
- Background
- Text on red selection indicator

### Red (EPD_RED)
- Selected menu items (background)
- Error messages
- Important status indicators

## E-Ink Optimization

1. **Refresh Strategy**:
   - Full refresh only when necessary (screen transitions)
   - Avoid rapid updates (minimum 2 seconds between full refreshes)
   - Footer updates can use partial refresh if supported

2. **Update Frequency**:
   - Main screen: Updates only on state changes
   - Running screen: Footer updates every second (cycle count)
   - Settings menu: Updates only on navigation
   - Edit screen: Updates on every value change

3. **Display Rotation**:
   - Default: Rotation 1 (90° clockwise, portrait)
   - Flipped: Rotation 3 (90° counter-clockwise, portrait flipped)

## Button Configuration

### GPIO Pin Assignment

Configure in `config.hpp`:

```cpp
static constexpr gpio_num_t BTN_UP_GPIO     = GPIO_NUM_4;   // Top button
static constexpr gpio_num_t BTN_SELECT_GPIO = GPIO_NUM_5;   // Middle button
static constexpr gpio_num_t BTN_DOWN_GPIO   = GPIO_NUM_6;   // Bottom button
```

### Button Behavior

- **Pull-up enabled**: Buttons connect to GND when pressed
- **Interrupt**: Negative edge trigger (falling edge)
- **Deep Sleep Wake**: All buttons can wake from deep sleep

## Implementation Notes

### State Machine

The UI uses a state machine with the following states:
- `MAIN`: Main screen (idle)
- `SETTINGS_MENU`: Settings menu list
- `SETTINGS_EDIT_*`: Editing individual settings
- `CONFIRM_STOP`: Stop confirmation dialog
- `ERROR_SCREEN`: Error display
- `RUNNING`: Test running
- `PAUSED`: Test paused
- `COMPLETE`: Test completed

### Thread Safety

- UI updates are handled in a single task (`UI::task`)
- Button events and protocol events are queued
- Display operations are not thread-safe (single task only)

### Settings Persistence

- Settings are stored in NVS (Non-Volatile Storage)
- Settings are saved immediately when edited
- Settings are sent to test unit via ESP-NOW when saved

## Future Enhancements

1. **Nested Menus**: Additional configuration options
2. **Real-time Graphs**: Cycle progress visualization
3. **History**: Previous test results
4. **Advanced Settings**: Motor parameters, acceleration profiles
5. **Multi-unit Support**: Control multiple test units
