# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an FTC (FIRST Tech Challenge) robot control repository based on the Road Runner quickstart. The codebase uses Road Runner v1.0+ for autonomous motion planning and path following with mecanum drive robots.

**Key Technologies:**
- Road Runner FTC library (v0.1.25) - motion planning and trajectory following
- FTC SDK - robot hardware control framework
- FTC Dashboard (v0.5.1) - real-time telemetry and tuning interface
- Android Gradle build system

## Build Commands

**Build the project:**
```bash
./gradlew build
```

**Install to connected robot controller:**
```bash
./gradlew installDebug
```

**Clean build artifacts:**
```bash
./gradlew clean
```

Note: The project uses Gradle wrapper (`gradlew`), so you don't need Gradle installed separately.

## Code Architecture

### Drive System (`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`)

**MecanumDrive.java** - Main drive class for mecanum drivetrains
- Integrates Road Runner's trajectory following with FTC hardware
- Contains tunable parameters in `MecanumDrive.Params` class (accessible via FTC Dashboard)
- Key parameters: `inPerTick`, `trackWidthTicks`, feedforward gains (`kS`, `kV`, `kA`), velocity/acceleration constraints
- Provides `actionBuilder()` for creating autonomous trajectories
- Supports both drive motor encoders and external localizers (dead wheels, OTOS, Pinpoint)

**TankDrive.java** - Alternative drive class for tank/differential drivetrains

### Localization Options

The codebase supports multiple localization methods (configured in drive class constructor):
- **TwoDeadWheelLocalizer** - Two odometry wheels + IMU
- **ThreeDeadWheelLocalizer** - Three odometry wheels (no IMU dependency)
- **OTOSLocalizer** - SparkFun OTOS optical tracking sensor
- **PinpointLocalizer** - goBILDA Pinpoint odometry computer

### Subsystems

**Intake.java** - Intake mechanism control
- Exposes Road Runner `Action` methods: `intakeIn(power)`, `intakeOut(power)`, `stop()`
- Actions are non-blocking and integrate with trajectory sequences

### Autonomous OpModes

Autonomous programs extend `LinearOpMode` and use Road Runner's `Action` system:

**Key patterns:**
```java
// Initialize with starting pose
MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);

// Build trajectory with action builder
Action path = drive.actionBuilder(START_POSE)
    .strafeTo(targetVector)           // strafe to position
    .lineToX(xCoord)                  // move to X coordinate
    .turn(angleRadians)               // turn in place
    .afterTime(0.0, intake.intakeIn()) // parallel action
    .waitSeconds(2.0)                 // blocking wait
    .build();

// Execute trajectory
Actions.runBlocking(path);
```

**Threading for parallel subsystems:**
- See `PickAndShootWithThreads.java` for example of running subsystems (like shooter) in parallel with motion
- Start thread before trajectory, ensure proper cleanup on `isStopRequested()`

### Tuning OpModes

Located in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tuning/`:

**TuningOpModes.java** - Registers tuning OpModes for FTC Dashboard
- Configure `DRIVE_CLASS` to match your drive class (MecanumDrive or TankDrive)
- OpModes are auto-registered when deployed

**Tuning sequence (via FTC Dashboard at http://192.168.43.1:8080):**
1. Motor direction debugging
2. Dead wheel direction (if using external encoders)
3. Forward/lateral/angular ramp tests for feedforward tuning
4. Localization testing
5. Spline/trajectory testing

Reference: https://rr.brott.dev/docs/v1-0/tuning/

## Coordinate System and Field Setup

The autonomous strategy documents (`AutonomousStrategy.md`) show field coordinate conventions:
- Origin typically at starting tile (e.g., C1)
- Positions use `Pose2d(x, y, heading)` where x/y are in inches, heading in radians
- Helper functions like `forward()` compute waypoints relative to current pose

**Typical autonomous structure:**
1. Set initial pose estimate: `MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE)`
2. Pre-build all trajectory actions using `actionBuilder()`
3. Execute in sequence with `Actions.runBlocking()`
4. Interleave subsystem actions using `.afterTime()` or direct power control between trajectories

## Development Patterns

**Subsystem Action pattern:**
All subsystems should expose Road Runner `Action` methods that:
- Initialize hardware state in first `run()` call
- Return `false` for one-shot actions (e.g., setting motor power)
- Return `true` for continuous actions until completion condition met
- Report telemetry via `TelemetryPacket` parameter

**Using FTC Dashboard:**
- Mark classes with `@Config` to expose static fields for live tuning
- Access dashboard at http://192.168.43.1:8080 when connected to robot controller WiFi
- Use `MultipleTelemetry` to send data to both driver station and dashboard

**Pose estimation:**
- Always initialize drive with known starting pose in autonomous
- For TeleOp, can load saved pose from file written at end of autonomous
- Update localization continuously via `drive.updatePoseEstimate()` in TeleOp loops

## Module Structure

- `FtcRobotController/` - Base SDK module (avoid editing)
- `TeamCode/` - All team-specific code goes here
- `libs/` - Contains FTC debug keystore
- `build.common.gradle` - Shared build configuration (avoid editing)
- `TeamCode/build.gradle` - Team-specific dependencies and build customization

## Important Notes

- Heading angles use radians (use `Math.toRadians()` to convert from degrees)
- FTC Dashboard config variables update in real-time - useful for tuning PID and motion parameters
- When building trajectories, use `strafeToLinearHeading()` to simultaneously change position and heading
- Use `.setReversed(true)` in action builders to drive backwards (useful for intake mechanisms)
- Road Runner trajectories are asynchronous by default - use `Actions.runBlocking()` or proper action composition
