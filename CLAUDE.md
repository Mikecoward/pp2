# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an FTC (FIRST Tech Challenge) Robot Controller project for the 2025-2026 DECODE competition season. The codebase uses the official FTC SDK v11.0 with custom team code for a mecanum drive robot using Pedro Pathing for autonomous navigation and Limelight3A for vision processing.

## Build and Development Commands

### Building the project
```bash
./gradlew build
```

### Build specific modules
```bash
./gradlew TeamCode:build
./gradlew FtcRobotController:build
```

### Clean build
```bash
./gradlew clean build
```

### Fast-load builds (using fast-load plugin)
```bash
./gradlew TeamCode:assembleFastLoad
```

### Assemble APK
```bash
./gradlew assembleDebug
./gradlew assembleRelease
```

## Architecture

### Module Structure
- **FtcRobotController**: Base FTC SDK module (do not modify)
- **TeamCode**: Custom team robot code (all development happens here)

### Team Code Organization

The team code follows a base class pattern with alliance-specific subclasses:

#### Base Classes
- **BaseCatBotTeleop**: Abstract base for teleoperated control
  - Handles mecanum drive with field-centric and robot-centric modes
  - Integrates Pedro Pathing follower for automated movements
  - Limelight3A vision integration for AprilTag detection and MegaTag localization
  - Motor control (slides, intake, catapult) with position/velocity control
  - Servo control (claw, wrist, arm, bucket)
  - ActionScheduler for timed actions
  - Alliance-aware coordinate mirroring (Blue ↔ Red)

- **BaseCatBotAuto**: Abstract base for autonomous programs
  - Pedro Pathing path following
  - Limelight3A vision for alignment and localization
  - Same motor/servo control as teleop
  - Alliance-aware pose mirroring
  - PoseStorage for transferring position between auto and teleop

#### Alliance Subclasses
Each base is subclassed for Red and Blue alliances:
- `CatBotTeleopRed` / `CatBotTeleopBlue`
- `CatBotAutoRed` / `CatBotAutoBlue`

The subclasses only override `getAlliance()` method. All coordinate transformations and field mirroring logic is handled in the base classes.

#### Field Mirroring
The field is 144" × 144". Blue→Red mirroring:
- `x' = 144 - x`
- `y' = y`
- `heading' = π - heading`

This allows defining all paths/poses in Blue coordinates and automatically mirroring them for Red alliance.

#### Supporting Classes
- **ActionScheduler**: Priority queue-based time scheduler for delayed actions
  - `inMs(nowSec, delayMs, action)`: Schedule action relative to current time
  - `atSec(dueSec, action)`: Schedule action at absolute time
  - Call `update(nowSec)` each loop to execute due actions
- **PoseStorage**: Static storage for current robot pose (shared between auto and teleop)
- **SRSHub**: Custom I2C device driver for SRS Hub hardware (address 0x57)
  - Supports analog/digital inputs, encoders, and I2C bus expansion
- **Constants**: Pedro Pathing configuration (in `pedroPathing/` subdirectory)
- **Tuning**: Pedro Pathing tuning OpMode with telemetry
- **PathFactory**: Path generation utilities

### Pedro Pathing Integration

The robot uses [Pedro Pathing](https://pedropathing.com) for autonomous path following:

- **Follower**: Main path following class, configured via `Constants.createFollower(hardwareMap)`
- **Localization**: goBILDA Pinpoint odometry computer
- **Configuration**: All tuning constants in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java`
  - PIDF coefficients for translational and heading control
  - Motor names and directions
  - Odometry pod positions
  - Path constraints (velocity, acceleration)

### Hardware Configuration

**Motors** (all DcMotorEx):
- Drive: `lf`, `rf`, `lb`, `rb` (mecanum wheels)
- Slides: `ls` (left slide), `rs` (right slide)
- Intake: `im` (intake motor)
- Catapult: `cp` (catapult motor)

**Servos**:
- `claw`: Specimen claw
- `wrist`: Wrist joint
- `arm`: Arm position
- `bucket`: Sample bucket

**Sensors**:
- `odo`: goBILDA Pinpoint odometry (I2C)
- `limelight`: Limelight3A vision sensor

### Limelight3A Vision

The Limelight3A is used for:
- AprilTag detection and pose estimation
- MegaTag localization (using multiple tags)
- Automatic pose correction during autonomous
- Target alignment in teleop (automated drive to scoring positions)

Vision pipeline IDs and configurations are defined in the base classes.

### Dependencies

External dependencies added to TeamCode:
- **fast-load-plugin** (0.1.4-beta1): Faster deployment to Robot Controller
- Repository: `https://www.matthewo.tech/maven/`

Pedro Pathing and bylazar libraries (configurables, telemetry, field) are included as part of the team code imports.

## OpMode Naming Convention

OpModes use annotations to appear in the Driver Station:
- `@TeleOp(name = "Display Name", group = "Group")`
- `@Autonomous(name = "Display Name", group = "Group")`

Active OpModes:
- Teleop: "Cat Bot Teleop RED" / "Cat Bot Teleop BLUE"
- Autonomous: "CatBot Auto RED" / "CatBot Auto BLUE"
- Testing: "Catapult Test", "HubTest" (SRSHub testing), "ServoTest", "Tuning" (Pedro Pathing)

## Development Notes

### Android Studio Version
Requires Android Studio Ladybug (2024.2) or later.

### SDK Version
FTC SDK 11.0 (released 20250827-105138) for DECODE season.

### Gradle
Uses Gradle wrapper (`./gradlew`). Build configuration split across:
- `build.gradle`: Top-level project config
- `build.common.gradle`: Common Android build settings (do not edit)
- `build.dependencies.gradle`: Shared dependencies
- `TeamCode/build.gradle`: Team-specific customizations

### Git Workflow
- Main branch: `main`
- FTC SDK is maintained in the base repository and should not be modified
- All team-specific changes go in the `TeamCode` module

### Code Style
- Uses `@Configurable` annotation from bylazar.configurables for runtime tuning
- Telemetry uses PanelsTelemetry for structured output
- Game timer accessed via `getRuntime()` for scheduling actions
