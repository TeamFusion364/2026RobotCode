# Team Fusion 364 — 2026 Robot Code

This is the code that runs our robot for the 2026 FRC season, **REBUILT**. It's a swerve-drive robot with a turreted shooter that automatically tracks the goal, an extending intake, and a full indexer/feeder path to move game pieces from the floor up to the flywheel.

It's built on WPILib's command-based framework and fully integrated with Team 6328's Advantagekit, so everything is logged and replayable.

## The short version

- **Drive** — CTRE swerve (TalonFX drive + steer, CANcoders, a Pigeon 2 gyro). Field-relative teleop, and it feeds the shooter its pose so the turret always knows where the goal is.
- **Shooter** — a turret that rotates to aim, a hood that adjusts angle, and a flywheel. Distance-to-goal gets fed through interpolated lookup maps to pick the right hood angle and flywheel speed, so the robot can shoot from basically anywhere on the field.
- **Intake** — an extending "stroker" intake that deploys out, runs rollers to grab pieces, and retracts. Has jam detection and pulse/unjam behavior.
- **Feeder + Kicker** — the indexer path that moves pieces up to the shooter. Watches motor current to detect jams and automatically reverses to clear them.
- **Vision** — Limelight on the real robot (PhotonVision in sim) feeding pose estimates back into the drivetrain's pose estimator.
- **LEDs** — CANdle status lights.

## How the auto-shoot works

The idea is that the driver mostly just drives, and the robot handles aiming and firing on its own.

By default the turret is always tracking the goal and keeping the flywheel partly spun up. When you hold the right trigger, it locks on for a real shot (and slows the drivetrain down so you can be precise). Once the flywheel is at speed **and** the turret is actually pointed at the target, the feeder automatically kicks in and starts feeding pieces into the flywheel. Let go and it idles back down.

If the feeder jams, it notices (via current draw), reverses to spit the jam back out, and then resumes feeding — no driver input needed.

## Controls (Xbox controller)

| Input | Action |
|---|---|
| Left stick / Right stick | Field-relative drive + rotate |
| Right trigger | Lock onto goal for a shot (slows drive, auto-fires when ready) |
| Left trigger | Extend intake + run rollers |
| Left bumper | Retract intake |
| Right bumper | Slow-retract / pulse intake |
| A (hold) | Lock heading to 0° |
| B (hold) | Force-reverse the indexer |
| X (hold) | Extend + unjam everything |
| Y | Reset gyro heading to 0° |
| D-pad Up | Emergency zero the turret + intake |

## Autonomous

Autos are built in [PathPlanner](https://pathplanner.dev/) and live in `src/main/deploy/pathplanner`. Paths get stitched together with named commands like `LockOnTarget5s`, `ExtendIntake`, and `PulseIntake`, so an auto is mostly "drive here, pick up, lock on and shoot, drive there." Pick the routine from the dashboard chooser before the match.

## Code layout

```
src/main/java/frc/robot/
├── RobotContainer.java     # wires up subsystems, autos, and button bindings
├── Constants.java          # ports, tuning values, shooter lookup maps
├── subsystems/
│   ├── drive/              # swerve modules, gyro, odometry
│   ├── Shooter/            # turret + hood + flywheel
│   ├── Intake/             # extending intake
│   ├── Feeder/ + Kicker/   # the indexer path
│   ├── vision/             # Limelight / PhotonVision
│   └── LEDs/
└── commands/               # the actual behaviors, grouped by subsystem
```

Every subsystem follows AdvantageKit's IO pattern: there's an `IO` interface, a real `TalonFX` implementation, and a `Sim` implementation. `Constants.currentMode` switches between **REAL**, **SIM**, and **REPLAY**, which is how the same code runs on the robot, in simulation, and back through recorded logs.

## Building and deploying

Standard GradleRIO project — you'll want [WPILib 2026](https://docs.wpilib.org/en/stable/) installed.

```bash
./gradlew build          # compile
./gradlew simulateJava   # run in simulation
./gradlew deploy         # push to the roboRIO
```

Formatting is enforced with Spotless (`./gradlew spotlessApply` to fix).

## A note on tuning

The shooter's distance→angle/speed maps in `Constants.ShooterMaps` need to be calibrated against the real robot — the values there are starting points, not gospel. If shots are consistently off, that's the first place to look.
