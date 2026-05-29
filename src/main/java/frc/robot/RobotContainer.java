// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.LEDs;
import frc.robot.commands.DriveCommands.DriveCommands;
import frc.robot.commands.IndexerCommands.FeedShooter;
import frc.robot.commands.IndexerCommands.IdleFeeder;
import frc.robot.commands.IndexerCommands.ReverseFeeder;
import frc.robot.commands.IntakeCommands.ExtendAndRunIntake;
import frc.robot.commands.IntakeCommands.ExtendIntake;
import frc.robot.commands.IntakeCommands.PulseIntake;
import frc.robot.commands.IntakeCommands.RetractIntake;
import frc.robot.commands.IntakeCommands.SlowRetractWithTimeout;
import frc.robot.commands.IntakeCommands.UnjamIntake;
import frc.robot.commands.ShooterCommands.TrackGoalOnly;
import frc.robot.commands.ShooterCommands.TrackTarget;
import frc.robot.commands.ShooterCommands.TrackTargetLive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Feeder.FeederIO;
import frc.robot.subsystems.Feeder.FeederIOSim;
import frc.robot.subsystems.Feeder.FeederIOTalonFX;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeIOTalonFX;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Kicker.KickerIO;
import frc.robot.subsystems.Kicker.KickerIOSim;
import frc.robot.subsystems.Kicker.KickerIOTalonFX;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;
  private final Intake intake;
  private final Feeder feeder;
  private final Kicker kicker;
  private final LEDs leds;
  // (flywheel trigger moved into Shooter subsystem)

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    SmartDashboard.putNumber("SM-RPS", 50);
    SmartDashboard.putNumber("SM-Hood", 0);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera2Name, drive::getRotation));

        shooter = new Shooter(new ShooterIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        feeder = new Feeder(new FeederIOTalonFX());
        kicker = new Kicker(new KickerIOTalonFX());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2, drive::getPose));

        shooter = new Shooter(new ShooterIOSim());
        intake = new Intake(new IntakeIOSim());
        feeder = new Feeder(new FeederIOSim());
        kicker = new Kicker(new KickerIOSim());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});

        shooter = new Shooter(new ShooterIO() {});
        intake = new Intake(new IntakeIO() {});
        feeder = new Feeder(new FeederIO() {});
        kicker = new Kicker(new KickerIO() {});

        break;
    }

    leds = new LEDs();

    // Smartdashboard getters
    SmartDashboard.putNumber("SM-Hood", 0);
    SmartDashboard.getNumber("SM-RPS", 50);

    // Pathplanner auto triggers
    // Tracks hub, shoot when flywheel spun for 3 seconds, then stop after 3 seconds
    // NamedCommands.registerCommand("TakeShot3s", new
    // TrackTarget(shooter).until(shooter.getFlywheelAtSetpointTrigger(2)).andThen(new
    // ParallelDeadlineGroup(new WaitCommand(3), new setIntakeVoltage(intake, 6)).andThen(new
    // setIntakeVoltage(intake, 0))));
    NamedCommands.registerCommand(
        "LockOnTarget5s",
        new TrackTarget(shooter)
            .withTimeout(4)
            .andThen(new TrackGoalOnly(shooter).withTimeout(0.25)));
    NamedCommands.registerCommand(
        "LockOnTarget10s",
        new TrackTarget(shooter)
            .withTimeout(10)
            .andThen(new TrackGoalOnly(shooter).withTimeout(0.25)));
    NamedCommands.registerCommand("ExtendIntake", new ExtendAndRunIntake(intake));
    NamedCommands.registerCommand("RetractIntake", new RetractIntake(intake));
    NamedCommands.registerCommand("PulseIntake", new PulseIntake(intake));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Provide shooter with the robot pose supplier and create the flywheel-at-setpoint
    // trigger inside the Shooter subsystem so it can be initialized safely.
    shooter.setRobotPoseSupplier(drive::getPose);
    shooter.setChassisSpeedsSupplier(drive::getChassisSpeeds);

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Shooter sysID
    autoChooser.addOption(
        "Flywheel Sysid (Quasistatic Forward)",
        shooter.flywheelQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel Sysid (Quasistatic Reverse)",
        shooter.flywheelQuasistatic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption(
        "Flywheel Sysid (Dynamic Forward)",
        shooter.flywheelDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel Sysid (Dynamic Reverse)",
        shooter.flywheelDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // DRIVETRAIN BINDINGS
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> false));

    // Default shooter to keep turret tracking at all times and keep flywheel partially spun
    shooter.setDefaultCommand(new TrackGoalOnly(shooter));

    // Cause the robot to begin shooting once the flywheel reaches speed and turret is at setpoint
    shooter
        .getFlywheelAtSetpointTrigger(10)
        .and(() -> shooter.isTurretAtSetpoint())
        .onTrue((new FeedShooter(feeder, kicker)))
        .onFalse(new IdleFeeder(feeder, kicker));

    // Automatic unjam on jam detection - only handle the unjam, don't manage idle
    feeder
        .getFeederJammedTrigger()
        .onTrue(
            new SequentialCommandGroup(
                new ReverseFeeder(feeder, kicker).withTimeout(Constants.feeder.unjamDuration),
                new FeedShooter(feeder, kicker)));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero,
                () -> false));

    // Reset gyro to 0° when Y button is pressed
    controller
        .y()
        .onTrue(
            Commands.runOnce(() -> drive.setPose(new Pose2d(3.55, 4.025, Rotation2d.kZero)), drive)
                .ignoringDisable(true));

    // Extend intake and unjam everything
    controller
        .x()
        .whileTrue(new UnjamIntake(intake, feeder, kicker))
        .onFalse(
            new ParallelCommandGroup(new IdleFeeder(feeder, kicker), new RetractIntake(intake)));

    // force unjam indexer
    controller
        .b()
        .whileTrue(new ReverseFeeder(feeder, kicker))
        .onFalse(new IdleFeeder(feeder, kicker));

    // Lock onto hub for shot while right trigger is held. Also slows down drivetrain
    controller.rightTrigger(0.5).whileTrue(new TrackTargetLive(shooter));
    controller
        .rightTrigger(0.5)
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX(),
                () -> true));

    // controller.povLeft().whileTrue(new PresetShooter(shooter, () -> 0, () -> 0, () -> 0));
    // controller.povRight().whileTrue(new PresetShooter(shooter, () -> 90, () -> 0, () -> 55));

    // INTAKE BINDINGS
    // Extend intake and run rollers when intake button is held (Does not retract on release)
    controller
        .leftTrigger(0.5)
        .onTrue(new ExtendAndRunIntake(intake))
        .onFalse(new ExtendIntake(intake));
    // Retract intake
    controller.leftBumper().onTrue(new RetractIntake(intake));
    // pulse intake
    controller
        .rightBumper()
        .whileTrue(new SlowRetractWithTimeout(intake))
        .onFalse(new ExtendIntake(intake));

    // EMERGENCY turret reset position to 0
    controller
        .povUp()
        .onTrue(new InstantCommand(() -> shooter.resetTurret(0)).ignoringDisable(true));
    controller
        .povUp()
        .onTrue(new InstantCommand(() -> intake.resetIntakeZero()).ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
