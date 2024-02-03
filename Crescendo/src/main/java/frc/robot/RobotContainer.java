// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of Nicholas was here
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Intake.IntakeUntilDetection;
import frc.robot.commands.Intake.ManualSetIntake;
import frc.robot.commands.Intake.Outtake;
import frc.robot.commands.autoShooting.AutoShoot;
import frc.robot.commands.index.ManualSetIndex;
import frc.robot.commands.shooter.ManualShoot;
import frc.robot.commands.shooter.SetShooterAngle;
import frc.robot.commands.shooter.SetShooterSpeed;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterSpeed;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.Telemetry;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular
                                                 // velocity

  // The robot's subsystems are defined here...
  private final Climber climber = Climber.get();
  // private final Drivetrain drivetrain2 = new Drivetrain(null, null);
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final Index index = new Index();
  // private final Intake intake = new Intake();
  // private final Limelight limelight = new Limelight();
  // private final ShooterAngle shooterAngle = new ShooterAngle();
  // private final ShooterSpeed shooterSpeed = ShooterSpeed.get();
  // private final Telemetry telemetry = new Telemetry(MaxSpeed);

  // The robot's commands are defined here...

  // autoShooting
  // private final AutoShoot autoShoot = new AutoShoot();

  // Climber

  // Index
  // private final ManualSetIndex manualSetIndex = new ManualSetIndex(index, MaxSpeed);

  // Intake
  // private final IntakeUntilDetection intakeUntilDetection = new IntakeUntilDetection(null, index);
  // private final ManualSetIntake manualSetIntake = new ManualSetIntake(null, index, MaxSpeed);
  // private final Outtake outtake = new Outtake(null, index);

  // Shooter
  // private final ManualShoot manualShoot =
  //     new ManualShoot(MaxSpeed, shooterSpeed, shooterAngle, MaxAngularRate, index);
  // private final SetShooterAngle setShooterAngle = new SetShooterAngle(shooterAngle, null);
  // private final SetShooterSpeed setShooterSpeed = new SetShooterSpeed(null, shooterSpeed);
  // private final Shoot shoot = new Shoot(null, null, shooterSpeed, shooterAngle, index);

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController xbox =
      new CommandXboxController(OperatorConstants.kDriverControllerPort); // My
                                                                          // xbox
  // private final Drivetrain drivetrain = Constants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10%
                                                                                 // deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // xbox.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // DRIVETRAIN
    // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() -> drive.withVelocityX(-xbox.getLeftY() * MaxSpeed) // Drive
    //                                                                                    // forward
    //                                                                                    // with
    //                                                                                    // negative Y
    //                                                                                    // (forward)
    //         .withVelocityY(-xbox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(-xbox.getRightX() * MaxAngularRate) // Drive counterclockwise with
    //                                                                 // negative X (left)
    //     ));



    xbox.back().whileTrue(climber.descendCommand());
    xbox.start().whileTrue(climber.ascendCommand());

    // xbox.rightTrigger().and(xbox.y()).whileTrue(manualSetIntake.manualSetIntake());
    // xbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // xbox.b().whileTrue(drivetrain.applyRequest(
    //     () -> point.withModuleDirection(new Rotation2d(-xbox.getLeftY(), -xbox.getLeftX()))));
    // xbox.button(1).onTrue(autoShoot);
    // // reset the field-centric heading on left bumper press
    // xbox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    // dpad center = manual shoot
    // left joystick = climber adjustment
    // left trigger and y = shooter intake
    // right trigger and y = manual intake (no laser sensor)
    // right joystick = shooter angle
    // a = auto shoot
    // x = stop shooter
    // y = intake
    // b = outtake
    // start = reset climber
    // mode = set climber to lowest chain position
    // back = set climber to highest chain position
    // bumpers = set manual shootstate (speed/angle)
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.print("TODO Auto");
  }


  /*
   * Intake intake game piece until dectection when detection occurs, stop the belt, the intake, and
   * initiate the falcon motors manual intake/belt until stop button stop intake outtake Game piece
   * move intake and belt
   */

  // Belt
  // *manual belt
  // *manual belt stop
  // *move belt for shooting (once shooter is at right speed)
  // shooter intake

  // Shooter Angle
  // adjust to set angles (for set distances)
  // reset to home
  // adjust to shortest angle to pass under stage?
  // manually adjust angle
  // auto adjust angle

  // Shooter
  // intake with shooter
  // adjust to set speed
  // auto adjust to speed?
  // stop shooter

  // LEDs
  // in "reved up" state
  // Errors
  // laser sensor out
  // When note has been detected
  // default colors
  // after game piece has been shot

  // Climber
  // Move up and down
  // lock?

  // Hello Brave Traveller
}
