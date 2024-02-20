// package frc.robot.commands.autoShooting;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.commands.shooter.Shoot;
// import frc.robot.enums.LimelightPipeline;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Index;
// import frc.robot.subsystems.AprilLimelight;
// import frc.robot.subsystems.ShooterAngle;
// import frc.robot.subsystems.ShooterSpeed;

// public class AutoShoot extends Command {
//     private final AprilLimelight limelight = AprilLimelight.get();
//     private final Drivetrain drivetrain = Drivetrain.get();
//     private final Index index = Index.get();
//     private final ShooterAngle shooterAngle = ShooterAngle.get();
//     private final ShooterSpeed shooterSpeed = ShooterSpeed.get();

//     private static double ProportionalGain = 0.18;
//     private static double MaxStrafeMetersPerSecond = 0.5;
//     private static double MaxDistanceMetersPerSecond = 0.75;
//     private static double StrafeToleranceDegrees = 1.5;
//     private static double DistanceToleranceDegrees = 1.5;
//     private static Rotation2d TargetAngle = Rotation2d.fromDegrees(180);
//     private static double SpeedLimiter = 0.5;
//     private static double MaxOmegaDegreesPerSecond = 90;
//     private static double TargetXSetpointDegrees = -4.16;
//     private static double TargetYSetpointDegrees = -19.28;

//     private PIDController strafeController = new PIDController(ProportionalGain, 0, 0);
//     private PIDController distanceController = new PIDController(ProportionalGain, 0, 0);

//     private double xMetersPerSecond = 0;
//     private double yMetersPerSecond = 0;

//     public AutoShoot() {

//     }

//     @Override
//     public void initialize() {
//         if (index.isNoteDetected()) {
//             this.cancel();
//         }

//         limelight.setDesiredPipeline(LimelightPipeline.Shooting);

//         strafeController.setTolerance(StrafeToleranceDegrees);
//         distanceController.setTolerance(DistanceToleranceDegrees);
//     }

//     @Override
//     public void execute() {
//         // if (!limelight.isActivePipeline(LimelightPipeline.Shooting)) {
//         //     drivetrain.driveFieldRelative(xMetersPerSecond, yMetersPerSecond, SpeedLimiter, TargetAngle, MaxOmegaDegreesPerSecond);
//         //     return;
//         // }

//         double targetX = limelight.getTargetX();
//         if (!Double.isNaN(targetX)) {
//             yMetersPerSecond = -strafeController.calculate(targetX, TargetXSetpointDegrees);
//             yMetersPerSecond = MathUtil.clamp(yMetersPerSecond, -MaxStrafeMetersPerSecond, MaxStrafeMetersPerSecond);
//         }
//         else {
//             yMetersPerSecond = 0;
//         }

//         double targetY = limelight.getTargetY();
//         if (!Double.isNaN(targetY)) {
//             xMetersPerSecond = distanceController.calculate(targetY, TargetYSetpointDegrees);
//             xMetersPerSecond = MathUtil.clamp(xMetersPerSecond, -MaxDistanceMetersPerSecond, MaxDistanceMetersPerSecond);
//         }
//         else {
//             xMetersPerSecond = 0;
//         }

//         drivetrain.driveFieldRelative(xMetersPerSecond, yMetersPerSecond, SpeedLimiter, TargetAngle, MaxOmegaDegreesPerSecond);

//     }

//     @Override
//     public void end(boolean interrupted) {
//         drivetrain.brakeTranslation();

//         limelight.printTargetData();
//     }

//     @Override
//     public boolean isFinished() {
//         return strafeController.atSetpoint() && distanceController.atSetpoint();
//     }

// }

// // rotate robot
// // adjust angle
// // adjust speed
// // move belt to shoot



// // - press button
// // - check if note is loaded (maybe)
// // - detect april tag
// // - calculate offset from center of shooter
// // - calculate disatance from shooter
// // - start spinning shooter motors
// // - set shooter angle
// // - use drive train to turn shooter
// // - validate shooter is within tolerances
// // - index note
// // - stop shooter motors
// // - reset shooter angle for next intake