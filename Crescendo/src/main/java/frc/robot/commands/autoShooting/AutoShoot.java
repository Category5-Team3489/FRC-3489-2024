package frc.robot.commands.autoshooting;

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

// ic class AutoShoot extends Command {
// private final AprilLimelight limelight = AprilLimelight.get();
// private final Drivetrain drivetrain = Drivetrain.get();
// private final Index index = Index.get();
// private final ShooterAngle shooterAngle = ShooterAngle.get();
//     private final ShooterSpeed shooterSpeed = ShooterSpeed.get();
  

// private static double MaxStrafeMetersPerSecond = 0.5;
// private static double MaxDistanceMetersPerSecond = 0.75;
// private static double StrafeToleranceDegrees = 1.5;
// private static double DistanceToleranceDegrees = 1.5;
// private static Rotation2d TargetAngle = Rotation2d.fromDegrees(180);
// private static double SpeedLimiter = 0.5;
// private static double MaxOmegaDegreesPerSecond = 90;
// private static double TargetXSetpointDegrees = -4.16;
//     private static double TargetYSetpointDegrees = -19.28;
  

//
  
// 

//     private double yMetersPerSecond = 0;
  

  

  

// ic void initialize() {
// index.isNoteDetected()) {
//     this.cancel();
//     }
  

  

// }
  

// ic void execute() {
// !limelight.isActivePipeline(LimelightPipeline.Shooting)) {
// 
//
//     return;
// 
//     }
  

// !Double.isNaN(targetX)) {
// 
//
// tX, TargetXSetpointDegrees);
//     yMetersPerSecond = MathUtil.clamp(yMetersPerSecond, -MaxStrafeMetersPerSecond, MaxStrafeMetersPerSecond);
// 
// 
//  {
//    yMetersPerSecond = 0;
//     }
  

// !Double.isNaN(targetY)) {
// 
//
// 
// 
//  {
//    xMetersPerSecond = 0;
//     }
  

//
  
// 

  

// ic void end(boolean interrupted) {
//         drivetrain.brakeTranslation();
  

// }
  

// ic boolean isFinished() {
//     return strafeController.atSetpoint();
// }
 
// }

// // rotate robot
// // adjust angle
// // adjust speed


//
// // - press button
// 
// // - detect april tag
// // - calculate offset from center of shooter
// // - calculate distance from shooter
// // - set shooter angle
// // - use drive train to rotate robot (aiming)
// // - validate shooter is within tolerances (maybe)
// // - index note (actually shooting it)
// // - stop shooter motors
// // - reset shooter angle for next intake (maybe)