package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.DrivetrainConstants;
import frc.robot.RobotContainer;
import frc.robot.enums.SpeedLimitState;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {

    int i = 0;

    @Override
    public void setControl(SwerveRequest request) {
        if (request.getClass() == FieldCentric.class) {
            var fieldCentric = (FieldCentric) request;
            if (fieldCentric.RotationalRate != 0 && i % 50 == 0) {
                System.out.println("Omega is not zero, value: "
                        + fieldCentric.RotationalRate / Constants.Drivetrain.MaxRadiansPerSecond);
                Thread.dumpStack();
            }
        }

        i++;

        super.setControl(request);
    }

    // Singleton
    private static final Drivetrain instance = DrivetrainConstants.DriveTrain;

    public SpeedLimitState speedLimit = SpeedLimitState.Half;

    private final double AroundTargetHeadingThresholdDegrees = 5;

    public static Drivetrain construct(SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants... modules) {
        return new Drivetrain(driveTrainConstants, modules);
    }

    public static Drivetrain get() {
        return instance;
    }

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private Drivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command applyFieldCentricFacingAngle(DoubleSupplier degrees, DoubleSupplier velocityX,
            DoubleSupplier velocityY) {
        final SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle();

        request.HeadingController.setP(8);
        request.HeadingController.setD(0.2);
        request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        return applyRequest(() -> request
                .withVelocityX(velocityX.getAsDouble())
                .withVelocityY(velocityY.getAsDouble())
                .withTargetDirection(Rotation2d.fromDegrees(degrees.getAsDouble())));
    }

    public boolean isAroundTargetHeading() {
        Rotation2d target = Rotation2d.fromDegrees(90);

        return Math
                .abs(this.m_pigeon2.getRotation2d().minus(target).getDegrees()) < AroundTargetHeadingThresholdDegrees;
    }

    public double getCurrentHeading() {
        return this.m_pigeon2.getAngle();
    }

    public double getSpeedLimit() {
        return speedLimit.getSpeedLimit();
    }

    public void setSpeedLimit(SpeedLimitState speedLimitState) {
        speedLimit = speedLimitState;
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
