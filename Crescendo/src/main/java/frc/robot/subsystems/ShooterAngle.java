package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.enums.ShooterAngleState;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

// Power robot on with shooter angle in home position
public class ShooterAngle extends SubsystemBase {
    private static ShooterAngle instance = new ShooterAngle();

    public static ShooterAngle get() {
        return instance;
    }

    private static final int LeftMotorCanId = 10;
    private static final int RightMotorCanId = 9;

    private static final double CorrectionDegreesPerSecond = 10;    //50

    // Gear ratio
    private static final double MotorRotationsPerRevolution = (58.0/12.0) * (58.0/14.0) * (64.0/12.0);   //Correct = (58.0 / 10.58) * (58.0 / 18.0) * (64.0 / 12.0)
    // private static final double MotorRotationsPerRevolutionOld = (58.0 / 12.0) *
    // (58.0 / 14.0) * (64.0 / 12.0);
    // new / old = 0.88216761184

    // Convert revolutions to degrees (Used in PID)
    private static final double MotorRotationsPerDegree = MotorRotationsPerRevolution / 360.0;
    private static final double DegreesPerMotorRotation = 1.0 / MotorRotationsPerDegree;

    // Shooter is at its target angle if the error
    // is within plus or minus this value
    private static final double AllowedErrorDegrees = 2.0;

    // Left and right are from the perspective of the robot
    private final CANSparkMax leftMotor = new CANSparkMax(LeftMotorCanId, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(RightMotorCanId, MotorType.kBrushless);

    private final SparkPIDController pidControllerLeft = leftMotor.getPIDController();
    private final SparkPIDController pidControllerRight = rightMotor.getPIDController();

    private final RelativeEncoder encoder = leftMotor.getEncoder();

    private double targetAngleDegrees = ShooterAngleState.Start.getAngle();

    private ShooterAngle() {
        Shuffleboard.getTab("Main")
                .addDouble("Target Shooter Angle", () -> targetAngleDegrees)
                .withSize(1, 1)
                .withPosition(7, 1);
    }

    @Override
    public void periodic() {
        setAngle(targetAngleDegrees);
    }

    private void setAngle(double angleDegrees) {
        setTargetAngle(angleDegrees);
        double targetRotations = angleDegrees * MotorRotationsPerDegree;
        pidControllerLeft.setReference(targetRotations, ControlType.kPosition, 0);
        pidControllerRight.setReference(targetRotations, ControlType.kPosition, 0);
    }

    public boolean isAtTargetAngle() {
        double rotations = encoder.getPosition();
        double angleDegrees = rotations * DegreesPerMotorRotation;

        double errorDegrees = targetAngleDegrees - angleDegrees;
        errorDegrees = Math.abs(errorDegrees);

        return errorDegrees < AllowedErrorDegrees;
    }

    public Command adjustManualAngle(double adjustPercent) {
        return Commands.run(() -> {
            double deltaDegrees = adjustPercent * CorrectionDegreesPerSecond * Robot.kDefaultPeriod;
            setTargetAngle(targetAngleDegrees + deltaDegrees);
        }, this);
    }

    private void setTargetAngle(double angleDegrees) {
        targetAngleDegrees = MathUtil.clamp(angleDegrees,
                ShooterAngleState.Start.getAngle(), ShooterAngleState.Max.getAngle());
        System.out.println("Target ANGLE ==============" + targetAngleDegrees);
    }

    public Command updateCommand(double angleDegrees) {
        System.out.println("UPDATE");
        return updateCommand(() -> angleDegrees);
    }

    public Command updateCommand(DoubleSupplier angleDegreesSupplier) {
        return Commands.run(() -> {
            setTargetAngle(angleDegreesSupplier.getAsDouble());
        }, this);
    }
}