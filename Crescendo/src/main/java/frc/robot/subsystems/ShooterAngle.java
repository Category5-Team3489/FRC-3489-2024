package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

//Power robot on with shooter angle in home position
public class ShooterAngle extends SubsystemBase {

    // Constants
    private static final double MotorRotationsPerRevolution = (100.0 / 1.0) * (2.0 / 1.0); // gear ratio (revolution)
    // convert revolutions to degrees
    private static final double MotorRotationsPerDegree = MotorRotationsPerRevolution / 360.0;
    private static final double DegreesPerMotorRotation = 1.0 / MotorRotationsPerDegree; // used in pid

    // Shooter is at its target angle if the error
    // is within plus or minus this value
    private static final double AllowedErrorDegrees = 2.0;

    private final CANSparkMax angleMotorLeft = new CANSparkMax(0, MotorType.kBrushless);
    private final CANSparkMax angleMotorRight = new CANSparkMax(1, MotorType.kBrushless);

    private final SparkPIDController pidController;
    private final RelativeEncoder encoder;

    private double targetAngleDegrees = Double.NaN;

    public ShooterAngle() {
        angleMotorLeft.setInverted(true);
        angleMotorLeft.follow(angleMotorRight);

        pidController = angleMotorRight.getPIDController();
        encoder = angleMotorRight.getEncoder();
    }

    public void setAngle(double angleDegrees) {
        targetAngleDegrees = angleDegrees;

        double targetRotations = angleDegrees * MotorRotationsPerDegree;
        pidController.setReference(targetRotations, ControlType.kPosition, 0);
    }

    public boolean isAtTarget() {
        if (Double.isNaN(targetAngleDegrees)) {
            return false;
        }

        double rotations = encoder.getPosition();
        double angleDegrees = rotations * DegreesPerMotorRotation;

        double errorDegrees = targetAngleDegrees - angleDegrees;
        errorDegrees = Math.abs(errorDegrees);

        return errorDegrees < AllowedErrorDegrees;
    }

    // manual shooting
    // adjust angle
    // adjust speed
    // move belt index

}