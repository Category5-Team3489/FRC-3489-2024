package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Enums.ShooterState;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Shooter extends SubsystemBase {

    private final TalonFX motor = new TalonFX(0);

    private final double allowedError = 30; //the amount of error allowed 

    public Shooter() {
        // TalonFXConfiguration configs = new TalonFXConfiguration();
        // configs.Slot0
        // motor.co
    }

    // Rev up motors
    public void setShooterSpeed(double shooterSpeed) {
        motor.set(shooterSpeed);

    }

    public void setShooterState(ShooterState shooterState) {
        motor.set(-shooterState.getSpeed());
        // VelocityVoltage a = new VelocityVoltage()
        // VelocityDutyCycle e = new VelocityDutyCycle()
        // https://github.com/Category5-Team3489/FRC-3489-2022/blob/main/FRC-3489-2022-AutoTesting/src/main/java/frc/robot/handlers/ShooterHandler.java
        // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java
    }

    public boolean isAtTarget() {
        double error = motor.getClosedLoopError().getValueAsDouble();
        if (error <= allowedError) {
            return true;
        }
        return false;
    }

}