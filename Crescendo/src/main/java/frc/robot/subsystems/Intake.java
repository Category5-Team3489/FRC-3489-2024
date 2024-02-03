package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IntakeState;

public class Intake extends SubsystemBase {
  // Constants
  

  // Devices
  private final CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);

  // Singleton
  private static final Intake instance = new Intake();
  public static Intake get() {
    return instance;
  }

  // set speed state
  public void setSpeedState(IntakeState speed) {
    motor.set(speed.getSpeed());
  }

  // set speed double
  public void setDoubleSpeed(double speed) {
    motor.set(speed);
  }

  // Stop Motors
  public void stopMotors() {
    motor.stopMotor();
  }
}
