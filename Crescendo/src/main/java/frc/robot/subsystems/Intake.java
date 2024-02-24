package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IntakeState;

public class Intake extends SubsystemBase {
  private static final Intake instance = new Intake();

  public static Intake get() {
    return instance;
  }

  // Constants

  // Devices
  private final CANSparkMax motor;

  private Intake() {
    motor = new CANSparkMax(11, MotorType.kBrushless);
  }



  public Command intakeCommand(IntakeState state) {

    return Commands.runOnce(() -> {
      System.out.println("Move Intake");

      motor.set(state.getSpeed());
    }, this);
  }

  // public void setIntake()

  // Stop Motors
  public void stop() {
    System.out.println("Stop Intake");
    motor.stopMotor();
  }
}
