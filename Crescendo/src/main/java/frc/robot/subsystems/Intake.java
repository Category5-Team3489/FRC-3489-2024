package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IntakeState;

public class Intake extends SubsystemBase {
  private static final Intake instance = new Intake();

  public IntakeState intakeState = IntakeState.Off;

  public static Intake get() {
    return instance;
  }


  // Constants

  // Devices
  private final CANSparkFlex motor;

  public boolean hasIntakeBeenSet = false;

  private Intake() {
    motor = new CANSparkFlex(11, MotorType.kBrushless);

    Shuffleboard.getTab("Main")
                    .addString("intake state", () -> intakeState.toString())
                    .withSize(1, 1)
                    .withPosition(2, 2);

    Shuffleboard.getTab("Main")
                    .addDouble("intake speed", () -> motor.getAppliedOutput())
                    .withSize(1, 1)
                    .withPosition(3, 2);
  }



  public Command intakeCommand(IntakeState state) {

    return Commands.runOnce(() -> {
      System.out.println("=====TESTING" + state.getSpeed());
      intakeState = state;
      //TODO TEST
      hasIntakeBeenSet = true;
      motor.set(state.getSpeed());
    }, this);
  }

  // public void setIntake()

  // Stop Motors
  public void stop() {
    System.out.println("Stop Intake");
    motor.stopMotor();
    //TODO TEST
    hasIntakeBeenSet = false;
  }
}
