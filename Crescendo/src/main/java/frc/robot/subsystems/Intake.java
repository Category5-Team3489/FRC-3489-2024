package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Enums.IntakeState;

public class Intake extends SubsystemBase {

    private final CANSparkMax frontMotor = new CANSparkMax(0, MotorType.kBrushless);
    private final CANSparkMax backMotor = new CANSparkMax(1, MotorType.kBrushless);

  public Intake() {}

  //Intake/Outtake front
    public void setFront(IntakeState speed) {
      frontMotor.set(speed.getSpeed());
    }

  //Intake/Outtake back
    public void setBack(IntakeState speed) {
      backMotor.set(speed.getSpeed());
    }

    //set speed both
    public void setBoth(IntakeState speed) {
      setFront(speed) ;
      setBack(speed) ;
    } 

    public void setDoubleBoth(double speed) {
      frontMotor.set(speed);
      backMotor.set(speed);
    }

  //Stop Motors
    public void stopMotors() {
        frontMotor.stopMotor();
        backMotor.stopMotor();
    }


 
}
