package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Enums.IntakeState;

public class Intake extends SubsystemBase {

    private final Spark frontMotor = new Spark(0);
    private final Spark backMotor = new Spark(1);

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

  //Stop Motors
    public void stopMotors() {
        frontMotor.stopMotor();
        backMotor.stopMotor();
    }


 
}
