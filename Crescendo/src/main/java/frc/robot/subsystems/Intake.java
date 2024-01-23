package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private Constants constants;

    private final Spark frontMotor = new Spark(0);
    private final Spark backMotor = new Spark(1);

  public Intake() {}

  //Intake/Outtake front
  // if intake: direction=1 //if outtake: direction=-1
    public void setFront(int direction) {   
      double speed = constants.intakeSpeed * direction;
      frontMotor.set(speed);
    }

  //Intake/Outtake back
    public void setBack(int direction) {
      double speed = constants.intakeSpeed * direction ; 
      backMotor.set(speed) ;
    }

    public void setBoth(int direction) {
      setFront(direction) ;
      setBack(direction) ;
    } 

  //Stop Motors
    public void stopMotors() {
        frontMotor.stopMotor();
        backMotor.stopMotor();
    }


 
}
