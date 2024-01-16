// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final DigitalInput buttonA = new DigitalInput(0);
  private final DigitalInput buttonB = new DigitalInput(1);
  private final TalonFX motorA = new TalonFX(1);
  private final TalonFX motorB = new TalonFX(5);
  private double speed = 0;
  private boolean lastButtonA = false;
  private boolean lastButtonB = false;
  private double buttonAPressSeconds = 0;
  private double buttonBPressSeconds = 0;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  private long i = 0;

  // 50hz
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (!lastButtonA && buttonA.get()) {
      speed -= 0.01;
    }
    lastButtonA = buttonA.get();
    if (buttonA.get()) {
      buttonAPressSeconds += 0.02;
    } else {
      buttonAPressSeconds = 0;
    }
    if (buttonAPressSeconds > 0.5) {
      speed -= 0.01;
    }

    if (!lastButtonB && buttonB.get()) {
      speed += 0.01;
    }
    lastButtonB = buttonB.get();
    if (buttonB.get()) {
      buttonBPressSeconds += 0.02;
    } else {
      buttonBPressSeconds = 0;
    }
    if (buttonBPressSeconds > 0.5) {
      speed += 0.01;
    }

    if (speed < 0) {
      speed = 0;
    } else if (speed > 1) {
      speed = 1;
    }

    i++;
    /*
     * if (i % 5 == 0) {
     * int percentSpeed = (int) (speed * 100);
     * System.out.println("Speed is " + percentSpeed + "%");
     * }
     */
    if (i % 10 == 0) {
      System.out.println("The velocity is " + motorA.getRotorVelocity());
    }

    if (buttonA.get() && buttonB.get()) {
      speed = 0;
    }
    motorA.set(speed);
    motorB.set(speed);

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
