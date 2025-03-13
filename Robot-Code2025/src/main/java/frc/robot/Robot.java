// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Setpoint;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.swerve.resetGyro();
    m_robotContainer.outtake.setTarget(Setpoint.outtakeRotationScore);
    m_robotContainer.outtake.setAngleTarget(Setpoint.outtakeAngleOff);
    m_robotContainer.creeper.setTarget(Setpoint.creeperLowPosition);

    m_robotContainer.swerve.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_robotContainer.outtake.setTarget(Setpoint.outtakeRotationScore);
    m_robotContainer.outtake.setAngleTarget(Setpoint.outtakeAngleOff);
    m_robotContainer.creeper.setTarget(Setpoint.creeperHighPosition);
    m_robotContainer.outtake.setWheelSpeed(0);
    m_robotContainer.creeper.setSpeed(0);

    m_robotContainer.setMotorBrake(true);
    m_robotContainer.setHeadingCorrection(true);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}