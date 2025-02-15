package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorMove extends Command {
  Elevator elevator;
  PIDController pid;
  double setpoint;

  public ElevatorMove(Elevator elevator, double setpoint) {
    this.elevator = elevator;
    this.setpoint = setpoint;
    pid = new PIDController(0.5, 0.0, 0.0);
    pid.setTolerance(1);

    addRequirements(elevator);
  }

  @Override
  public void execute() {
    double outputR = pid.calculate(elevator.getEncoderR(), -setpoint);
    double outputL = pid.calculate(elevator.getEncoderL(), setpoint);

    outputL = MathUtil.clamp(outputL, -0.4, 0.4);
    outputR = MathUtil.clamp(outputR, -0.4, 0.4);

    elevator.setSpeed(-outputR, outputL);
    // elevator.setSpeed(0.2, 0.2);

    SmartDashboard.putNumber("OutputR", outputR);
    SmartDashboard.putNumber("OutputL", outputL);
  }

  @Override
  public void end(boolean interrupted) {
    // elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
