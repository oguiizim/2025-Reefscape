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
    pid = new PIDController(0.1, 0.0, 0.0);
    pid.setTolerance(100);

    addRequirements(elevator);
  }

  @Override
  public void execute() {
    double outputR = pid.calculate(elevator.getEncoder(), setpoint);

    outputR = MathUtil.clamp(outputR, -0.6, 0.8);

    elevator.setSpeed(outputR, outputR);

    if (elevator.getVelocity() != 0 && elevator.getEncoder() == 0) {
      elevator.setSpeed(0, 0);
      System.out.println("Elevator Encoder is disconnected");
    }

    SmartDashboard.putNumber("OutputR", outputR);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
