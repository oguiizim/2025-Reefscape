package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.Setpoint;
import frc.robot.subsystems.Creeper;

public class CreeperScore extends Command {
  Creeper creeper;
  double setpoint;
  PIDController pid;

  public CreeperScore(Creeper creeper, double setpoint) {
    this.creeper = creeper;
    this.setpoint = setpoint;
    pid = new PIDController(PIDConstants.creeperP, PIDConstants.creeperI, PIDConstants.creeperD);

  }

  @Override
  public void execute() {
    double output = MathUtil.clamp(pid.calculate(creeper.getAngle(), Setpoint.creeperLowPosition), -0.4, 0.4);
    creeper.setAngleSpeed(output);

    if (pid.atSetpoint()) {
      creeper.setSpeed(0.4);
    }

    // if (creeper.atSetpoint()) {
    // creeper.setSpeed(0.4);
    // }
  }

  @Override
  public void end(boolean interrupted) {
    creeper.setSpeed(0);
    this.cancel();
  }

  @Override
  public boolean isFinished() {
    return !creeper.getIR();
  }
}
