package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.Creeper;

public class CreeperAngle extends Command {
   Creeper creeper;
   double setpoint;
   PIDController pid;

   public CreeperAngle(Creeper creeper, double setpoint) {
      this.setpoint = setpoint;
      this.creeper = creeper;
      pid = new PIDController(PIDConstants.creeperP, PIDConstants.creeperI, PIDConstants.creeperD);
   }

   @Override
   public void execute() {
      double output = pid.calculate(creeper.getAngle(), setpoint);
      output = MathUtil.clamp(output, -0.5, .5);
      creeper.setAngleSpeed(output);
   }

   @Override
   public boolean isFinished() {
      return false;
   }

   @Override
   public void end(boolean interrupted) {
      creeper.setAngleSpeed(0);
   }
}
