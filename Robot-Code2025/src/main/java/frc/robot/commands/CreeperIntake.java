package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.Setpoint;
import frc.robot.subsystems.Creeper;

public class CreeperIntake extends Command {
   Creeper creeper;
   double speed, setpoint;
   PIDController pid, pid2;

   public CreeperIntake(Creeper creeper, double setpoint, double speed) {
      this.creeper = creeper;
      this.speed = speed;
      this.setpoint = setpoint;
      pid = new PIDController(PIDConstants.creeperP, PIDConstants.creeperI, PIDConstants.creeperD);
   }

   @Override
   public void execute() {
      double output = MathUtil.clamp(pid.calculate(creeper.getAngle(), setpoint), -0.5, 0.5);
      creeper.setAngleSpeed(output);
      creeper.setSpeed(speed);

   }

   @Override
   public void end(boolean interrupted) {
      creeper.setSpeed(0);
      creeper.stopAngle();
   }

   @Override
   public boolean isFinished() {
      return creeper.getIR();
   }
}
