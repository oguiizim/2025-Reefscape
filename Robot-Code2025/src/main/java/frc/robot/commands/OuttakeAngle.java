package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake;

public class OuttakeAngle extends Command {
   Outtake outtake;
   PIDController pid;
   double set;

   public OuttakeAngle(Outtake outtake, double set) {
      this.outtake = outtake;
      this.set = set;
      pid = new PIDController(1.7, 0, 0);
   }

   @Override
   public void execute() {
      double output = pid.calculate(outtake.getAngleEncoder(), set);
      output = MathUtil.clamp(output, -0.8, .8);
      outtake.setAngleSpeed(output);
   }

   @Override
   public boolean isFinished() {
      return false;
   }

   @Override
   public void end(boolean interrupted) {
      outtake.setAngleSpeed(0);
   }
}
