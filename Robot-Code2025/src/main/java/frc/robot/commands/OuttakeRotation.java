package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake;

public class OuttakeRotation extends Command {
   Outtake outtake;
   PIDController pid;
   double set;

   public OuttakeRotation(Outtake outtake, double set) {
      this.outtake = outtake;
      this.set = set;
      pid = new PIDController(0.35, 0, 0);
   }

   @Override
   public void execute() {
      double output = pid.calculate(outtake.getRotationEncoder(), set);
      output = MathUtil.clamp(output, -0.1, .1);
      outtake.setRotationSpeed(output);
   }

   @Override
   public boolean isFinished() {
      return pid.atSetpoint();
   }

   @Override
   public void end(boolean interrupted) {
      outtake.setAngleSpeed(0);
   }
}