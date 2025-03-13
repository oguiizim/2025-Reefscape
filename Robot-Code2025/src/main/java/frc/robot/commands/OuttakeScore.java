package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake;

public class OuttakeScore extends Command {
   Outtake outtake;
   double speed;

   public OuttakeScore(Outtake outtake, double speed) {
      this.outtake = outtake;
      this.speed = speed;
   }

   @Override
   public void execute() {
      // Minus for intake, and plus for score
      outtake.setWheelSpeed(speed);
   }

   @Override
   public void end(boolean interrupted) {
      outtake.setWheelSpeed(0);
   }

   @Override
   public boolean isFinished() {
      return !outtake.getIR();
   }
}
