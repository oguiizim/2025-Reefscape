package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Creeper;

public class CreeperIntake extends Command {
   Creeper creeper;
   double speed;

   public CreeperIntake(Creeper creeper, double speed) {
      this.creeper = creeper;
      this.speed = speed;
   }

   @Override
   public void execute() {
      // Minus for intake, and plus for score
      creeper.setSpeed(-speed);
   }

   @Override
   public void end(boolean interrupted) {
      creeper.setSpeed(0);
   }

   @Override
   public boolean isFinished() {
      return creeper.getIR();
   }
}
