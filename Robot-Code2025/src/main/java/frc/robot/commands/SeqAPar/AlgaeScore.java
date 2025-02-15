package frc.robot.commands.SeqAPar;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Setpoint;
import frc.robot.subsystems.Creeper;

public class AlgaeScore extends SequentialCommandGroup {
  Creeper creeper;

  public AlgaeScore(Creeper creeper) {
    addCommands(
        Commands.runOnce(() -> creeper.setTarget(Setpoint.creeperLowPosition), creeper),
        new WaitCommand(1),
        Commands.runOnce(() -> creeper.setSpeed(0.7), creeper),
        new WaitCommand(1),
        Commands.runOnce(() -> creeper.setSpeed(0), creeper));
  }
}
