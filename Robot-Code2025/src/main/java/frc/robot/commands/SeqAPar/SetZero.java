package frc.robot.commands.SeqAPar;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Setpoint;
import frc.robot.subsystems.Creeper;
import frc.robot.subsystems.Outtake;

public class SetZero extends ParallelCommandGroup {
  Creeper creeper;
  Outtake outtake;

  public SetZero(Creeper creeper, Outtake outtake) {
    addCommands(Commands.runOnce(() -> creeper.setTarget(Setpoint.creeperHighPosition), creeper));
  }
}
