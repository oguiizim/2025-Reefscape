package frc.robot.commands.SeqAPar;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.Setpoint;
import frc.robot.commands.CreeperAngle;
import frc.robot.commands.CreeperIntake;
import frc.robot.subsystems.Creeper;

public class AlgaeIntake extends ParallelCommandGroup {
  Creeper creeper;

  public AlgaeIntake(Creeper creeper) {

    addCommands(
        Commands.runOnce(() -> creeper.setTarget(Setpoint.creeperIntakePosition), creeper),
        new CreeperIntake(creeper, 0.5));
  }
}
