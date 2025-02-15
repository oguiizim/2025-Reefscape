package frc.robot.commands.SeqAPar;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Setpoint;
import frc.robot.commands.OuttakeAngle;
import frc.robot.commands.OuttakeIntake;
import frc.robot.commands.OuttakeRotation;
import frc.robot.subsystems.Outtake;

public class CoralScore extends SequentialCommandGroup {
  Outtake outtake;

  public CoralScore(Outtake outtake, double speed) {
    addCommands(
        new OuttakeRotation(outtake, Setpoint.outtakeRotationScore),
        new OuttakeAngle(outtake, 0.55),
        new WaitUntilCommand(1),
        new OuttakeIntake(outtake, speed));
  }
}
