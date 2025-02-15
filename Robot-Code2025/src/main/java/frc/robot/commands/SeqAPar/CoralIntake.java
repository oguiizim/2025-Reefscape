package frc.robot.commands.SeqAPar;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Setpoint;
import frc.robot.commands.OuttakeAngle;
import frc.robot.commands.OuttakeIntake;
import frc.robot.commands.OuttakeRotation;
import frc.robot.subsystems.Outtake;

public class CoralIntake extends ParallelCommandGroup {
  Outtake outtake;

  public CoralIntake(Outtake outtake, double speed) {
    addCommands(
        new OuttakeRotation(outtake, Setpoint.outtakeRotationIntake),
        new OuttakeAngle(outtake, 0.65),
        new OuttakeIntake(outtake, speed));
  }
}
