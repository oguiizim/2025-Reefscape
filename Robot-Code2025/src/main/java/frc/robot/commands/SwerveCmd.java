package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.Strings;
import frc.robot.subsystems.drivetrain.Swerve;
import swervelib.SwerveInputStream;

public class SwerveCmd extends Command {
  private DoubleSupplier vX, vY, heading;
  private BooleanSupplier boostSupplier;
  private Swerve swerve;
  private String fastMode = "OFF";

  public SwerveCmd(Swerve swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading,
      BooleanSupplier boostSupplier) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;
    this.boostSupplier = boostSupplier;

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    if (boostSupplier.getAsBoolean()) {
      fastMode = "ON";
      drive(0.3);
    }
    drive(1.0);
  }

  public void drive(Double scaleTranslation) {
    SwerveInputStream driveOrientedField = SwerveInputStream.of(swerve.getSwerveDrive(), vX, vY)
        .allianceRelativeControl(true)
        .deadband(Drivetrain.deadband)
        .withControllerRotationAxis(heading)
        .scaleRotation(0.7)
        .scaleTranslation(scaleTranslation);

    swerve.driveFieldOriented(driveOrientedField);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
