// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.Swerve;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.core.io.ContentReference;

import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class Drive extends Command {

  private final Swerve swerve;
  private final DoubleSupplier vX, vY;
  private final DoubleSupplier headingHorizontal, headingVertical;
  private final XboxController control = new XboxController(0);
  private boolean initRotation = false;

  /**
   * Used to drive a swerve robot in full field-centric mode. vX and vY supply
   * translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. headingHorzontal and
   * headingVertical are the Cartesian
   * coordinates from which the robot's angle will be derived— they will be
   * converted to a polar angle, which the robot
   * will rotate to.
   *
   * @param swerve            The swerve drivebase subsystem.
   * @param vX                DoubleSupplier that supplies the x-translation
   *                          joystick input. Should be in the range -1
   *                          to 1 with deadband already accounted for. Positive X
   *                          is away from the alliance wall.
   * @param vY                DoubleSupplier that supplies the y-translation
   *                          joystick input. Should be in the range -1
   *                          to 1 with deadband already accounted for. Positive Y
   *                          is towards the left wall when
   *                          looking through the driver station glass.
   * @param headingHorizontal DoubleSupplier that supplies the horizontal
   *                          component of the robot's heading angle. In the
   *                          robot coordinate system, this is along the same axis
   *                          as vY. Should range from -1 to 1 with
   *                          no deadband. Positive is towards the left wall when
   *                          looking through the driver station
   *                          glass.
   * @param headingVertical   DoubleSupplier that supplies the vertical component
   *                          of the robot's heading angle. In the
   *                          robot coordinate system, this is along the same axis
   *                          as vX. Should range from -1 to 1
   *                          with no deadband. Positive is away from the alliance
   *                          wall.
   */
  public Drive(Swerve swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingHorizontal,
      DoubleSupplier headingVertical) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingHorizontal = headingHorizontal;
    this.headingVertical = headingVertical;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    initRotation = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double maxSpeed = 0.5;

    if (control.getRawAxis(4) != 0) {
      maxSpeed = 0.5;
    }

    // Get the desired chassis speeds based on a 2 joystick module.
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
        headingHorizontal.getAsDouble(),
        headingVertical.getAsDouble(), maxSpeed);

    // Prevent Movement After Auto
    if (initRotation) {
      if (headingHorizontal.getAsDouble() == 0 && headingVertical.getAsDouble() == 0) {
        // Get the curretHeading
        Rotation2d firstLoopHeading = swerve.getHeading();

        // Set the Current Heading to the desired Heading
        desiredSpeeds = swerve.getTargetSpeeds(0, 0, firstLoopHeading.getSin(), firstLoopHeading.getCos(),
            maxSpeed);
      }
      // Dont Init Rotation Again
      initRotation = false;
    }

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
        Drivetrain.loopTime, Drivetrain.robotMass, List.of(Constants.Drivetrain.chassi),
        swerve.getSwerveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}