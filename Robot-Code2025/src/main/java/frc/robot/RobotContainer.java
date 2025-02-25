// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.Setpoint;
import frc.robot.commands.CreeperAngle;
import frc.robot.commands.CreeperIntake;
import frc.robot.commands.CreeperScore;
import frc.robot.commands.ElevatorMove;
import frc.robot.commands.OuttakeAngle;
import frc.robot.commands.OuttakeRotation;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Creeper;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveInputStream;

public class RobotContainer {

        public Swerve swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"));
        public final Outtake outtake = new Outtake();
        public final Elevator elevator = new Elevator();
        public final Creeper creeper = new Creeper();
        public final Climber climber = new Climber();

        private SendableChooser<Command> autoChooser;

        CommandXboxController driverControl = new CommandXboxController(0);
        CommandXboxController opControl = new CommandXboxController(1);

        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
                        () -> driverControl.getLeftY() * 1,
                        () -> driverControl.getLeftX() * 1)
                        .withControllerRotationAxis(driverControl::getRightX)
                        .deadband(Drivetrain.deadband)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverControl::getRightX,
                                        driverControl::getRightY)
                        .headingWhile(true);

        SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(swerve.getSwerveDrive(),
                        () -> -driverControl.getLeftY(),
                        () -> -driverControl.getLeftX())
                        .withControllerRotationAxis(() -> driverControl.getRawAxis(
                                        2))
                        .deadband(Drivetrain.deadband)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        driverControl.getRawAxis(
                                                        2) *
                                                        Math.PI)
                                        *
                                        (Math.PI *
                                                        2),
                                        () -> Math.cos(
                                                        driverControl.getRawAxis(
                                                                        2) *
                                                                        Math.PI)
                                                        *
                                                        (Math.PI *
                                                                        2))
                        .headingWhile(true);

        public RobotContainer() {

                boolean isComp = false;

                autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                                (stream) -> isComp ? stream.filter(auto -> auto.getName().startsWith("Final"))
                                                : stream);

                SmartDashboard.putData("Auto Chooser", autoChooser);

                creeper.setDefaultCommand(new CreeperAngle(creeper, Setpoint.creeperHighPosition));

                configureBindings();
        }

        private void configureBindings() {
                Command driveFieldOrientedAnglularVelocity = swerve.driveFieldOriented(driveAngularVelocity);
                swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);

                Trigger aD = driverControl.a();

                // Reset Swerve Gyro
                new Trigger(aD).onTrue(Commands.runOnce(swerve::resetGyro, swerve));

                // Intake Coral
                opControl.a().onTrue(new OuttakeAngle(outtake, Setpoint.outtakeAngleIntake));
                opControl.a().onTrue(new OuttakeRotation(outtake, Setpoint.outtakeRotationIntake));
                opControl.a().whileTrue(Commands.startEnd(() -> outtake.setWheelSpeed(0.45),
                                () -> outtake.setWheelSpeed(0.0), outtake));

                // Intake Algae
                opControl.b().whileTrue(new CreeperIntake(creeper, Setpoint.creeperIntakePosition, 0.4));

                // Score Algae
                opControl.x().whileTrue(new CreeperScore(creeper, Setpoint.creeperLowPosition));

                opControl.povUp().onTrue(new ElevatorMove(elevator, 70));
                opControl.povUp().onTrue(new OuttakeAngle(outtake, Setpoint.outtakeAngleScoreL3));
                opControl.povUp().onTrue(new OuttakeRotation(outtake, Setpoint.outtakeRotationScore));

                opControl.povDown().onTrue(new OuttakeAngle(outtake, Setpoint.outtakeAngleScoreL2));
                opControl.povDown().onTrue(new OuttakeRotation(outtake, Setpoint.outtakeRotationScore));

                opControl.povRight().onTrue(new OuttakeAngle(outtake, Setpoint.outtakeAngleScoreL1));
                opControl.povRight().onTrue(new OuttakeRotation(outtake, Setpoint.outtakeRotationScore));

                opControl.back().onTrue(Commands.runOnce(() -> elevator.setZero(), elevator));

                // opControl.rightTrigger().whileTrue(Commands.startEnd(() ->
                // outtake.setWheelSpeed(-0.4),
                // () -> outtake.setWheelSpeed(0.0), outtake));

                // opControl.leftTrigger().whileTrue(
                // Commands.startEnd(() -> creeper.setSpeed(-0.4), () -> creeper.setSpeed(0),
                // creeper));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();

                // return swerve.getAutonomousCommand("New Auto");
        }

        public void setMotorBrake(boolean brake) {
                swerve.setMotorBrake(brake);
        }
}