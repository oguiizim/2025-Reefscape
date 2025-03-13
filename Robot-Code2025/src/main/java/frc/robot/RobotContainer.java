// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.Setpoint;
import frc.robot.commands.CreeperIntake;
import frc.robot.commands.ElevatorMove;
import frc.robot.commands.OuttakeIntake;
import frc.robot.commands.OuttakeScore;
import frc.robot.subsystems.Creeper;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.drivetrain.Swerve;
import swervelib.SwerveInputStream;

public class RobotContainer {

        public Swerve swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"));
        public final Outtake outtake = new Outtake();
        public final Elevator elevator = new Elevator();
        public final Creeper creeper = new Creeper();

        private SendableChooser<Command> autoChooser;

        CommandXboxController driverControl = new CommandXboxController(0);
        CommandXboxController opControl = new CommandXboxController(1);

        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
                        () -> driverControl.getLeftY() * -1,
                        () -> driverControl.getLeftX() * -1)
                        .withControllerRotationAxis(() -> driverControl.getRightX() * -1)
                        .deadband(Drivetrain.deadband)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true).scaleRotation(0.6);

        SwerveInputStream driveAngularVelocityLow = SwerveInputStream.of(swerve.getSwerveDrive(),
                        () -> driverControl.getLeftY() * -1,
                        () -> driverControl.getLeftX() * -1)
                        .withControllerRotationAxis(() -> driverControl.getRightX() * -1)
                        .deadband(Drivetrain.deadband)
                        .scaleTranslation(0.08)
                        .allianceRelativeControl(true)
                        .scaleRotation(0.5);

        SwerveInputStream driveAngularVelocityInvert = SwerveInputStream.of(swerve.getSwerveDrive(),
                        () -> driverControl.getLeftY() * -1,
                        () -> driverControl.getLeftX() * -1)
                        .withControllerRotationAxis(() -> driverControl.getRightX() * 1)
                        .deadband(Drivetrain.deadband)
                        .scaleTranslation(1.0)
                        .allianceRelativeControl(true).scaleRotation(0.6);

        SwerveInputStream driveAngularVelocityLowInvert = SwerveInputStream.of(swerve.getSwerveDrive(),
                        () -> driverControl.getLeftY() * -1,
                        () -> driverControl.getLeftX() * -1)
                        .withControllerRotationAxis(() -> driverControl.getRightX() * 1)
                        .deadband(Drivetrain.deadband)
                        .scaleTranslation(0.1)
                        .allianceRelativeControl(true)
                        .scaleRotation(0.5);

        public RobotContainer() {
                setNamedCommands();

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Auto Chooser", autoChooser);
                DriverStation.silenceJoystickConnectionWarning(true);

                configureBindings();
        }

        private void setNamedCommands() {
                new EventTrigger("algaeUp").onTrue(new ElevatorMove(elevator, 9000).alongWith(new CreeperIntake(creeper,
                                Setpoint.creeperIntakePosition2, 0.5)));

                new EventTrigger("algaeDown")
                                .onTrue(new ElevatorMove(elevator, 4000).alongWith(new CreeperIntake(creeper,
                                                Setpoint.creeperIntakePosition, 0.5)));

                new EventTrigger("elevatorDown").onTrue(new ElevatorMove(elevator, 100));

                new EventTrigger("L1")
                                .onTrue(Commands.runOnce(() -> outtake.setAngleTarget(Setpoint.outtakeAngleScoreL1))
                                                .alongWith(Commands.runOnce(() -> outtake
                                                                .setTarget(Setpoint.outtakeRotationIntake))));

                new EventTrigger("L3")
                                .onTrue(Commands.runOnce(() -> outtake.setAngleTarget(Setpoint.outtakeAngleScoreL3))
                                                .alongWith(Commands.runOnce(() -> outtake
                                                                .setTarget(Setpoint.outtakeRotationScore)))
                                                .alongWith(new ElevatorMove(elevator, 2500)));

                new EventTrigger("scoreL1")
                                .onTrue(Commands.runOnce(() -> outtake.setWheelSpeed(-0.25)));

                new EventTrigger("getOutAlgae")
                                .onTrue(Commands.runOnce(() -> outtake.setAngleTarget(Setpoint.outtakeAngleIntake))
                                                .alongWith(Commands.runOnce(() -> outtake.setWheelSpeed(-0.5))));

                new EventTrigger("processor")
                                .onTrue(Commands.runOnce(() -> creeper.setTarget(Setpoint.creeperScorePosition)));

                NamedCommands.registerCommand("scoreL3",
                                Commands.runOnce(() -> outtake.setAngleTarget(Setpoint.outtakeAngleScoreL3 - 0.35)));

                NamedCommands.registerCommand("scoreL1N", new OuttakeScore(outtake, -0.25));

                NamedCommands.registerCommand("elevatorDown", new ElevatorMove(elevator, 100));

                NamedCommands.registerCommand("spinAlgae", Commands.runOnce(() -> creeper.setSpeed(-0.5)));

                NamedCommands.registerCommand("getOutAlgae",
                                Commands.runOnce(() -> outtake.setAngleTarget(Setpoint.outtakeAngleIntake))
                                                .alongWith(Commands.runOnce(() -> outtake.setWheelSpeed(-0.5))));

                NamedCommands.registerCommand("stopMotor", Commands.runOnce(() -> outtake.setWheelSpeed(0))
                                .alongWith(Commands.runOnce(() -> creeper.setSpeed(0))));

                NamedCommands.registerCommand("turn180", swerve.alignAngleInDegrees(180));

                NamedCommands.registerCommand("resetOuttake",
                                Commands.runOnce(() -> outtake.setAngleTarget(Setpoint.outtakeAngleOff)));
        }

        private void configureBindings() {
                Command driveFieldOrientedVelocity = swerve.driveFieldOriented(driveAngularVelocity);
                Command driveFieldOrientedLowVelocity = swerve.driveFieldOriented(driveAngularVelocityLow);
                Command driveInvert = swerve.driveFieldOriented(driveAngularVelocityInvert);
                // Command driveLowInvert =
                // swerve.driveFieldOriented(driveAngularVelocityLowInvert);

                swerve.setDefaultCommand(driveFieldOrientedVelocity);

                driverControl.y().toggleOnTrue(driveInvert);

                SmartDashboard.putData("Default", swerve.getDefaultCommand());

                driverControl.rightTrigger().whileTrue(driveFieldOrientedLowVelocity);

                driverControl.b().whileTrue(swerve.alignAngleInDegrees(180));

                // Reset Swerve Gyro
                driverControl.a().onTrue(Commands.runOnce(() -> swerve.resetGyro()));

                // Intake Coral
                opControl.a().whileTrue(new OuttakeIntake(outtake, 0.6));
                opControl.a().onTrue(Commands.runOnce(() -> outtake.setAngleTarget(Setpoint.outtakeAngleIntake)));
                opControl.a().onTrue(Commands.runOnce(() -> outtake.setTarget(Setpoint.outtakeRotationIntake)));
                // Score Coral
                opControl.leftTrigger().whileTrue(Commands.startEnd(() -> outtake.setWheelSpeed(-0.2),
                                () -> outtake.setWheelSpeed(0.0), outtake));

                // Intake Algae L2-L3
                opControl.b().whileTrue(new CreeperIntake(creeper,
                                Setpoint.creeperIntakePosition, 0.5));
                opControl.b().onTrue(new ElevatorMove(elevator, 4000));

                // Intake Algae L3-L4
                opControl.y().whileTrue(new CreeperIntake(creeper,
                                Setpoint.creeperIntakePosition2, 0.5));
                opControl.y().onTrue(new ElevatorMove(elevator, 9500));

                // Angle Algae to Score
                opControl.x().onTrue(Commands.runOnce(() -> creeper.setTarget(Setpoint.creeperScorePosition)));
                // Score Algae
                opControl.rightTrigger().whileTrue(
                                Commands.startEnd(() -> creeper.setSpeed(-0.4), () -> creeper.setSpeed(0),
                                                creeper));

                // Creeper And Outtake Return
                opControl.start().onTrue(Commands.runOnce(() -> creeper.setTarget(Setpoint.creeperHighPosition)));
                opControl.start().onTrue(Commands.runOnce(() -> outtake.setAngleTarget(Setpoint.outtakeAngleOff)));

                opControl.leftBumper().onTrue(
                                Commands.runOnce(() -> outtake.setAngleTarget(outtake.setNewAngle()), outtake));

                // Outtake L1
                opControl.povDown()
                                .onTrue(Commands.runOnce(() -> outtake.setAngleTarget(Setpoint.outtakeAngleScoreL1)));
                opControl.povDown().onTrue(Commands.runOnce(() -> outtake.setTarget(Setpoint.outtakeRotationIntake)));

                // Outtake L2
                opControl.povRight()
                                .onTrue(Commands.runOnce(() -> outtake.setAngleTarget(Setpoint.outtakeAngleScoreL2)));
                opControl.povRight().onTrue(Commands.runOnce(() -> outtake.setTarget(Setpoint.outtakeRotationScore)));

                // Outtake L3
                opControl.povLeft()
                                .onTrue(Commands.runOnce(() -> outtake.setAngleTarget(Setpoint.outtakeAngleScoreL3)));
                opControl.povLeft().onTrue(Commands.runOnce(() -> outtake.setTarget(Setpoint.outtakeRotationScore)));
                opControl.povLeft().onTrue(new ElevatorMove(elevator, 2500));

                // Outtake L4
                opControl.povUp().onTrue(Commands.runOnce(() -> outtake.setAngleTarget(Setpoint.outtakeAngleScoreL4)));
                opControl.povUp().onTrue(Commands.runOnce(() -> outtake.setTarget(Setpoint.outtakeRotationScore)));
                opControl.povUp().onTrue(new ElevatorMove(elevator, 9800));

                // Return Elevator
                opControl.rightBumper().onTrue(new ElevatorMove(elevator, 100));

                // Zero Elevator
                opControl.back().onTrue(Commands.runOnce(() -> elevator.setZero(), elevator));

                driverControl.rightBumper().whileTrue(Commands.startEnd(() -> elevator.setSpeed(0.3, 0.3),
                                () -> elevator.setSpeed(0, 0), elevator));

                driverControl.leftBumper().whileTrue(Commands.startEnd(() -> elevator.setSpeed(-0.3, -0.3),
                                () -> elevator.setSpeed(0, 0), elevator));
        }

        public Command getAutonomousCommand() {
                // return swerve.getAutonomousCommand("Final Processor");
                // return swerve.getAutonomousCommand("Final Mid");
                // return swerve.getAutonomousCommand("Final Barge");
                return autoChooser.getSelected();
        }

        public void setMotorBrake(boolean brake) {
                swerve.setMotorBrake(brake);
        }

        public void setHeadingCorrection(boolean heading) {
                swerve.getSwerveDrive().setHeadingCorrection(heading);
        }
}