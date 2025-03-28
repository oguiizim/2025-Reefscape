// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.Setpoint;
import frc.robot.commands.ElevatorMove;
import frc.robot.commands.OuttakeIntake;
import frc.robot.commands.OuttakeScore;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.drivetrain.Swerve;
import swervelib.SwerveInputStream;

public class RobotContainer {

        public Swerve swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"));
        public final Outtake outtake = new Outtake();
        public final Elevator elevator = new Elevator();
        public final Climber climber = new Climber();

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
                new EventTrigger("elevatorDown").onTrue(new ElevatorMove(elevator, 100));

                new EventTrigger("L1")
                                .onTrue(Commands.runOnce(() -> outtake.setAngleTarget(Setpoint.outtakeAngleScoreL1))
                                                .alongWith(Commands.runOnce(() -> outtake
                                                                .setTarget(Setpoint.outtakeRotationIntake))));

                new EventTrigger("scoreL1")
                                .onTrue(Commands.runOnce(() -> outtake.setWheelSpeed(-0.18)));

                NamedCommands.registerCommand("elevatorL4", new ElevatorMove(elevator, 9800));

                NamedCommands.registerCommand("L4",
                                Commands.runOnce(() -> outtake.setAngleTarget(Setpoint.outtakeAngleScoreL4))
                                                .alongWith(Commands.runOnce(() -> outtake
                                                                .setTarget(Setpoint.outtakeRotationScore))));

                NamedCommands.registerCommand("scoreL4",
                                Commands.runOnce(() -> outtake.setAngleTarget(outtake.setNewAngle()))
                                                .andThen(new ElevatorMove(elevator, 100)));

                NamedCommands.registerCommand("scoreL1N", new OuttakeScore(outtake, -0.25));

                NamedCommands.registerCommand("elevatorDown", new ElevatorMove(elevator, 100));

                NamedCommands.registerCommand("stopMotor", Commands.runOnce(() -> outtake.setWheelSpeed(0)));

                NamedCommands.registerCommand("resetOuttake",
                                Commands.runOnce(() -> outtake.setAngleTarget(Setpoint.outtakeAngleOff)));
        }

        private void configureBindings() {
                Command driveFieldOrientedVelocity = swerve.driveFieldOriented(driveAngularVelocity);
                Command driveFieldOrientedLowVelocity = swerve.driveFieldOriented(driveAngularVelocityLow);
                Command driveInvert = swerve.driveFieldOriented(driveAngularVelocityInvert);

                swerve.setDefaultCommand(driveFieldOrientedVelocity);
                driverControl.x().toggleOnTrue(driveInvert);
                driverControl.rightTrigger().whileTrue(driveFieldOrientedLowVelocity);

                // Reset Swerve Gyro
                driverControl.a().onTrue(Commands.runOnce(() -> swerve.resetGyro()));

                driverControl.b().whileTrue(
                Commands.startEnd(() -> climber.setSpeed(0.2), () -> climber.setSpeed(0),
                 climber));
                driverControl.y().whileTrue(
                 Commands.startEnd(() -> climber.setSpeed(-0.2), () -> climber.setSpeed(0),
                climber));

                // Intake Coral
                opControl.a().whileTrue(new OuttakeIntake(outtake, 0.6));
                opControl.a().onTrue(Commands.runOnce(() -> outtake.setAngleTarget(Setpoint.outtakeAngleIntake)));
                opControl.a().onTrue(Commands.runOnce(() -> outtake.setTarget(Setpoint.outtakeRotationIntake)));
                // Score Coral
                opControl.leftTrigger().whileTrue(Commands.startEnd(() -> outtake.setWheelSpeed(-0.2),
                                () -> outtake.setWheelSpeed(0.0), outtake));

                // Creeper And Outtake Return
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
                opControl.povLeft().onTrue(new ElevatorMove(elevator, 3200));

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