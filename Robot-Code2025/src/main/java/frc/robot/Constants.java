// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;

public final class Constants {
  public static class Drivetrain {

    public static final int pdhID = 50;

    public static final double maxSpeed = 0.5;
    public static final double loopTime = 0.13;
    public static final double robotMass = 30;

    public static final double turnConstant = 6;

    public static final double wheelDiameterInMeters = 0.1016;
    public static final double driveGearRatio = 6.75;
    public static final double angleGearRatio = 21.43;
    public static final int encoderResolution = 1;

    public static final Matter chassi = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), robotMass);

    public static final double driveMotorConversion = SwerveMath.calculateMetersPerRotation(wheelDiameterInMeters,
        driveGearRatio, encoderResolution);
    public static final double steeringMotorConversion = SwerveMath.calculateDegreesPerSteeringRotation(angleGearRatio,
        encoderResolution);

    public static final double deadband = 0.1;

    public static final double camPitch = 45;

    public static final double multiplicadorRotacional = 0.8;
    public static final double multiplicadorTranslacionalY = 0.9;
    public static final double multiplicadorTranslacionalX = 0.9;
  }

  public static class Motors {
    public static final int outtakeWheel = 9; // DONE
    public static final int outtakeRotation = 10; // DONE
    public static final int outtakeAngle = 11; // DONE

    public static final int creeperWheel = 12; // DONE
    public static final int creeperAngle1 = 13; // DONE
    public static final int creeperAngle2 = 19; // DONE

    public static final int elevatorR = 14; // DONE
    public static final int elevatorL = 15; // DONE

    public static final int climberClaw = 16; // DONE
    public static final int climberAngle = 17; // DONE
    public static final int climberBaseAngle = 18; // DONE
  }

  public static class DIO {
    public static final int absEncoderOuttakeRotation = 1;
    public static final int absEncoderOuttakeAngle = 0;
    // public static final int sensorOuttake = 7;

    public static final int absEncoderCreeperAngle = 2;
    public static final int sensorCreeper = 9;

    // public static final int encoderElevatorRFirstPath = 3;
    // public static final int encoderElevadorRSecondPath = 4;

    // public static final int encoderElevatorLFirstPath = 5;
    // public static final int encoderElevatorLSecondPath = 6;
  }

  public static class Setpoint {
    public static final double creeperHighPosition = 0.38;
    public static final double creeperIntakePosition = 0.3;
    public static final double creeperLowPosition = 0.17;

    public static final double outtakeRotationScore = 0.42;
    public static final double outtakeRotationIntake = 0.68;

    public static final double outtakeAngleIntake = 0.65;
    public static final double outtakeAngleScoreL4 = 0.80;
    public static final double outtakeAngleScoreL3 = 0.80;
    public static final double outtakeAngleScoreL2 = 0.80;
    public static final double outtakeAngleScoreL1 = 0.80;
    public static final double outtakeAngleOff = 0.48;

    // public static final double outtakeAngleNormal = 0.459;
    // public static final double outtakeRotationNormal = 0.42;
    // public static final double creeperNormal = 0.12;
  }

  public static class PIDConstants {
    public static final double creeperP = 1.65;
    public static final double creeperI = 0.0;
    public static final double creeperD = 0.0;

  }
}