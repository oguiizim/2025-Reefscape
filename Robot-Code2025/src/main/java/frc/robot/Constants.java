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

    public static final double maxSpeed = 4.5; // 4.72
    public static final double loopTime = 0.13;
    public static final double robotMass = 108.03;

    public static final double turnConstant = 6;

    public static final double wheelDiameterInMeters = Units.inchesToMeters(4);
    public static final double driveGearRatio = 6.75;
    public static final double angleGearRatio = 21.43;
    public static final int encoderResolution = 1;

    public static final Matter chassi = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), robotMass);

    public static final double driveMotorConversion = SwerveMath.calculateMetersPerRotation(wheelDiameterInMeters,
        driveGearRatio, encoderResolution);
    public static final double steeringMotorConversion = SwerveMath.calculateDegreesPerSteeringRotation(angleGearRatio,
        encoderResolution);
    public static final double deadband = 0.1;

    public static final double camPitch = 60;

    public static final double multiplicadorRotacional = 0.8;
    public static final double multiplicadorTranslacionalY = 0.9;
    public static final double multiplicadorTranslacionalX = 0.9;
  }

  public static class Motors {
    public static final int outtakeWheel = 9; // DONE
    public static final int outtakeRotation = 10; // DONE
    public static final int outtakeAngle = 11; // DONE

    public static final int climberMotor = 12; // DONE
    
    public static final int elevatorR = 14; // DONE
    public static final int elevatorL = 15; // DONE
  }

  public static class DIO {
    public static final int absEncoderOuttakeRotation = 0;
    public static final int absEncoderOuttakeAngle = 1;
    public static final int sensorOuttake = 5;

    public static final int elevatorP1 = 3;
    public static final int elevatorP2 = 4;
  }

  public static class Setpoint {
    private static final double outtakeRotationNormal = 0.25;
    public static final double outtakeRotationScore = 0.412;
    public static final double outtakeRotationIntake = 0.15;

    private static final double outtakeAngleNormal = 0.431;
    public static final double outtakeAngleIntake = outtakeAngleNormal + 0.33; // Angle Normal + 0.339
    public static final double outtakeAngleScoreL1 = outtakeAngleNormal + 0.2;
    public static final double outtakeAngleScoreL2 = outtakeAngleNormal + 0.33;
    public static final double outtakeAngleScoreL3 = outtakeAngleNormal + 0.4;
    public static final double outtakeAngleScoreL4 = outtakeAngleNormal + 0.39;
    public static final double outtakeAngleOff = outtakeAngleNormal + 0.12; // Angle Normal + 0.024

    public static final double distanceRobotToReef = 1.0;
  }

  public static final class Strings {
    public static final String ON = "ON";
    public static final String OFF = "OFF";
  }
}