package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIO;
import frc.robot.Constants.Motors;

public class Outtake extends SubsystemBase {
    SparkMax outtakeWheel, outtakeAngle, outtakeRotation;
    SparkMaxConfig sparkConfig, angleConfig;

    // The wrist of the outtake
    DutyCycleEncoder outtakeRotationEncoder = new DutyCycleEncoder(DIO.absEncoderOuttakeRotation);

    // The arm of the outtake
    DutyCycleEncoder outtakeAngleEncoder = new DutyCycleEncoder(DIO.absEncoderOuttakeAngle);

    public Outtake() {
        outtakeWheel = new SparkMax(Motors.outtakeWheel, MotorType.kBrushless);
        outtakeRotation = new SparkMax(Motors.outtakeRotation, MotorType.kBrushless);
        outtakeAngle = new SparkMax(Motors.outtakeAngle, MotorType.kBrushless);
        sparkConfig = new SparkMaxConfig();
        angleConfig = new SparkMaxConfig();

        outtakeAngleEncoder.setInverted(true);
        sparkConfig.idleMode(IdleMode.kBrake);
        outtakeWheel.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        angleConfig.idleMode(IdleMode.kBrake);
        angleConfig.inverted(false);
        outtakeAngle.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setWheelSpeed(double speed) {
        outtakeWheel.set(speed);
    }

    public double getRotationEncoder() {
        return outtakeRotationEncoder.get();
    }

    public double getAngleEncoder() {
        return outtakeAngleEncoder.get();
    }

    public void setAngleSpeed(double speed){
        outtakeAngle.set(speed);
    }

    public void setRotationSpeed(double speed){
        outtakeRotation.set(speed);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Outtake Rotation", getRotationEncoder());
        SmartDashboard.putNumber("Outtake Angle", getAngleEncoder());

        SmartDashboard.putNumber("Wheel", outtakeWheel.getDeviceId());
    }
}
