package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIO;
import frc.robot.Constants.Motors;
import frc.robot.Constants.Setpoint;

public class Outtake extends SubsystemBase {
    SparkMax outtakeAngle, outtakeRotation;
    SparkMaxConfig angleConfig;
    DigitalInput coralIR;
    TalonFX outtakeWheel;

    // The wrist of the outtake
    DutyCycleEncoder outtakeRotationEncoder = new DutyCycleEncoder(DIO.absEncoderOuttakeRotation);

    // The arm of the outtake
    DutyCycleEncoder outtakeAngleEncoder = new DutyCycleEncoder(DIO.absEncoderOuttakeAngle);
    PIDController pidRotation = new PIDController(1.2, 0.0, 0.0);
    PIDController pidAngle = new PIDController(4.2, 0, 0.0);
    PIDController newPidAngle = new PIDController(0.7, 0., 0.0);

    public Outtake() {
        outtakeWheel = new TalonFX(Motors.outtakeWheel);
        outtakeRotation = new SparkMax(Motors.outtakeRotation, MotorType.kBrushless);
        outtakeAngle = new SparkMax(Motors.outtakeAngle, MotorType.kBrushless);
        angleConfig = new SparkMaxConfig();

        coralIR = new DigitalInput(DIO.sensorOuttake);

        outtakeAngleEncoder.setInverted(true);

        angleConfig.idleMode(IdleMode.kBrake);
        angleConfig.inverted(false);
        outtakeAngle.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean getIR() {
        if (coralIR.get()) {
            return false;
        }
        return true;
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

    public void setAngleSpeed(double speed) {
        outtakeAngle.set(-speed);
    }

    public void setRotationSpeed(double speed) {
        outtakeRotation.set(speed);
    }

    public void setTarget(double setpoint) {
        pidRotation.setSetpoint(setpoint);
    }

    public void setAngleTarget(double setpoint) {
        pidAngle.setSetpoint(setpoint);
    }

    public double getAngleSetpoint() {
        return pidAngle.getSetpoint();
    }

    public double setNewAngle() {
        return pidAngle.getSetpoint() - 0.15;
    }

    @Override
    public void periodic() {
        double outputRotation = pidRotation.calculate(getRotationEncoder());
        setRotationSpeed(MathUtil.clamp(outputRotation, -0.1, 0.1));

        double outputAngle = pidAngle.calculate(getAngleEncoder());
        setAngleSpeed(MathUtil.clamp(outputAngle, -0.2, 0.4));
        
        if (getAngleSetpoint() == Setpoint.outtakeAngleOff) {
            double outputReturn = newPidAngle.calculate(getAngleEncoder(),
                    Setpoint.outtakeAngleOff);
            setAngleSpeed(MathUtil.clamp(outputReturn, -0.2, 0.2));
        }

        if (getAngleSetpoint() <= getAngleEncoder()) {
            double outputDown = newPidAngle.calculate(getAngleEncoder(),
                    getAngleSetpoint());
            setAngleSpeed(MathUtil.clamp(outputDown, -0.2, 0.2));
        }

        SmartDashboard.putNumber("Outtake Rotation", getRotationEncoder());
        SmartDashboard.putNumber("Outtake Angle", getAngleEncoder());
        SmartDashboard.putBoolean("Outtake IR", getIR());
        SmartDashboard.putBoolean("IsTrue", outtakeAngleEncoder.isConnected());
    }
}