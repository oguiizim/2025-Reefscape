package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIO;
import frc.robot.Constants.Motors;

public class Elevator extends SubsystemBase {
   SparkMax elevatorR, elevatorL;
   RelativeEncoder encoderR, encoderL;
   SparkMaxConfig rConfig, lConfig;
   Encoder encoder;

   public Elevator() {
      rConfig = new SparkMaxConfig();
      lConfig = new SparkMaxConfig();

      elevatorR = new SparkMax(Motors.elevatorR, MotorType.kBrushless);
      elevatorR.configure(rConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      elevatorL = new SparkMax(Motors.elevatorL, MotorType.kBrushless);
      elevatorL.configure(lConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      rConfig.inverted(true);
      rConfig.idleMode(IdleMode.kBrake);

      lConfig.inverted(true);
      lConfig.idleMode(IdleMode.kBrake);

      encoderR = elevatorR.getEncoder();
      encoderL = elevatorL.getEncoder();

      encoder = new Encoder(DIO.elevatorP1, DIO.elevatorP2);
      encoder.setReverseDirection(true);
   }

   public void setSpeed(double speedR, double speedL) {
      elevatorL.set(speedL);
      elevatorR.set(-speedR);
   }

   public void stop() {
      elevatorL.stopMotor();
      elevatorR.stopMotor();
   }

   public void setZero() {
      encoder.reset();
   }

   public double getEncoder() {
      return encoder.get();
   }

   public double getVelocity(){
      return encoderR.getVelocity();
   }

   @Override
   public void periodic() {
      SmartDashboard.putNumber("Elevator Encoder", getEncoder());
      SmartDashboard.putNumber("Velo1", encoderR.getVelocity());
      SmartDashboard.putNumber("Velo2", encoderL.getVelocity());
   }
}
