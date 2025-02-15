package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;

public class Elevator extends SubsystemBase {
   SparkMax elevatorR, elevatorL;
   RelativeEncoder encoderR, encoderL;
   SparkMaxConfig rConfig, lConfig;

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
   }

   public void setSpeed(double speedR, double speedL) {
      elevatorL.set(speedL);
      elevatorR.set(-speedR);
   }

   public void stop() {
      elevatorL.stopMotor();
      elevatorR.stopMotor();
   }

   public double getEncoderL() {
      return encoderL.getPosition();
   }

   public double getEncoderR() {
      return encoderR.getPosition();
   }

   public void setZero() {
      encoderL.setPosition(0);
      encoderR.setPosition(0);
   }

   @Override
   public void periodic() {
      SmartDashboard.putNumber("Elevator Encoder R", elevatorR.getEncoder().getPosition());
      SmartDashboard.putNumber("Elevator Encoder L", elevatorL.getEncoder().getPosition());
   }
}
