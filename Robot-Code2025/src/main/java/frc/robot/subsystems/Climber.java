package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;

public class Climber extends SubsystemBase {
   SparkMax climberMotor;
   SparkMaxConfig angleConfig;
   RelativeEncoder encoder;
   
   public Climber() {
      climberMotor = new SparkMax(Motors.climberMotor, MotorType.kBrushless);
      angleConfig = new SparkMaxConfig();
      encoder = climberMotor.getEncoder();

      angleConfig.inverted(false);
      angleConfig.idleMode(IdleMode.kBrake);
      climberMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }

   public void setSpeed(double speed) {
      climberMotor.set(-speed);
   }

   public void stopAngle() {
      climberMotor.stopMotor();
   }

   @Override
   public void periodic() {

   }
}