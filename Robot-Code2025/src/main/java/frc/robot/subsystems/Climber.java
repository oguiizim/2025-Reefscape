package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;

public class Climber extends SubsystemBase {
  SparkMax claw, baseAngle, angle;
  RelativeEncoder clawE, baseAngleE, angleE;

  public Climber() {
    claw = new SparkMax(Motors.climberClaw, MotorType.kBrushless);
    baseAngle = new SparkMax(Motors.climberBaseAngle, MotorType.kBrushless);
    angle = new SparkMax(Motors.climberAngle, MotorType.kBrushless);

    clawE = claw.getEncoder();
    baseAngleE = baseAngle.getEncoder();
    angleE = angle.getEncoder();
  }

  public void setClaw(double speed) {
    claw.set(speed);
  }

  public void setBaseAngle(double speed) {
    baseAngle.set(speed);
  }

  public void setAngle(double speed) {
    angle.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Base Angle", baseAngleE.getPosition());
    SmartDashboard.putNumber("Angle", angleE.getPosition());
  }
}
