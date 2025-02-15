package frc.robot.subsystems;

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

public class Creeper extends SubsystemBase {
   SparkMax creeperAngle1, creeperAngle2, creeperWheel;
   SparkMaxConfig angleConfig, wConfig;
   DutyCycleEncoder angleEncoder;
   DigitalInput algaeIR;
   PIDController pid = new PIDController(1.65, 0, 0);

   public Creeper() {
      angleConfig = new SparkMaxConfig();
      wConfig = new SparkMaxConfig();
      algaeIR = new DigitalInput(DIO.sensorCreeper);

      creeperAngle1 = new SparkMax(Motors.creeperAngle1, MotorType.kBrushless);
      creeperAngle2 = new SparkMax(Motors.creeperAngle2, MotorType.kBrushless);
      creeperWheel = new SparkMax(Motors.creeperWheel, MotorType.kBrushless);
      angleEncoder = new DutyCycleEncoder(DIO.absEncoderCreeperAngle);

      angleConfig.inverted(false);
      angleConfig.idleMode(IdleMode.kBrake);
      creeperAngle1.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }

   public boolean getIR() {
      if (algaeIR.get()) {
         return false;
      }
      return true;
   }

   public double getAngle() {
      return angleEncoder.get();
   }

   public void setSpeed(double speed) {
      creeperWheel.set(speed);
   }

   public void setAngleSpeed(double speed) {
      creeperAngle1.set(speed);
      creeperAngle2.set(speed);
   }

   public void stopAngle() {
      creeperAngle1.stopMotor();
      creeperAngle2.stopMotor();
   }

   public void setTarget(double setpoint) {
      pid.setSetpoint(setpoint);
   }

   @Override
   public void periodic() {
      double output = pid.calculate(getAngle());
      output = MathUtil.clamp(output, -0.4, .4);
      setAngleSpeed(output);

      SmartDashboard.putBoolean("Creeper IR", algaeIR.get());
      SmartDashboard.putNumber("Creeper Angle", angleEncoder.get());
   }
}
