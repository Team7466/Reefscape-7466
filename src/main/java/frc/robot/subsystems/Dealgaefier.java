// Create Dealgaefier.java:
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DealgaefierConstants;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

public class Dealgaefier extends SubsystemBase {
  private VictorSPX redLine;

  public Dealgaefier() {
    redLine = new VictorSPX(DealgaefierConstants.dealgaefierMotor);
  }

  

  public void set(double speed) {
    redLine.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public void stop() {
    redLine.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Dealgaefier Output", dealgaefierMotor.getMotorOutputPercent());
  }
}
