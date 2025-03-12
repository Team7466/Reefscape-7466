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

public class Dealgaefier extends SubsystemBase {
  private final SparkMax dealgaefierMotor;
  private SparkMaxConfig motorConfig;

  public Dealgaefier() {
    dealgaefierMotor = new SparkMax(DealgaefierConstants.dealgaefierMotor, MotorType.kBrushed);
    motorConfig = new SparkMaxConfig();

    setConfigs();
    applyConfigs();
  }

  /** Method to Apply the configuration to the SPARK. */
  private void applyConfigs() {
    dealgaefierMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Set parameters for the SPARK. */
  private void setConfigs() {
    motorConfig
        .smartCurrentLimit(30) // Lower for brushed motor
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.2)
        .voltageCompensation(12.0);

    motorConfig
        .signals
        .appliedOutputPeriodMs(20)
        .primaryEncoderPositionPeriodMs(500)
        .primaryEncoderVelocityPeriodMs(20);
  }

  public void set(double speed) {
    dealgaefierMotor.set(speed);
  }

  public void stop() {
    dealgaefierMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Dealgaefier Output", dealgaefierMotor.getAppliedOutput());
    SmartDashboard.putNumber("Dealgaefier Current", dealgaefierMotor.getOutputCurrent());
  }
}
