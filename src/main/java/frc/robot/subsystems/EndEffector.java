package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
  private final SparkMax effectorMotor;
  private SparkMaxConfig motorConfig;

  public EndEffector() {
    effectorMotor = new SparkMax(EndEffectorConstants.endEffectorMotor, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    setConfigs();
    applyConfigs();
  }

  /** Method to Apply the configuration to the SPARK. */
  private void applyConfigs() {
    effectorMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Set parameters for the SPARK. */
  private void setConfigs() {
    motorConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.15)
        .voltageCompensation(12.0);

    motorConfig
        .signals
        .appliedOutputPeriodMs(10)
        .primaryEncoderPositionPeriodMs(500)
        .primaryEncoderVelocityPeriodMs(20);
  }

  public void set(double speed) {
    effectorMotor.set(speed);
  }

  public void stop() {
    effectorMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("EndEffector Output", effectorMotor.getAppliedOutput());
    SmartDashboard.putNumber("EndEffector Current", effectorMotor.getOutputCurrent());
  }
}
