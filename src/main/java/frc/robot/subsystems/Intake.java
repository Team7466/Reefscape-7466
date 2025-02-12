package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final SparkMax intakeMotor;
  private SparkMaxConfig motorConfig;

  public Intake() {
    intakeMotor = new SparkMax(IntakeConstants.intakeMotor, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    setConfigs();
    applyConfigs();
  }

  /** Method to Apply the configuration to the SPARK. */
  private void applyConfigs() {
    intakeMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Set parameters for the SPARK. */
  private void setConfigs() {
    motorConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.1)
        .voltageCompensation(12.0);

    motorConfig
        .signals
        .appliedOutputPeriodMs(10)
        .primaryEncoderPositionPeriodMs(500)
        .primaryEncoderVelocityPeriodMs(20);
  }

  public void set(double speed) {
    intakeMotor.set(speed);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Output", intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
  }
}
