package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final SparkMax intakeMotor;
  private SparkMaxConfig motorConfig;
  private DigitalInput beamBreak;
  private AnalogInput  infraRed;
  Debouncer debounce ;

  public Intake() {
    intakeMotor = new SparkMax(IntakeConstants.intakeMotor, MotorType.kBrushless);
    beamBreak = new DigitalInput(IntakeConstants.beamBreak);
    motorConfig = new SparkMaxConfig();
    debounce = new Debouncer(0.2, Debouncer.DebounceType.kRising);
    infraRed = new AnalogInput(3);


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
        .openLoopRampRate(0.15)
        .voltageCompensation(12.0);

    motorConfig
        .signals
        .appliedOutputPeriodMs(10)
        .primaryEncoderPositionPeriodMs(500)
        .primaryEncoderVelocityPeriodMs(20);
  }

  public boolean isCoralInside() {
    return !beamBreak.get();
    //return debounce.calculate(intakeMotor.getAppliedOutput() > 9.0 );
   // return infraRed.getVoltage()>4.0 ;
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
    SmartDashboard.putBoolean("Coral", isCoralInside());
    SmartDashboard.putNumber("infrared",infraRed.getVoltage());
  }
}
