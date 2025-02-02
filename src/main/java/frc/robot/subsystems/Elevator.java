// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class Elevator extends SubsystemBase {

  private final SparkMax elevMotor;
  private final SparkMax elevFollower;
  private SparkMaxConfig elevConfig;
  private SparkMaxConfig elevFollowerConfig;
  private RelativeEncoder throughbEncoder;

  public Elevator() {

    elevMotor = new SparkMax(7, MotorType.kBrushless);
    elevFollower = new SparkMax(8, MotorType.kBrushless);

    elevConfig = new SparkMaxConfig();
    elevFollowerConfig = new SparkMaxConfig();

    throughbEncoder = elevMotor.getAlternateEncoder();
    setConfigs();
    applyConfigs();
  }

  /** Method to Apply the configuration to the SPARKs. */
  private void applyConfigs() {
    elevMotor.configure(elevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevFollower.configure(
        elevFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Set parameters that will apply to all SPARKs. */
  private void setConfigs() {
    elevConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.25)
        .closedLoopRampRate(0.25)
        .voltageCompensation(12.0);

    elevConfig
        .encoder
        .velocityConversionFactor(Constants.DriveConstants.velocityConversionFactor)
        .positionConversionFactor(Constants.DriveConstants.positionConversionFactor);

    elevConfig.alternateEncoder.countsPerRevolution(8192);

    elevConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);

    elevConfig
        .signals
        .appliedOutputPeriodMs(5)
        .externalOrAltEncoderPositionAlwaysOn(true)
        .externalOrAltEncoderVelocityAlwaysOn(true)
        .externalOrAltEncoderVelocity(5)
        .externalOrAltEncoderPosition(5)
        .primaryEncoderPositionPeriodMs(500)
        .primaryEncoderVelocityPeriodMs(500);

    elevFollowerConfig.apply(elevConfig).follow(elevMotor);
  }

  @Override
  public void periodic() {
    // Display the applied output of the left and right side onto the dashboard
    SmartDashboard.putNumber("Elev Out", elevMotor.getAppliedOutput());
    SmartDashboard.putNumber("Elev Speed", throughbEncoder.getVelocity());
    // This method will be called once per scheduler run
  }
}
