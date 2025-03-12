// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {

  private final DifferentialDriveOdometry odometry;
  private final AHRS gyro;
  private final SparkMax leftMotor;
  private final SparkMax leftMotorFollower;
  private final SparkMax rightMotor;
  private final SparkMax rightMotorFollower;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final DifferentialDriveKinematics kinematics;
  private RobotConfig config;
  private DifferentialDrive robotDrive;
  private SparkMaxConfig globalConfig;
  private SparkMaxConfig rightConfig;
  private SparkMaxConfig leftFollowerConfig;
  private SparkMaxConfig rightFollowerConfig;

  public DriveSubsystem() {

    kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.trackWidthMeters);

    // navX Micro using usb
    gyro = new AHRS(NavXComType.kUSB1, NavXUpdateRate.k50Hz);

    // All other subsystem initialization
    leftMotor = new SparkMax(Constants.DriveConstants.leftMotor, MotorType.kBrushless);
    leftMotorFollower = new SparkMax(Constants.DriveConstants.leftFollower, MotorType.kBrushless);
    rightMotor = new SparkMax(Constants.DriveConstants.rightMotor, MotorType.kBrushless);
    rightMotorFollower = new SparkMax(Constants.DriveConstants.rightFollower, MotorType.kBrushless);

    globalConfig = new SparkMaxConfig();
    rightConfig = new SparkMaxConfig();
    leftFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig = new SparkMaxConfig();

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    setConfigs();
    applyConfigs();

    robotDrive = new DifferentialDrive(leftMotor, rightMotor);
    robotDrive.setSafetyEnabled(false);
    robotDrive.setDeadband(0.04);

    odometry =
        new DifferentialDriveOdometry(
            gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry
        this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        new PPLTVController(
            0.02), // PPLTVController is the path following controller for differential drive
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    System.out.println(pose);
    odometry.resetPosition(gyro.getRotation2d(), getCurrentPositions(), pose);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    DifferentialDriveWheelSpeeds currentSpeeds =
        new DifferentialDriveWheelSpeeds(
            leftMotor.getEncoder().getVelocity(), rightEncoder.getVelocity());
    return kinematics.toChassisSpeeds(currentSpeeds);
  }

  public DifferentialDriveWheelPositions getCurrentPositions() {
    DifferentialDriveWheelPositions positions =
        new DifferentialDriveWheelPositions(leftEncoder.getPosition(), rightEncoder.getPosition());
    return positions;
  }

  /**
   * Method to Apply the configuration to the SPARKs.
   *
   * <p>kResetSafeParameters is used to get the SPARK MAX to a known state. This is useful in case
   * the SPARK MAX is replaced.
   *
   * <p>kPersistParameters is used to ensure the configuration is not lost when the SPARK MAX loses
   * power. This is useful for power cycles that may occur mid-operation.
   */
  private void applyConfigs() {
    leftMotor.configure(
        globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotorFollower.configure(
        leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotorFollower.configure(
        rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Set parameters that will apply to all SPARKs. We will also use this as the left leader config.
   */
  private void setConfigs() {
    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.2)
        .closedLoopRampRate(0.25)
        .voltageCompensation(12.0);

    globalConfig
        .encoder
        .velocityConversionFactor(Constants.DriveConstants.velocityConversionFactor)
        .positionConversionFactor(Constants.DriveConstants.positionConversionFactor);

    globalConfig
        .signals
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(5);

    // Apply the global config and invert since it is on the opposite side
    rightConfig.apply(globalConfig).inverted(true);

    // Apply the global config and set the leader SPARK for follower mode
    leftFollowerConfig.apply(globalConfig).follow(leftMotor);

    // Apply the global config and set the leader SPARK for follower mode
    rightFollowerConfig.apply(globalConfig).follow(rightMotor);
  }

  /**
   * Command to drive the robot
   *
   * @param xSpeed Drive power (throttle). Squared for smoother controls.
   * @param zRotation Rotation in the z axis(around itself). Squared for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return run(
        () -> {
          robotDrive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble(), true);
        });
  }

  /** drive method for pathplanner */
  public void drive(ChassisSpeeds speeds) {
    robotDrive.arcadeDrive(
        speeds.vxMetersPerSecond / Constants.DriveConstants.maxSpeed,
        speeds.omegaRadiansPerSecond / 6);
  }

  @Override
  public void periodic() {
    // Display the applied output of the left and right side onto the dashboard
    SmartDashboard.putNumber("Left Out", leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("Right Out", rightMotor.getAppliedOutput());
    SmartDashboard.putNumber("Left Speed", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Speed", rightEncoder.getVelocity());
    // This method will be called once per scheduler run
  }
}
