// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private final DifferentialDriveOdometry odometry;
  private final AHRS gyro;
  private final SparkMax leftMotor;
  private final SparkMax leftMotor2;
  private final SparkMax rightMotor;
  private final SparkMax rightMotor2;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final DifferentialDriveKinematics kinematics;
  private RobotConfig config;

  public DriveSubsystem() {

    kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.trackWidthMeters);

    // navX MXP using SPI
    gyro = new AHRS(NavXComType.kMXP_SPI);

    // All other subsystem initialization
    leftMotor= new SparkMax(3, MotorType.kBrushless);
    leftMotor2= new SparkMax(4, MotorType.kBrushless);
    rightMotor= new SparkMax(5, MotorType.kBrushless);
    rightMotor2= new SparkMax(6, MotorType.kBrushless);

    leftEncoder=leftMotor.getEncoder();
    rightEncoder=rightMotor.getEncoder();

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(),
    leftEncoder.getPosition(), rightEncoder.getPosition()
  );




    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
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


   

  private void driveRobotRelative(ChassisSpeeds speeds) {

    throw new UnsupportedOperationException("Unimplemented method 'driveRobotRelative'");
  }
 

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    System.out.println(pose);
    odometry.resetPosition(gyro.getRotation2d(), getCurrentPositions(), pose);
  }
   
  public ChassisSpeeds getCurrentSpeeds(){
    DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(leftMotor.getEncoder().getVelocity(), 0);
    return kinematics.toChassisSpeeds( speeds);
  }

  public DifferentialDriveWheelPositions getCurrentPositions(){
   DifferentialDriveWheelPositions positions = new DifferentialDriveWheelPositions(leftEncoder.getPosition(), rightEncoder.getPosition());  
   return positions ;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
