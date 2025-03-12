// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kOperatorControllerPort = 2;
  }

  public static class DriveConstants {
    public static final int leftMotor = 3; // CAN ID
    public static final int leftFollower = 4; // CAN ID
    public static final int rightMotor = 5; // CAN ID
    public static final int rightFollower = 6; // CAN ID

    public static final double trackWidthMeters = 0.5308; // meters
    public static final double gearRatio = 10.71428;
    public static final double maxSpeed = 4.23; // meters per second
    public static final double wheelCircumference = Units.inchesToMeters(6 * Math.PI);
    public static final double velocityConversionFactor =
        (1.0 / gearRatio) * (wheelCircumference) / 60.0;
    public static final double positionConversionFactor =
        (1.0 / gearRatio) * (wheelCircumference);
  }

  public static class ElevatorConstants {
    public static final double positionConversionFactor = 0.1029;
    public static final double velocityConversionFactor = 1.0;
    public static final int elevMotor = 8; // CAN ID
    public static final int elevFollower = 9; // CAN ID
  }

  public static class EndEffectorConstants {
    public static final int endEffectorMotor = 10; // CAN ID
  }

  public static class IntakeConstants {
    public static final int intakeMotor = 11; // CAN ID
  }

  public static class DealgaefierConstants {
    public static final int dealgaefierMotor = 12; // CAN ID
  }
}
