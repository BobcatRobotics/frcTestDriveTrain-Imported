/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  /* CONSTANTS INFO
   * From the official wpilib docs:
   * "It is recommended that users separate these constants 
   * into individual inner classes corresponding to subsystems or robot modes,
   * to keep variable names shorter."
   * "In Java, all constants should be declared public static final
   * so that they are globally accessible and cannot be changed."
   * "In Java, it is recommended that the constants be used from other classes
   * by statically importing the necessary inner class."
   */

  //Button definitions for gamepad
  public static final int A_Button = 2;
  public static final int B_Button = 3;
  public static final int X_Button = 1;
  public static final int Y_Button = 4;

  public static final int Right_Bumper_Button = 6;
  public static final int Right_Trigger_Button = 8;
  public static final int Left_Bumper_Button = 5;
  public static final int Left_Trigger_Button = 7;

  public static final int Left_Joystick_Pressed = 11;
  public static final int Right_Joystick_Pressed = 12;


  // public static final int Left_Joystick_X_Axis = 1;
  // public static final int Left_Joystick_Y_Axis = 0;

  // public static final int Right_Joystick_X_Axis= 13;
  // public static final int Right_Joystick_Y_Axis= 14;

  public static final int D_Pad_Up = 0;
  public static final int D_Pad_Down = 180;
  public static final int D_Pad_Left = 270;
  public static final int D_Pad_Right = 90;


  public static final String SHOOT_SCOOT = "Shoot & Scoot";
  public static final String SCOOT_N_SHOOT = "Scoot & Shoot";
  public static final String TRENCH_RUN = "Trench Run";
  public static final String PUSH = "PUSH";
  public static final String DONT_PUSH = "DONT_PUSH";

  // public static final int RS_Shift_Switch = 3;

   /**
    * Constants for the drive train
    */
  public static final class DrivetrainConstants {

    public static final double INVERT_MOTOR = -1.0;
    // Drive Train motors
    public static final int LTMotorPort = 0;
    public static final int LMMotorPort = 1;
    public static final int LLMotorPort = 2;
    public static final int RTMotorPort = 3;
    public static final int RMMotorPort = 4; 
    public static final int RLMotorPort = 5;
    public static final double gearRatio = 30.75; //9.375; feet not meters anymore
    public static final double encoderTicksPerRev = 2048;
    public static final double wheelCircumferenceInches = Math.PI*6;
    public static final double wheelDiameterMeters = 0.1524;
    public static final int encoderCPR = 2048;

    // Need to edit all the numbers under this for 2020
    // public static final int[] leftEncoderPorts = new int[] { 0, 1 };
    // public static final int[] rightEncoderPorts = new int[] { 3, 4 };
    public static final boolean leftEncoderReversed = false;
    public static final boolean rightEncoderReversed = false;

    //not used
    public static final double encoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (wheelDiameterMeters * Math.PI) / (double) encoderCPR;

    public static final boolean GYRO_REVERSED = true;
  }

  /**
   * Constants for the input device ports
   */
  public static final class RobotContainerConstants {
    // Sticks
    public static final int leftStickPort = 0;
    public static final int rightStickPort = 1;
    public static final int gamepadPort = 2;

    // Gamepad POV values in degrees
    // public static final int povNone = -1;
    public static final int povUp = 0;
    // public static final int povUpperRight = 45;
    public static final int povRight = 90;
    // public static final int povLowerRight = 135;
    public static final int povDown = 180;
    // public static final int povLowerLeft = 225;
    public static final int povLeft = 270;
    // public static final int povUpperLeft = 315;
  }

  /**
   * Constants for path finding and trajectories
   */
  public static final class RouteFinderConstants {
    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining
    // these values for your robot.
    // public static final double ksVolts = 0.574;
    public static final double ksVolts = 0.59844;
    // public static final double kvVoltSecondsPerMeter = 2.111;
    public static final double kvVoltSecondsPerMeter = 2.1274;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.132;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.13245;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0389; // Romi value
    public static final double kTrackwidthMeters = 0.566;

    // Example value only - as above, this must be tuned for your drive!
    // public static final double kPDriveVel = .010268;
    public static final double kPDriveVel = .0054816;
    public static final double kMaxVelocityError = 0.25; // (m/s)
    public static final double kMaxControlEffort = 7; // (v)

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // On-the-go route finding waypoint variables
    public static final int pointx = 0;
    public static final int pointy = 0;
    public static final int rotation = 0;

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
  }
}
