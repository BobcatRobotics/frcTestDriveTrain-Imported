/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.RobotContainerConstants.gamepadPort;
import static frc.robot.Constants.RobotContainerConstants.leftStickPort;
import static frc.robot.Constants.RobotContainerConstants.rightStickPort;
import static frc.robot.Constants.RouteFinderConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.RouteFinderConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.RouteFinderConstants.kTrackwidthMeters;
import static frc.robot.Constants.RouteFinderConstants.kaVoltSecondsSquaredPerMeter;
import static frc.robot.Constants.RouteFinderConstants.ksVolts;
import static frc.robot.Constants.RouteFinderConstants.kvVoltSecondsPerMeter;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import frc.robot.utils.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.RouteFinderConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavxGyro;
// import frc.robot.utils.BobcatTrajectoryCreater;
import frc.robot.utils.FalconPathPlanner;

public class RobotContainer {
  // Subsystems should be private here and have to be passed to commands because it is better coding practice.

  // Joysticks
  public static final Joystick leftStick = new Joystick(leftStickPort);
  public static final Joystick rightStick = new Joystick(rightStickPort);
  public static final Joystick gamepad = new Joystick(gamepadPort);
  
  // Drivetrain
  public static final Drivetrain drivetrain = new Drivetrain();
  
  // Network Table
  public static NetworkTable py_vision_network_table;

  //Gyro
  public static final NavxGyro gyro = new NavxGyro(SPI.Port.kMXP);

  public static FalconPathPlanner path;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {}

  public static TrajectoryConfig getConfig(DifferentialDriveVoltageConstraint constraint) {
    var kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = constraint;

    // Create config for trajectory
    return new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint)
        // Doesn't reverse the trajectory
        .setReversed(false);
  }

  /**
   * 
   * @return the command to run in autonomous
   */
  public static Ball getClosestBall() {
    py_vision_network_table = NetworkTableInstance.getDefault().getTable("pyVision");
    String jsonString = py_vision_network_table.getEntry("jsonData").getString("");
    GsonBuilder gsonBuilder = new GsonBuilder();
    Gson gson = gsonBuilder.create();
    Ball[] ball_array = gson.fromJson(jsonString, Ball[].class);
    // No JSON / no ball in sight
    if (ball_array == null || ball_array.length == 0) {
      return null;
    }
    // At least one ball - figure out closest ball
    // TODO: Get this value from start of game
    String teamColor = "red";
    Ball closestBall = null;
    for (Ball ball: ball_array) {
      // System.out.println(ball);
      if (closestBall == null) closestBall = ball;
      // Check if next ball is closer than current ball
      if (ball.getRadius() > closestBall.getRadius()) closestBall = ball;
    }
    return closestBall;
  }

  private double safeAutoTurnSpeed = 0.1; // TODO: Tune this (I'm guessing it's between -1 and 1)
  private double safeAutoForwardSpeed = 0.1; // TODO: Tune this (I'm guessing it's between -1 and 1)
 /*
  public Command getAutonomousCommand() {
    drivetrain.resetEncoders();
    drivetrain.resetOdometry();
    drivetrain.zeroHeading();
    var kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    var limiter = 4;
    TrajectoryConfig config = new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    config.setKinematics(kDriveKinematics);
    Trajectory trajectory = null;
    String trajectoryJSON = "paths/straightleft.wpilib.json";
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);

    try {
      trajectory = BobcatTrajectoryCreater.fromWaypoints(trajectoryPath, config);
    } catch (Exception e) {
      DriverStation.reportError("Unable to create trajectory: " + trajectoryJSON, e.getStackTrace());
    }
    if (trajectory == null) {
      System.out.println("WE AINT FOUND SHIT");
      return null;
    }

    RamseteController disabledRamsete = new RamseteController() {
      @Override
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
              double angularVelocityRefRadiansPerSecond) {
          return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
      }
    };

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
    RamseteCommand command = new RamseteCommand(
      trajectory,
      drivetrain::getPose,
      disabledRamsete,//new RamseteController(2.0, 0.7),
      feedforward,
      kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(Constants.RouteFinderConstants.kPDriveVel, 0, 0),
      new PIDController(Constants.RouteFinderConstants.kPDriveVel, 0, 0),
      (leftVolts, rightVolts) -> {
        leftVolts = leftVolts * (limiter/12);
        rightVolts = rightVolts * (limiter/12);
        drivetrain.tankDriveVolts(leftVolts,rightVolts);
      },
      drivetrain
    );

    return command;
  }*/


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
    public FalconPathPlanner getAutonomousCommand() {
      drivetrain.resetEncoders();
      drivetrain.resetOdometry();
      drivetrain.zeroHeading();
      var kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

      //create waypoint path
      double[][] waypoints = new double[][]{
        {1, 1},
        {5, 1},
        {9, 12},
        {12, 9},
        {15, 6},
        {19, 12}
      }; 

      double totalTime = 15; //max seconds we want to drive the path
      double timeStep = 0.02; //period of control loop on Rio, seconds
      double robotTrackWidth = 1.87; //distance between left and right wheels, feet

      path = new FalconPathPlanner(waypoints);
      path.calculate(totalTime, timeStep, robotTrackWidth);

      return path;
  }



  public Command getRamseteAutoCommand() {

    // drivetrain.resetEncoders();

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
              RouteFinderConstants.ksVolts,
                RouteFinderConstants.kvVoltSecondsPerMeter,
                RouteFinderConstants.kaVoltSecondsSquaredPerMeter),
                RouteFinderConstants.kDriveKinematics,
            3);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                RouteFinderConstants.kMaxSpeedMetersPerSecond,
                RouteFinderConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(RouteFinderConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory straightLineTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through line
            List.of(new Translation2d(0.5, 0)),
            // End 1 meters straight ahead of where we started, facing forward
            new Pose2d(1, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            straightLineTrajectory,
            drivetrain::getPose,
            new RamseteController(RouteFinderConstants.kRamseteB, RouteFinderConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
              RouteFinderConstants.ksVolts,
              RouteFinderConstants.kvVoltSecondsPerMeter,
              RouteFinderConstants.kaVoltSecondsSquaredPerMeter),
              RouteFinderConstants.kDriveKinematics,
            drivetrain::getWheelSpeeds,
            new PIDController(RouteFinderConstants.kPDriveVel, 0, 0),
            new PIDController(RouteFinderConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrain::tankDriveVolts,
            drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(straightLineTrajectory.getInitialPose());

    // Timer time = new Timer();
    // time.reset();
    // time.start();
    // while (!time.hasElapsed(2));

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }

}