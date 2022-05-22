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
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.RouteFinderConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavxGyro;

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
            3); // max voltage is 3 just to make sure testing yields slow motion along path

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
    
    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }

}