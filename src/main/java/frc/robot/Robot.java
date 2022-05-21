// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveTele;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  private DriveTele drivetele = new DriveTele(RobotContainer.drivetrain, RobotContainer.rightStick, RobotContainer.leftStick);
  // private GatherAndLogData gatherAndLogData = new GatherAndLogData();

  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;
  private Joystick gamepad;
  private Drivetrain drivetrain;
  private Compressor compressor;
  private NavxGyro gyro;
  private boolean shoot = false;
  // private CommandBase desiredAutoCommand;
  private ShuffleboardTab tab = Shuffleboard.getTab("Things Tab");
  private double waitTime = 0;
  private float[] fastestSpeed = {0,0,0};
  private double lastSPeed = 0.0;
  private int i = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    gamepad = m_robotContainer.gamepad;
    // intake = m_robotContainer.intake;
    // shooter = m_robotContainer.shooter;
    drivetrain = m_robotContainer.drivetrain;
    gyro = m_robotContainer.gyro;
    // compressor = m_robotContainer.compressor;
    gyro.calibrate();
    //SmartDashboard.putNumber("Set Speed", 2800);
    tab.add("Shooter current RPM",0);
    SmartDashboard.putNumber("Left PID output", 0);
    SmartDashboard.putNumber("Right PID output", 0);

    SmartDashboard.putNumber("ltMotor.getSensorCollection.getIntegratedSensorPosition", drivetrain.ltMotor.getSensorCollection().getIntegratedSensorPosition());
    SmartDashboard.putNumber("rtMotor.getSensorCollection.getIntegratedSensorPosition", drivetrain.rtMotor.getSensorCollection().getIntegratedSensorPosition());
    SmartDashboard.putNumber("ltMotor.getSelectedSensorPosition", drivetrain.ltMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("rtMotor.getSelectedSensorPosition", drivetrain.rtMotor.getSelectedSensorPosition());
    //SmartDashboard.putNumber("volts", 1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    drivetrain.periodic();
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    drivetrain.brake();
  }

  @Override
  public void disabledPeriodic() {
    // intake.stopIntake();
    // shooter.stopShooter();
    drivetrain.brake();

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // i=0;
    CommandScheduler.getInstance().cancelAll();
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_autonomousCommand = m_robotContainer.getRamseteAutoCommand();
    m_autonomousCommand.schedule();

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand.nodeOnlyPath != null) {
    //   System.out.println("There's a path generated");
    //   updateShuffleBoard();
    // }
    // command.schedule();
    updateShuffleBoard();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // CommandScheduler.getInstance().run();
    // // System.out.println("Finished running auto command is " + m_autonomousCommand.isFinished());
    // //drivetrain.tankSpeedVelocity(m_autonomousCommand.smoothLeftVelocity[i][1], m_autonomousCommand.smoothRightVelocity[i][1]);
    // drivetrain.tankSpeedVelocity(60.0, 60.0);
    // i++;
    // if(i >= 144) {
    //   drivetrain.stop();
    // }
    updateShuffleBoard();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    drivetrain.brake();
    CommandScheduler.getInstance().cancelAll();
    drivetele.schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // CommandScheduler.getInstance().run();
   /*
    if(gamepad.getRawButton(Constants.B_Button)) {
      double voltage = SmartDashboard.getNumber("volts", 1);
      drivetrain.tankDriveVolts(voltage, voltage);
    } else {
      drivetrain.stop();
    }
    */
    // drivetrain.tankSpeedVelocity(60.0,60.0);
    // double[] outputs = drivetrain.setSpeeds(new DifferentialDriveWheelSpeeds(1, 1));

    // SmartDashboard.putNumber("Left PID output", outputs[0]);
    // SmartDashboard.putNumber("Right PID output", outputs[1]);

    updateShuffleBoard();
  }

  public void updateShuffleBoard() {

    SmartDashboard.putNumber("ltMotor.getSensorCollection.getIntegratedSensorPosition", drivetrain.ltMotor.getSensorCollection().getIntegratedSensorPosition());
    SmartDashboard.putNumber("rtMotor.getSensorCollection.getIntegratedSensorPosition", drivetrain.rtMotor.getSensorCollection().getIntegratedSensorPosition());
    SmartDashboard.putNumber("ltMotor.getSelectedSensorPosition", drivetrain.ltMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("rtMotor.getSelectedSensorPosition", drivetrain.rtMotor.getSelectedSensorPosition());
    // double speed = SmartDashboard.getNumber("Set Speed", 4800);
    // float[] currSpeeds = drivetrain.getGyroSpeeds();

    // for (int i =0; i < currSpeeds.length; i++) {
    //   if (currSpeeds[i] > fastestSpeed[i]) {
    //     fastestSpeed[i] = currSpeeds[i];
    //   }
    //   SmartDashboard.putNumber("Gyro " + i, fastestSpeed[i]);
    // }
    // double nSpeed = drivetrain.getWheelSpeeds().leftMetersPerSecond;
    // SmartDashboard.putNumber("speed",drivetrain.getWheelSpeeds().leftMetersPerSecond);
    // double accel = (nSpeed - speed) / (1/20);
    // SmartDashboard.putNumber("Accel",accel);
    // SmartDashboard.putNumber("adjSpeed",drivetrain.getAdjLeftSpeed());
    // SmartDashboard.putNumber("odometry",drivetrain.getLeftSpeed());
    // SmartDashboard.putNumber("x",drivetrain.getPose().getX());
    // SmartDashboard.putNumber("y",drivetrain.getPose().getY());
    // speed = nSpeed;
    // // System.out.println("SET SPEED IS " + speed);
    // // shooter.setSpeed(speed);
    // // SmartDashboard.putNumber("Current RPM", shooter.getLeftRPM());
    
    // // SmartDashboard.putNumber("NavX heading cos", RobotContainer.navx.getRotation2d().getCos());
    // // SmartDashboard.putNumber("NavX PID", RobotContainer.navx.pidGet());
    // // SmartDashboard.putNumber("NavX angle", RobotContainer.navx.getAngle());
    // SmartDashboard.putNumber("DriveTrain get pose", drivetrain.getWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("Gyro", drivetrain.getHeading());
    SmartDashboard.putNumber("Get adj left speed", drivetrain.getAdjLeftSpeed());
    SmartDashboard.putNumber("get left speed", drivetrain.getLeftSpeed());
   // SmartDashboard.putNumberArray("left velocity smoothed falcon path planner", RobotContainer.path.smoothLeftVelocity[5]);
    // SmartDashboard.putNumber("get gyro speedx", new Float(drivetrain.getGyroSpeeds()[0]).doubleValue());
    // SmartDashboard.putNumber("get gyro speedy", new Float(drivetrain.getGyroSpeeds()[1]).doubleValue());
    double motorOutputVolts = drivetrain.mOV;
    double rV = drivetrain.rVolts;
    //SmartDashboard.putNumber("TankDrive Volts", rV);
    //SmartDashboard.putNumber("Motor Volts", motorOutputVolts);
    boolean close = Math.round(motorOutputVolts) == Math.round(rV);
    //SmartDashboard.putBoolean("Voltages Close?", close);

  }
  
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

}
