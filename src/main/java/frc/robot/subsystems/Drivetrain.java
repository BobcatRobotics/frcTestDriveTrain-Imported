package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.GYRO_REVERSED;
import static frc.robot.Constants.DrivetrainConstants.INVERT_MOTOR;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

   private WPI_TalonFX ltMotor;
   private WPI_TalonFX lmMotor;
   private WPI_TalonFX lbMotor;
   private WPI_TalonFX rtMotor;
   private WPI_TalonFX rmMotor;
   private WPI_TalonFX rbMotor;

   private MotorControllerGroup leftMotorGroup;
   private MotorControllerGroup rightMotorGroup;
   private final DifferentialDrive m_drive;

   private TalonSRX srx;
   public double rVolts;
   public double mOV;
   
   public SimpleMotorFeedforward m_feedforward;
   public PIDController m_leftPIDController = new PIDController(Constants.RouteFinderConstants.kPDriveVel,0,0);
   public PIDController m_rightPIDController = new PIDController(Constants.RouteFinderConstants.kPDriveVel,0,0);
    // The motors on the left side of the drive.
    // private final SpeedControllerGroup leftMotors =
    /* new SpeedControllerGroup(
        new WPI_TalonFX(LTMotorPort),
        new WPI_TalonFX(LMMotorPort),
        new WPI_TalonFX(LLMotorPort)); */

    // The motors on the right side of the drive.
   // private final SpeedControllerGroup rightMotors = 
    /* new SpeedControllerGroup(
        new WPI_TalonFX(RTMotorPort),
        new WPI_TalonFX(RMMotorPort),
        new WPI_TalonFX(RLMotorPort))d;
 */
    // motor properties
    private double rightPower = 0.0;
    private double leftPower = 0.0;

    private boolean invertRight = false; // Whether or not to invert the right motor
    private boolean invertLeft = true; // Whether or not to invert the left motor
    private double voltageRegScaleFactor = 1.0; // TODO: Use this to adjust motor speed

    public double getVoltageRegScaleFactor() {
        return voltageRegScaleFactor;
    }

    //private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors); // The robot's drive


    private final AHRS gyro = new AHRS(SPI.Port.kMXP); // The gyro sensor

    private final DifferentialDriveOdometry odometry; // Odometry class for tracking robot pose
    /**
     * Method use to drive the robot
     */
    public Drivetrain() {
        //ChassisSpeeds speeds = new ChassisSpeeds(,,Math.PI); figure out what to do with this
        ltMotor = new WPI_TalonFX(Constants.DrivetrainConstants.LTMotorPort);
        lmMotor = new WPI_TalonFX(Constants.DrivetrainConstants.LMMotorPort);
        lbMotor = new WPI_TalonFX(Constants.DrivetrainConstants.LLMotorPort);
        rtMotor = new WPI_TalonFX(Constants.DrivetrainConstants.RTMotorPort);
        rmMotor = new WPI_TalonFX(Constants.DrivetrainConstants.RMMotorPort);
        rbMotor = new WPI_TalonFX(Constants.DrivetrainConstants.RLMotorPort);
        // ltMotor.config_kP(0, Constants.RouteFinderConstants.kPDriveVel,0);
        // ltMotor.config_kI(0, 0);
        // ltMotor.config_kD(0,0,0);
        // lmMotor.config_kP(0, Constants.RouteFinderConstants.kPDriveVel,0);
        // lmMotor.config_kI(0, 0,0);
        // lmMotor.config_kD(0,0,0);
        // lbMotor.config_kP(0, Constants.RouteFinderConstants.kPDriveVel,0);
        // lbMotor.config_kI(0, 0,0);
        // lbMotor.config_kD(0,0,0);
        // rtMotor.config_kP(0, Constants.RouteFinderConstants.kPDriveVel,0);
        // rtMotor.config_kI(0, 0,0);
        // rtMotor.config_kD(0,0,0);
        // rmMotor.config_kP(0, Constants.RouteFinderConstants.kPDriveVel,0);
        // rmMotor.config_kI(0, 0,0);
        // rmMotor.config_kD(0,0,0);
        // rbMotor.config_kP(0, Constants.RouteFinderConstants.kPDriveVel,0);
        // rbMotor.config_kI(0, 0,0);
        // rbMotor.config_kD(0,0,0);
        ltMotor.configFactoryDefault();
        lmMotor.configFactoryDefault();
        lbMotor.configFactoryDefault();
        rtMotor.configFactoryDefault();
        rmMotor.configFactoryDefault();
        rbMotor.configFactoryDefault();

        // ltMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        // ltMotor.setSelectedSensorPosition(0, 0, 0);
        // ltMotor.setSensorPhase(false);
        // lmMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        // lmMotor.setSelectedSensorPosition(0, 0, 0);
        // lmMotor.setSensorPhase(false);
        // lbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        // lbMotor.setSelectedSensorPosition(0, 0, 0);
        // lbMotor.setSensorPhase(false);
        // rtMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        // rtMotor.setSelectedSensorPosition(0, 0, 0);
        // rtMotor.setSensorPhase(false);
        // rmMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        // rmMotor.setSelectedSensorPosition(0, 0, 0);
        // rmMotor.setSensorPhase(false);
        // rbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        // rbMotor.setSelectedSensorPosition(0, 0, 0);
        // rbMotor.setSensorPhase(false);

        ltMotor.setNeutralMode(NeutralMode.Brake);
        lmMotor.setNeutralMode(NeutralMode.Brake);
        lbMotor.setNeutralMode(NeutralMode.Brake);
        rtMotor.setNeutralMode(NeutralMode.Brake);
        rmMotor.setNeutralMode(NeutralMode.Brake);
        rbMotor.setNeutralMode(NeutralMode.Brake);

        ltMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        lmMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        lbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        rtMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        rmMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        rbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);

        m_feedforward = new SimpleMotorFeedforward(Constants.RouteFinderConstants.ksVolts, Constants.RouteFinderConstants.kvVoltSecondsPerMeter);

        leftMotorGroup = new MotorControllerGroup(ltMotor, lmMotor, lbMotor);
        rightMotorGroup = new MotorControllerGroup(rtMotor, rmMotor, rbMotor);
        rightMotorGroup.setInverted(true);
        m_drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
        
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        resetEncoders();
        zeroHeading();
    }

    public double[] setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        final double leftFeedForward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedForward = m_feedforward.calculate(speeds.rightMetersPerSecond);

        final double leftOutput = m_leftPIDController.calculate(lmMotor.getSelectedSensorVelocity());
        final double rightOutput = m_rightPIDController.calculate(rmMotor.getSelectedSensorVelocity());

        leftMotorGroup.setVoltage(leftOutput+leftFeedForward);
        rightMotorGroup.setVoltage(rightOutput+rightFeedForward);

        return new double[]{leftOutput+leftFeedForward, rightOutput+rightFeedForward};

        // Encoder e = new Encoder(1,2);
        // e.getRate();
        // lmMotor.getSelectedSensorVelocity();
    }

    public void brake() {
        ltMotor.setNeutralMode(NeutralMode.Brake);
        lmMotor.setNeutralMode(NeutralMode.Brake);
        lbMotor.setNeutralMode(NeutralMode.Brake);
        rtMotor.setNeutralMode(NeutralMode.Brake);
        rmMotor.setNeutralMode(NeutralMode.Brake);
        rbMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void coast() {
        ltMotor.setNeutralMode(NeutralMode.Coast);
        lmMotor.setNeutralMode(NeutralMode.Coast);
        lbMotor.setNeutralMode(NeutralMode.Coast);
        rtMotor.setNeutralMode(NeutralMode.Coast);
        rmMotor.setNeutralMode(NeutralMode.Coast);
        rbMotor.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Get the left motor power
     * @return The left motor power
     */
    public double getLeftPower() {
        return leftPower;
    }

    /**
     * Get the right motor power
     * @return The right motor power
     */
    public double getRightPower() {
        return rightPower;
    }

    /**
     * Set the right power
     * @param pwr Value to set the power to
     */
     public void setRightPower(double pwr) {
         if (pwr > 1.0) {
             rightPower = 1.0;
             return;
         } else if (pwr < -1.0) {
             rightPower = -1.0;
             return;
         }

         rightPower = pwr;
     }

     /**
      * Set the left power
      * @param pwr Value to set the power to
      */
     public void setLeftPower(double pwr) {
        if (pwr > 1.0) {
            leftPower = 1.0;
            return;
        } else if (pwr < -1.0) {
            leftPower = -1.0;
            return;
        }

        leftPower = pwr;
     }

     /**
      * Stop the right motor
      */
     public void stopRightMotor() {
         rightPower = 0.0;
         rtMotor.stopMotor();
         rmMotor.stopMotor();
         rbMotor.stopMotor();
         
     }

    /**
     * Stop the left motor
     */
    public void stopLeftMotor() {
        leftPower = 0.0;
        ltMotor.stopMotor();
        lmMotor.stopMotor();
        lbMotor.stopMotor();
    }

    /**
     * Drive with default values from the joysticks
     */
    public void drive() {
        drive(rightPower, leftPower);
    }
    
    /**
     * Drive with custom values
     * @param rightP Right motor power
     * @param leftP Left motor power
     */
    public void drive(double rightP, double leftP) {
        if (invertRight) {
            rightP *= INVERT_MOTOR;
        }
        
        if (invertLeft) {
            leftP *= INVERT_MOTOR;
        }
        
        rtMotor.set(rightP);
        rmMotor.set(rightP);
        rbMotor.set(rightP);

        ltMotor.set(leftP);
        lmMotor.set(leftP);
        lbMotor.set(leftP);
    }

    public void checkmotors(){
        rtMotor.getSelectedSensorVelocity();
        rmMotor.getSelectedSensorVelocity();
        rbMotor.getSelectedSensorVelocity();
        ltMotor.getSelectedSensorVelocity();
        lmMotor.getSelectedSensorVelocity();
        lbMotor.getSelectedSensorVelocity();
    }
        

    /**
     * Stops the drive train
     */
    public void stop() {
        stopRightMotor();
        stopLeftMotor();
    }

    /**
     * Called periodically by the CommmandScheduler
     */
    @Override
    public void periodic() {
        //odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(), rightEncoder.getDistance()); // Update the odometry in the periodic block (gets location on field)
        //periodic called ever .02s
        odometry.update(Rotation2d.fromDegrees(getHeading()), ltMotor.getSensorCollection().getIntegratedSensorPosition(), rtMotor.getSensorCollection().getIntegratedSensorPosition());
    }

    /**
     * Returns the currently-estimated pose of the robot
     * @return The pose. (position on the field)
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        //return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
        double left = 10.0 * (ltMotor.getSelectedSensorVelocity() * 60)/ (Constants.DrivetrainConstants.gearRatio * Constants.DrivetrainConstants.encoderTicksPerRev);
        double right = 10.0* (rtMotor.getSelectedSensorVelocity() * 60)/ (Constants.DrivetrainConstants.gearRatio *  Constants.DrivetrainConstants.encoderTicksPerRev);
        return new DifferentialDriveWheelSpeeds(left,right);
                                            
    }
    public double getAdjLeftSpeed() {
        return getWheelSpeeds().leftMetersPerSecond;
    }
    public double getLeftSpeed() {
        return ltMotor.getSelectedSensorVelocity();
    }

    public float[] getGyroSpeeds() {
        float[] rVal = {gyro.getVelocityX(),gyro.getVelocityY(),gyro.getVelocityY()};
        return rVal;
    }

    /**
     * Resets the odometry to a default pose Pose2d(0, 0, new Rotation2d(0))
     * Resets the encoders, also automatically resets heading
     */
    public void resetOdometry() {
        resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    }

    /**
     * Resets the odometry to the specified pose
     * Resets the encoders, also automatically resets heading
     * @param pose The pose to which to set the odometry
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, pose.getRotation());
    }

    /**
     * Drives the robot using arcade controls
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        //diffDrive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        // ltMotor.setVoltage(leftVolts);
        // lmMotor.setVoltage(leftVolts);
        // lbMotor.setVoltage(leftVolts);
        // rtMotor.setVoltage(-rightVolts);
        // rmMotor.setVoltage(-rightVolts);
        // rbMotor.setVoltage(-rightVolts);
        leftMotorGroup.setVoltage(leftVolts);        
        rightMotorGroup.setVoltage(rightVolts);        
        m_drive.feed();
        odometry.update(Rotation2d.fromDegrees(gyro.getYaw()), ltMotor.getSensorCollection().getIntegratedSensorPosition(), rtMotor.getSensorCollection().getIntegratedSensorPosition());
    } 

    // currently in encoder counts/ 100ms
    public void tankSpeedVelocity(double left, double right) {
        rVolts = right;
        // left = 10.0 * (left * 60 )/ (Constants.DrivetrainConstants.gearRatio * Constants.DrivetrainConstants.encoderTicksPerRev);
        // right = 10.0 * (left * 60)/ (Constants.DrivetrainConstants.gearRatio * Constants.DrivetrainConstants.encoderTicksPerRev);
        lmMotor.set(ControlMode.Velocity, left / 600 *Constants.DrivetrainConstants.encoderTicksPerRev);
        ltMotor.set(ControlMode.Velocity, left / 600 *Constants.DrivetrainConstants.encoderTicksPerRev);
        lbMotor.set(ControlMode.Velocity, left / 600 *Constants.DrivetrainConstants.encoderTicksPerRev);
        rmMotor.set(ControlMode.Velocity, right / 600 *Constants.DrivetrainConstants.encoderTicksPerRev);
        rbMotor.set(ControlMode.Velocity, right / 600 *Constants.DrivetrainConstants.encoderTicksPerRev);
        rtMotor.set(ControlMode.Velocity, right / 600 *Constants.DrivetrainConstants.encoderTicksPerRev);   
        System.out.println("driving");
        mOV = rtMotor.getMotorOutputVoltage();

        odometry.update(Rotation2d.fromDegrees(gyro.getYaw()), ltMotor.getSensorCollection().getIntegratedSensorPosition(), rtMotor.getSensorCollection().getIntegratedSensorPosition());
    }

    /**
     * Resets the drive encoders to currently read a position of 0
     */
    public void resetEncoders() {
        ltMotor.setSelectedSensorPosition(0.0);
        lmMotor.setSelectedSensorPosition(0.0);
        lbMotor.setSelectedSensorPosition(0.0);
        rtMotor.setSelectedSensorPosition(0.0);
        rmMotor.setSelectedSensorPosition(0.0);
        rbMotor.setSelectedSensorPosition(0.0);

        // leftEncoder.reset();
        // rightEncoder.reset();
    }

    /**
     * Gets the average distance of the two encoders
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return ltMotor.getSensorCollection().getIntegratedSensorPosition()/2048 *( .5 * Math.PI);
    }

    /**
     * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    /* public void setMaxOutput(double maxOutput) {
        diffDrive.setMaxOutput(maxOutput);
    } */

    /**
     * Zeroes the heading of the robot
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyro.getAngle();
    }

    /**
     * Returns the turn rate of the robot
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() ;
    }
}