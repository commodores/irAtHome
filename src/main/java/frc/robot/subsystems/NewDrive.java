// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveManual;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NewDrive extends SubsystemBase {
  private WPI_TalonFX falcon1_leftLead    = new WPI_TalonFX(DriveConstants.kLeftMasterPort);
  private WPI_TalonFX falcon2_leftFollow  = new WPI_TalonFX(DriveConstants.kLeftSlavePort);
  private WPI_TalonFX falcon3_rightLead   = new WPI_TalonFX(DriveConstants.kRightMasterPort);
  private WPI_TalonFX falcon4_rightFollow = new WPI_TalonFX(DriveConstants.kRightSlavePort);

  private PigeonIMU pigeon = new PigeonIMU(DriveConstants.kPigeonPort);
  
  private boolean driveInvert = false;
  
  private final DifferentialDrive m_drive;
  private final SimpleMotorFeedforward  m_feedforward = 
      new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);

  private PIDController left_PIDController = new PIDController(DriveConstants.kPDriveVel, 0.0, DriveConstants.kDDriveVel);
  private PIDController right_PIDController =  new PIDController(DriveConstants.kPDriveVel, 0.0, DriveConstants.kDDriveVel);
 
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  
  public NewDrive() {

    pigeon.configFactoryDefault();

    m_drive = new DifferentialDrive(falcon1_leftLead, falcon3_rightLead);
    falcon1_leftLead.configFactoryDefault();
    falcon2_leftFollow.configFactoryDefault();
    falcon3_rightLead.configFactoryDefault();
    falcon4_rightFollow.configFactoryDefault();

    // set brake mode
    falcon1_leftLead.setNeutralMode(NeutralMode.Brake);
    falcon2_leftFollow.setNeutralMode(NeutralMode.Brake);
    falcon3_rightLead.setNeutralMode(NeutralMode.Brake);
    falcon4_rightFollow.setNeutralMode(NeutralMode.Brake);

    // No need to invert Follow Motors
    falcon1_leftLead.setInverted(true);
    falcon3_rightLead.setInverted(false);
    falcon2_leftFollow.setInverted(InvertType.FollowMaster);
    falcon4_rightFollow.setInverted(InvertType.FollowMaster);

    // set Lead/Follow 
    falcon2_leftFollow.follow(falcon1_leftLead);
    falcon4_rightFollow.follow(falcon3_rightLead);

    
    // default feedback sensor
    falcon1_leftLead.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, DriveConstants.driveTimeout);
    falcon3_rightLead.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, DriveConstants.driveTimeout);

    m_drive.setRightSideInverted(false);

    // TODO: only set open loop ramp AFTER auton, so not to conflict with path follow
    falcon1_leftLead.configOpenloopRamp(.5);
    falcon3_rightLead.configOpenloopRamp(.5);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  }

  @Override
  public void periodic() {
    // Note: periodic() is run by the scheduler, always. No matter what.
    // Update the odometry in the periodic block
    double leftDist = getLeftPosition(); 
    double rightDist = getRightPosition();
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftDist, rightDist);

    // log drive train and data to Smartdashboard
    /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
    SmartDashboard.putNumber("IMU_FusedHeading", pigeon.getFusedHeading());
    // NOTE: call getFusedHeading(FusionStatus) to detect gyro errors

    // report the wheel speed, position, and pose
    SmartDashboard.putNumber("left_wheel_Velocity", getLeftVelocity());
    SmartDashboard.putNumber("right_wheel_Velocity", getRightVelocity());
    SmartDashboard.putNumber("left_wheel_Distance", leftDist); 
    SmartDashboard.putNumber("right_wheel_Distance", rightDist);
    
    Pose2d currentPose = m_odometry.getPoseMeters();
    SmartDashboard.putNumber("pose_x",currentPose.getTranslation().getX());
    SmartDashboard.putNumber("pose_y",currentPose.getTranslation().getY());
    SmartDashboard.putNumber("pose_theta", currentPose.getRotation().getDegrees());
  }

  /**
   * Returns the distance in Meteres the left wheel has travelled
   *
   * @return distance in meters
   */
  double getLeftPosition() {
    // Native units are encoder ticks (2048 ticks per revolution)
   return falcon1_leftLead.getSelectedSensorPosition() * DriveConstants.kDistancePerWheelRevolutionMeters * DriveConstants.kGearReduction / DriveConstants.kEncoderCPR;
 }

 /**
  * Returns the distance in Meteres the right wheel has travelled
  *
  * @return distance in meters
  */
 double getRightPosition() {
   // Native units are encoder ticks (2048 ticks per revolution)
   return falcon3_rightLead.getSelectedSensorPosition() * DriveConstants.kDistancePerWheelRevolutionMeters * DriveConstants.kGearReduction / DriveConstants.kEncoderCPR;
 }

 /**
  * Returns the velocity of the left wheel in meters per second
  *
  * @return velocity in meters/second
  */
 double getLeftVelocity() {
   // Native units are encoder ticks per 100ms
   return falcon1_leftLead.getSelectedSensorVelocity() * DriveConstants.kDistancePerWheelRevolutionMeters * DriveConstants.kGearReduction * 10.0 / DriveConstants.kEncoderCPR ;
 }

 /**
  * Returns the velocity of the right wheel in meters per second
  *
  * @return velocity in meters/second
  */
 double getRightVelocity() {
   // Native units are encoder ticks per 100ms
   return falcon3_rightLead.getSelectedSensorVelocity() * DriveConstants.kDistancePerWheelRevolutionMeters * DriveConstants.kGearReduction * 10.0 / DriveConstants.kEncoderCPR ;
 }

 /**
  * Returns the currently-estimated pose of the robot.
  *
  * @return The pose.
  */
 public Pose2d getPose() {
   return m_odometry.getPoseMeters();
 }

 /**
  * getFuturePose() - predict robot pose t_sec in the future based on the current robot wheel
  * speed.
  * 
  * This is only an estimate and the larger the value t_sec, the less accurate the estimate will
  * be.
  * 
  * @param t_sec
  * @return Estimated pose t_sec in the future.
  */
 public Pose2d getFuturePose(double t_sec) {
   Pose2d current_pose = m_odometry.getPoseMeters();
   if (t_sec <= 0) {
     System.out.println("Error: getFuturePose needs a positive time value.");
     return current_pose;
   }

   // new odometery class starting from current position
   Rotation2d current_heading = Rotation2d.fromDegrees(getHeading());
   DifferentialDriveOdometry future_odometry =
       new DifferentialDriveOdometry(current_heading, current_pose);

   // predict where we will be t_sec in the future based on our current wheel speeds
   future_odometry.update(current_heading, getLeftVelocity() * t_sec,
       getRightVelocity() * t_sec);

   return future_odometry.getPoseMeters();
 }

 /**
  * Returns the Feedforward settings for the drivetrain.
  * 
  * @return Feedforward
  */
 public SimpleMotorFeedforward getFeedforward() {
   return m_feedforward;
 }

 /**
  * Returns the current wheel speeds of the robot.
  *
  * @return The current wheel speeds.
  */
 public DifferentialDriveWheelSpeeds getWheelSpeeds() {
   return new DifferentialDriveWheelSpeeds(
       getLeftVelocity(),
       getRightVelocity());
 }

 /**
  * getChassisSpeeds() - return the robot velocity in meters/second in robot centric X and Y
  * direction, and the rotation of the robot in radians/second.
  * 
  * Note: Vy should always be zero, because the robot cannot drive sideways.
  * 
  * @return ChassisSpeeds: (vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond)
  */
 public ChassisSpeeds getChassisSpeeds() {
   return DriveConstants.kDriveKinematics.toChassisSpeeds(getWheelSpeeds());
 }

 /**
  * Returns the left PIDController object
  *
  * @return PIDController
  */
 public PIDController getLeftPidController() {
   return left_PIDController;
 }

 /**
  * Returns the right PIDController object
  *
  * @return PIDController
  */
 public PIDController getRightPidController() {
   return right_PIDController;
 }

 /**
  * Resets the odometry to the specified pose.
  *
  * @param pose The pose to which to set the odometry.
  */
 public void resetOdometry(Pose2d pose) {
   resetEncoders();
   m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
 }

 /**
  * Drives the robot using arcade controls.
  *
  * @param fwd the commanded forward movement
  * @param rot the commanded rotation
  */
 public void arcadeDrive(double fwd, double rot) {
   // use slew rate filters to implement ramp up/down of speed and rotation
   //m_drive.arcadeDrive(speedFilter.calculate(fwd), rotationFilter.calculate(rot));
   m_drive.arcadeDrive(fwd, rot);
 }

 /**
  * Controls the left and right sides of the drive directly with voltages.
  *
  * @param leftVolts  the commanded left output
  * @param rightVolts the commanded right output
  */
 public void tankDriveVolts(double leftVolts, double rightVolts) {
   falcon1_leftLead.setVoltage(leftVolts);
   falcon3_rightLead.setVoltage(rightVolts);
   m_drive.feed();
 }

  public void curvatureDrive(double speed, double rotation, boolean quickturn){
    m_drive.curvatureDrive(speed, rotation, quickturn);
  }

 /**
  * Resets the drive encoders to currently read a position of 0.
  */
 public void resetEncoders() {
   falcon1_leftLead.setSelectedSensorPosition(0);
   falcon3_rightLead.setSelectedSensorPosition(0);
 }

 /**
  * Gets the average distance of the two encoders.
  *
  * @return the average of the two encoder readings
  */
 public double getAverageEncoderDistance() {
   return ((getLeftPosition() + getRightPosition()) / 2.0);
 }

 /**
  * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
  *
  * @param maxOutput the maximum output to which the drive will be constrained
  */
 public void setMaxOutput(double maxOutput) {
   m_drive.setMaxOutput(maxOutput);
 }

 
 
 /**
  * Zeroes the heading of the robot.
  */
 public void zeroHeading() {
   pigeon.setFusedHeading(0);
 }

 /**
  * Returns the heading of the robot.
  *
  * @return the robot's heading in degrees, from 180 to 180
  */
 public double getHeading() {
   // m_gyro.getFusedHeading() returns degrees
   return Math.IEEEremainder(pigeon.getFusedHeading(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
 }

 /**
  * Returns the turn rate of the robot.
  *
  * @return The turn rate of the robot, in degrees per second
  */
  public double getTurnRate() {
    double [] xyz_dps = new double [3];
    // getRawGyro returns in degrees/second
    pigeon.getRawGyro(xyz_dps);
    return xyz_dps[2] * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

 /**
  * Enable current limiting.
  *
  * @param current limit
  */


 public boolean getDriveInvert() {
   return driveInvert;
 }

 public void setDriveInvert(boolean invert) {
   driveInvert = invert;
 }


}
