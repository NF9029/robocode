// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.team9029.wpilibj9029.MPU6050;

import static frc.robot.Constants.DriveTrainConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
//import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
//import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
//import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;



public class DriveTrain extends SubsystemBase {
  //private static final byte kRegisterAddress = 0x3B;
  //private static final double kGsPerLSB = 0.00390625;
  
  private final WPI_VictorSPX m_leftMaster = new WPI_VictorSPX(MOTOR_PORT1);
  private final WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(MOTOR_PORT2);
  
  private final WPI_VictorSPX m_rightMaster = new WPI_VictorSPX(MOTOR_PORT3);
  private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(MOTOR_PORT4);
  
  public final MotorControllerGroup m_leftMotors;
  public final MotorControllerGroup m_rightMotors;
  
  //private final DifferentialDrive m_drive;
  
  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
  
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
  
  //private final AnalogGyro m_gyro = new AnalogGyro(0);
  private final DifferentialDriveOdometry m_odometry;
  
  // Encoders
  private final Encoder m_leftEncoder = new Encoder(LEFT_ENCODER_PORT_A, LEFT_ENCODER_PORT_B, LEFT_ENCODER_REVERSED, Encoder.EncodingType.k4X); 
  private final Encoder m_rightEncoder = new Encoder(RIGHT_ENCODER_PORT_A, RIGHT_ENCODER_PORT_B, RIGHT_ENCODER_REVERSED, Encoder.EncodingType.k4X); 
  
  private final DifferentialDriveKinematics m_kinematics = 
  new DifferentialDriveKinematics(TRACK_WIDTH);

  // Gyro 
  public MPU6050 m_mpu6050 = new MPU6050(I2C_ADDRESS);

  /**
  * Constructs an object.
  * Resets encoders, inverts a side of motor controllers, etc.
  */
  public DriveTrain() {
    m_mpu6050.reset();
    // All of this is gonna have to be edited but, at least some structure?
    m_leftFollower.follow(m_leftMaster);
    
    m_leftMotors = new MotorControllerGroup(m_leftMaster, m_leftFollower);
    m_rightMotors = new MotorControllerGroup(m_rightMaster, m_rightFollower);

    
    //m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
    
    m_rightMotors.setInverted(true);
    
    m_leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    m_rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    
    m_odometry = new DifferentialDriveOdometry(m_mpu6050.getRotationX());
  }
  /**
  * Sets the desired wheel speeds.
  *
  * @param speeds The desired wheel speeds.
  */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedfoward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedfoward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    
    final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    
    m_leftMotors.setVoltage(leftOutput + leftFeedfoward);
    m_rightMotors.setVoltage(rightOutput + rightFeedfoward);
  };
  
  /**
  * Drives the robot with the given linear velocity and angular velocity.
  *
  * @param xSpeed Linear velocity in m/s.
  * @param rot Angular velocity in rad/s.
  */
  public void drive(double speed, double rotation) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0.0 , rotation));
    setSpeeds(wheelSpeeds);
  }

  public void fix(double c_center) {
    double step = c_center - 160;
    double error = step / 160;
    System.out.println(error);
    m_leftMotors.set(error);
    m_rightMotors.set(-error);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // configure defaults
  public void configure() {}

  /**
   * Reset's the encoders on the drive train.
   * Called from drive command.
   */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
  
  public void printEncoderData() {
    System.out.println("rate: " + m_leftEncoder.getRate() + 
    ", Distance: " + m_leftEncoder.getDistance() + 
    ", DPP: " + m_leftEncoder.getDistancePerPulse() + 
    ", Direction: " + m_leftEncoder.getDirection());
  }
/* 
  public void printAxes() {
    AllAxes data = new AllAxes();
    ByteBuffer rawData = ByteBuffer.allocate(14);
    m_i2c.read(kRegisterAddress, 14, rawData);
    rawData.order(ByteOrder.LITTLE_ENDIAN);
    data.XAxis = rawData.getShort(8);
    data.YAxis = rawData.getShort(10);
    data.ZAxis = rawData.getShort(12);
    System.out.println(data);
  }
*/

  /** Updates the field-relative position. */
  public void updateOdometry() {
    m_odometry.update(
    m_mpu6050.getRotationX(), // rotation in Rotation2d from custom lib. may use Y or Z accordingly
    m_leftEncoder.getDistance(), 
    m_rightEncoder.getDistance());
  }
}
