// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.lib.mpu6050.MPU6050;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
//import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class DriveTrain extends SubsystemBase {
  /* Differential drive */
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = Math.PI * 2; // rotation


  private static final double kTrackWidth = 0.605; // meters
  private static final double kWheelRadius = 0.1526; // meters
  
  private static final int kEncoderResolution = 4096;
  //private static final byte kAddress = 0x68;
  //private static final byte kRegisterAddress = 0x3B;
  //private static final double kGsPerLSB = 0.00390625;
  
  private final WPI_VictorSPX m_leftMaster = new WPI_VictorSPX(1);
  private final WPI_VictorSPX m_leftFollower = new WPI_VictorSPX(2);
  
  private final WPI_VictorSPX m_rightMotors = new WPI_VictorSPX(3);
  
  private final MotorControllerGroup m_leftMotors;
  
  private final DifferentialDrive m_drive;
  
  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
  
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
  
  private final AnalogGyro m_gyro = new AnalogGyro(0);
  private final DifferentialDriveOdometry m_odometry;
  
  // No idea what our encoders are
  private final Encoder m_leftEncoder = new Encoder(0, 1); 
  private final Encoder m_rightEncoder = new Encoder(2, 3); 
  
  private final DifferentialDriveKinematics m_kinematics = 
  new DifferentialDriveKinematics(kTrackWidth);

  // Gyro 
  public MPU6050 m_mpu6050 = new MPU6050((byte) 0x68);

  /**
  * Constructs an object.
  * Resets encoders, inverts a side of motor controllers, etc.
  */
  public DriveTrain() {
    m_gyro.reset();
    // All of this is gonna have to be edited but, at least some structure?
    m_leftFollower.follow(m_leftMaster);
    
    m_leftMotors = new MotorControllerGroup(m_leftMaster, m_leftFollower);
    
    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
    
    m_rightMotors.setInverted(true);
    
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
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
  
  public void printEncoderData() {
    System.out.println(m_leftEncoder.getDistance());
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
    m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }
}
