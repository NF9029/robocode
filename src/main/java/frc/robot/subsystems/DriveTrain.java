// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveTrain extends SubsystemBase {
  /* Differential drive */
  private static final double kTrackWidth = 0.381 * 2; // meters
  
  private final WPI_VictorSPX leftMaster = new WPI_VictorSPX(1);
  private final WPI_VictorSPX leftFollower = new WPI_VictorSPX(2);

  private final WPI_VictorSPX rightMaster = new WPI_VictorSPX(3);
  private final WPI_VictorSPX rightFollower = new WPI_VictorSPX(4);
  
  private final MotorControllerGroup leftMotors;
  private final MotorControllerGroup rightMotors;
  
  private final DifferentialDrive mDrive;

  // No idea what our encoders are
  private final Encoder leftEncoder = new Encoder(0, 1); 
  private final Encoder rightEncoder = new Encoder(2, 3); 
  
  private final DifferentialDriveKinematics m_kinematics =
  new DifferentialDriveKinematics(kTrackWidth);
  
  /**
  * Constructs an object.
  * Resets encoders, inverts a side of motor controllers, etc.
  */
  public DriveTrain() {
    // All of this is gonna have to be edited but, at least some structure?
    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    rightMotors = new MotorControllerGroup(rightMaster, rightFollower);
    leftMotors = new MotorControllerGroup(leftMaster, leftFollower);

    mDrive = new DifferentialDrive(leftMotors, rightMotors);

    rightMotors.setInverted(true);
    
    leftEncoder.setDistancePerPulse(360./400.);
    rightEncoder.setDistancePerPulse(360./400.);

    leftEncoder.reset();
    rightEncoder.reset();

    mDrive.isAlive();
  }
  /**
  * Sets the desired wheel speeds.
  *
  * @param speeds The desired wheel speeds.
  */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    leftMotors.setVoltage(0.0);
    rightMotors.setVoltage(0.0);
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

  public void printEncoderData() {
    System.out.println(leftEncoder.getDistance());
  }
}
