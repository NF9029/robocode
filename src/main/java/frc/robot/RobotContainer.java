// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.commands.Auto;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterHorizontal;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static int portR = 0, portS = 1;

  // Joysticks
  public final XboxController m_robotController = new XboxController(portR);
  public final Joystick m_shooterController = new Joystick(portS);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // Subsystems
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final ShooterHorizontal m_shooterRotation = new ShooterHorizontal();
  private final ShooterHood m_hood = new ShooterHood();
  
  // Drivetrain Commands
  private final Drive drive = new Drive(m_driveTrain);

  // Shooter Commands

  // Intake Commands

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // m_robotController Buttons: A -> 1, B -> 2, X -> 3, Y -> 4
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An TankDrive will run in autonomous
    return null;
  }

  public void drive() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_speedLimiter.calculate(m_robotController.getLeftY()) * DriveTrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(m_robotController.getRightX()) * DriveTrain.kMaxAngularSpeed;

    m_driveTrain.drive(xSpeed, rot);
  }

  // TEST MODE
  public void initTest() {
    m_hood.initTest();
  }

  public void testSensors() {
    // System.out.print("Encoder data: ");
    // driveTrain.printEncoderData();

    System.out.print("MPU6050 data: ");
    // driveTrain.m_mpu6050.printAllData();
    // driveTrain.m_mpu6050.printGyroData();
    // driveTrain.m_mpu6050.printAccData();
    // driveTrain.m_mpu6050.printTemperature();

    m_driveTrain.m_mpu6050.printAngles();
  }

  public void testEncoder() {
    m_hood.printEncoderData();
  }
  
}
