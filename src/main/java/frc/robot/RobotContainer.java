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
import edu.wpi.first.wpilibj2.command.Subsystem;
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
  private static int m_portR = 0, m_portS = 1;

  // Joysticks
  public final XboxController m_robotController = new XboxController(m_portR);
  public final Joystick m_shooterController = new Joystick(m_portS);


  // Subsystems
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final ShooterHorizontal m_shooterRotation = new ShooterHorizontal();
  private final ShooterHood m_hood = new ShooterHood();
  
  // Drivetrain Commands
  private final Drive m_driveCommand = new Drive(m_driveTrain, m_robotController);

  // Shooter Commands

  // Intake Commands

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_driveTrain.setDefaultCommand(m_driveCommand);
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

  public void ScheduleTeleop() {
    m_driveTrain.getDefaultCommand().schedule();
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
