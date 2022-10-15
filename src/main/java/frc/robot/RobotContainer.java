// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ROBOT_CONTROLLER_PORT;
import static frc.robot.Constants.SHOOTER_CONTROLLER_PORT;
import static frc.robot.Constants.HorizontalConstants.DISTANCE_TO_TARGET;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CloseHatch;
//import edu.wpi.first.wpilibj2.command.Subsystem;
//import frc.robot.commands.Auto;
import frc.robot.commands.Drive;
import frc.robot.commands.Intake;
import frc.robot.commands.Lift;
import frc.robot.commands.OpenHatch;
import frc.robot.commands.HorizontalManualSteer;
import frc.robot.commands.HorizontalAutoSteer;
import frc.robot.commands.HoodManualSteer;
import frc.robot.commands.HoodAutoSteer;
import frc.robot.commands.Shooting;
import frc.robot.subsystems.BallLifter;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CollectorHatch;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterHorizontal;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Joysticks
  public final XboxController m_robotController = new XboxController(ROBOT_CONTROLLER_PORT);
  public final Joystick m_shooterController = new Joystick(SHOOTER_CONTROLLER_PORT);

  // Button
  //public final JoystickButton m_1 = new JoystickButton(m_shooterController, 1);
  public final JoystickButton m_openHatchButton = new JoystickButton(m_robotController, 4);
  public final JoystickButton m_closeHatchButton = new JoystickButton(m_robotController, 1);

  public final JoystickButton m_shooterButton = new JoystickButton(m_shooterController, 1);
  public final JoystickButton m_intakeButton = new JoystickButton(m_shooterController, 3);
  public final JoystickButton m_liftButton = new JoystickButton(m_shooterController, 4);

  // Subsystems
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final ShooterHorizontal m_shooterRotation = new ShooterHorizontal();
  private final ShooterHood m_hood = new ShooterHood();
  private final CollectorHatch m_hatch = new CollectorHatch();
  private final Shooter m_shooter = new Shooter();
  private final BallLifter m_lifter = new BallLifter();
  private final Collector m_collector = new Collector();

  
  // Drivetrain Commands
  private final Drive m_driveCommand = new Drive(m_driveTrain, m_robotController);
  
  // Shooter Commands
  private final HorizontalManualSteer m_horizontalManualSteer = new HorizontalManualSteer(m_shooterRotation, m_shooterController);
  private final HorizontalAutoSteer m_horizontalAutoSteer = new HorizontalAutoSteer(m_shooterRotation);

  private final HoodManualSteer m_hoodManualSteer = new HoodManualSteer(m_hood, m_shooterController);
  private final HoodAutoSteer m_hoodAutoSteer = new HoodAutoSteer(m_hood);

  private final Shooting m_shoot = new Shooting(m_shooter);

  // Intake Commands
  private final OpenHatch m_openHatch = new OpenHatch(m_hatch);
  private final CloseHatch m_closeHatch = new CloseHatch(m_hatch);
  
  private final Intake m_intake = new Intake(m_collector);
  
  private final Lift m_liftBall = new Lift(m_lifter);


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
    m_openHatchButton.debounce(0.05).whenActive(m_openHatch);
    m_closeHatchButton.debounce(0.05).whenActive(m_closeHatch);

    m_intakeButton.whenHeld(m_intake);
    m_liftButton.whenHeld(m_liftBall);
    
    m_shooterButton.whenPressed(m_shoot);
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
      getHorizontalSteerCommand().schedule();
      getHoodSteerCommand().schedule();
  }
  
  public boolean isHorizontalManual() {
    if (-0.40 < m_shooterController.getZ() && m_shooterController.getZ() < 0.40) {
      return false;
    }
    return true;
  }

  public boolean isHoodManual() {
    return true;
  }
  
  public Command getHoodSteerCommand() {
    if (isHoodManual()) {
      return m_horizontalManualSteer;
    } 
    return m_horizontalAutoSteer;
  }

  public Command getHorizontalSteerCommand() {
    if (isHorizontalManual()) {
      // en son otomatik çalıştırılmışsa ve 
      // otomatik bitmemişse bitmesine izin ver
      //if (m_shooterLastAuto && !m_autoSteer.isFinished()) {
      //  m_shooterLastAuto = true;
      //  return m_autoSteer;
      //}
      // otomatik modu işini bitirmişse manuele al
      //m_shooterLastAuto = false;
      return m_horizontalManualSteer;
    } 
    //m_shooterLastAuto = true;
    return m_horizontalAutoSteer;
  }

  public void ScheduleTeleopPeriodic() {
  }
  
  // TEST MODE
  public void initTest() {
    m_hood.initTest();
    //m_driveTrain.resetEncoders();

    m_shooter.m_motorController.set(0.5);
    m_hood.m_motorController.set(0.2); // positive is up (ters bağla)
    m_shooterRotation.m_motorController.setVoltage(12); // POSITIVE VOLT GOES RIGHT
    m_lifter.m_victor.setVoltage(4);
    m_collector.m_motorController.setVoltage(12);
    m_driveTrain.m_leftMotors.setVoltage(4);
    m_driveTrain.m_rightMotors.setVoltage(4);


    //m_driveCommand.test();
    //m_hoodManualSteer.test();
    //m_horizontalManualSteer.test();
    //m_horizontalAutoSteer.test();
    //m_intake.test();
    //m_liftBall.test();
    //m_shoot.test();
  }

  public void testSensors() {
    System.out.print("Encoder data: ");
    m_driveTrain.printEncoderData();
    //new WaitCommand(2);


    // System.out.print("MPU6050 data: ");
    // driveTrain.m_mpu6050.printAllData();
    // driveTrain.m_mpu6050.printGyroData();
    // driveTrain.m_mpu6050.printAccData();
    // driveTrain.m_mpu6050.printTemperature();

    // m_driveTrain.m_mpu6050.printAngles();
  }

  public double getBaseGyroData() {
    return m_driveTrain.m_mpu6050.getAngleX();
  }

  public double getShooterGyroData() {
    return m_shooterRotation.getGyroAngle();
  }

  public void testEncoder() {
    m_hood.printEncoderData();
  }
  
}
