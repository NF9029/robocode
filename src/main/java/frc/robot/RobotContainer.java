// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.HorizontalConstants.DISTANCE_TO_TARGET;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutomaticSteer;
import frc.robot.commands.CloseHatch;
//import edu.wpi.first.wpilibj2.command.Subsystem;
//import frc.robot.commands.Auto;
import frc.robot.commands.Drive;
import frc.robot.commands.Intake;
import frc.robot.commands.Lift;
import frc.robot.commands.OpenHatch;
import frc.robot.commands.ShooterSteer;
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
  private final ShooterSteer m_manualSteer = new ShooterSteer(m_shooterRotation, m_shooterController);
  private final AutomaticSteer m_autoSteer = new AutomaticSteer(m_shooterRotation, m_shooterController, DISTANCE_TO_TARGET);

  private final Lift m_liftBall = new Lift();

  // Intake Commands
  private final OpenHatch m_openHatch = new OpenHatch(m_hatch);
  private final CloseHatch m_closeHatch = new CloseHatch(m_hatch);
  private final Intake m_intake = new Intake();
  

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
      getShooterSteerCommand().schedule();
  }
  
  public boolean isManual() {
    if (-0.40 < m_shooterController.getZ() && m_shooterController.getZ() < 0.40) {
      return false;
    }
    return true;
  }
  
  public Command getShooterSteerCommand() {
    if (isManual()) {
      // en son otomatik çalıştırılmışsa ve 
      // otomatik bitmemişse bitmesine izin ver
      //if (m_shooterLastAuto && !m_autoSteer.isFinished()) {
      //  m_shooterLastAuto = true;
      //  return m_autoSteer;
      //}
      // otomatik modu işini bitirmişse manuele al
      //m_shooterLastAuto = false;
      return m_manualSteer;
    } 
    //m_shooterLastAuto = true;
    return m_autoSteer;
  }

  public void ScheduleTeleopPeriodic() {
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
