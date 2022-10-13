//package frc.robot.commands;
//
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.Subsystem;
//import frc.robot.subsystems.Shooter;
//import frc.robot.subsystems.ShooterHood;
//import frc.robot.subsystems.ShooterHorizontal;

/**
 * This command group will first -in a paralel command group included in this one- aim the shooter steering
 * vertically via the hood and horizontally (at the same time) and then shoot.
 */
/*
public class ShooterCommandGroup extends SequentialCommandGroup{
    private ShooterHorizontal m_shooter_horizontal;
    private Joystick m_joystick;

    public ShooterCommandGroup(ShooterHorizontal horizontal, ShooterHood hood, Shooter shoot) {
        m_shooter_horizontal = horizontal;
        addCommands(new ParallelCommandGroup(new ShooterSteer(m_shooter_horizontal, m_joystick), new HoodSteer(m_shooterHood)), new Shooting(m_shooter));
    }
}
*/