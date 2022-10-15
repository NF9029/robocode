package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterHorizontal;

import static frc.robot.Constants.AutonomusConstants.*;

public class Auto extends CommandBase {
    private final DriveTrain m_driveBase;

    private final HorizontalAutoSteer m_horizontalSteerCommand;
    private final HoodAutoSteer m_hoodSteerCommand;
    private final Shooting m_shootCommand;

    private final Timer m_timer = new Timer();

    public Auto(DriveTrain drivetrain, Shooter shooter, ShooterHood shooterHood, ShooterHorizontal shooterHorizontal, Joystick controller) {
        m_driveBase = drivetrain;
        m_shootCommand = new Shooting(shooter);
        m_hoodSteerCommand = new HoodAutoSteer(shooterHood);
        m_horizontalSteerCommand = new HorizontalAutoSteer(shooterHorizontal);
    }

    public void shoot() {
        m_horizontalSteerCommand.schedule();
        m_hoodSteerCommand.schedule();
        m_shootCommand.schedule();
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        if (m_timer.get() <= 5) {
            m_driveBase.drive(SPEED, ROTATION);
        }
        shoot();
    }
}
