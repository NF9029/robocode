package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterHorizontal;


public class Auto extends CommandBase {
    private final DriveTrain m_driveBase;

    private final Shooting m_shoot;
    private final HoodManualSteer m_hood;
    private final HorizontalAutoSteer m_steer;

    private final Timer m_timer = new Timer();

    public Auto(DriveTrain drivetrain, Shooter shooter, ShooterHood hood, ShooterHorizontal steer, Joystick controller, double distanceToTarget) {
        m_driveBase = drivetrain;
        m_shoot = new Shooting(shooter);
        m_hood = new HoodManualSteer(hood);
        m_steer = new HorizontalAutoSteer(steer, controller, distanceToTarget);
    }

    public void shoot() {
        m_hood.schedule();
        m_steer.schedule();
        m_shoot.schedule();
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        m_driveBase.drive(-6, 0);
        if (m_timer.get() == 5) {
            
        }
    }
}
