package frc.robot.commands;

//import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class Shooting extends CommandBase {
    private final Shooter m_shooter;
    private boolean m_isFinished = false;

    public Shooting(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.toggleShoot();
        m_isFinished = true;
        System.out.println("shooter is active: " + m_shooter.isActive());
    }

    @Override
    public boolean isFinished() {
        if (m_isFinished == false) {
            return false;
        }
        m_isFinished = false;
        return true;
    }
}
