package frc.robot.commands;

//import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class Shooting extends CommandBase {
    private final Shooter m_shooter;

    public Shooting(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_shooter.shoot();
    }
}
