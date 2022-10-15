package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterHood;

public class HoodAutoSteer extends CommandBase {
    private final ShooterHood m_subsystem;

    public HoodAutoSteer(ShooterHood subsystem) {
        m_subsystem = subsystem;

        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        m_subsystem.autoSteer();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}