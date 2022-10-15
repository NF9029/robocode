package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterHood;

public class HoodAutoSteer extends CommandBase {
    private final ShooterHood m_subsystem;

    public HoodAutoSteer(ShooterHood hood) {
        m_subsystem = hood;

        addRequirements(hood);
    }

    @Override
    public void execute() {
        m_subsystem.autoSteer();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}