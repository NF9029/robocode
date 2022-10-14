package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterHood;

public class HoodSteer extends CommandBase {
    ShooterHood m_hood;
    public HoodSteer(ShooterHood hood) {
        m_hood = hood;
        addRequirements(hood);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}