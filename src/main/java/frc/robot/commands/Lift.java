package frc.robot.commands;

import static frc.robot.Constants.BallLifterConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallLifter;

public class Lift extends CommandBase {
    private final BallLifter m_lifter;
    public Lift(BallLifter lifter) {
        m_lifter = lifter;

        addRequirements(m_lifter);
    }

    @Override
    public void execute() {
        m_lifter.lift();
    }

    @Override
    public void end(boolean interrupted) {
        m_lifter.stop();
    }
}
