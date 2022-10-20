package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Yank extends CommandBase {
    private final DriveTrain m_base;
    private static double m_center;

    /**
     * 
     * @param subsystem
     * @param center center of target as on camera feed
     */
    public Yank(DriveTrain subsystem, double center) {
        m_base = subsystem;
        m_center = center;

        addRequirements(m_base);
    }

    @Override
    public void execute() {
        m_base.fix(m_center);
    }
}
