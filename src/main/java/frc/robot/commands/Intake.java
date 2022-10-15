package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class Intake extends CommandBase {
    private final Collector m_collector; 
    public Intake(Collector collector) {
        m_collector = collector;

        addRequirements(m_collector);
    
    }

    @Override
    public void execute() {
        m_collector.setSpeed();
        System.out.println(m_collector.getSpeed());
    }

    @Override
    public void end(boolean interrupted) {
        m_collector.stop();
    }

    public void test() {
        m_collector.test();
    }
}
