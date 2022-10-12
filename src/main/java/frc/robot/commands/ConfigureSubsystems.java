package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ConfigureSubsystems extends InstantCommand {
    DriveTrain m_drivetrain;
    Shooter m_shooter;
    ShooterHood m_shooterhood;
    ShooterHorizontal m_shooterhorizontal;
    Collector m_collector;
    BallLifter m_balllifter;

    public ConfigureSubsystems(DriveTrain p_drivetrain, 
    Shooter p_shooter, 
    ShooterHood p_shooterhood, 
    ShooterHorizontal p_shooterhorizontal, 
    Collector p_collector, 
    BallLifter p_balllifter) {
        m_drivetrain = p_drivetrain;
        m_shooter = p_shooter;
        m_shooterhood = p_shooterhood;
        m_shooterhorizontal = p_shooterhorizontal;
        m_collector = p_collector;
        m_balllifter = p_balllifter;
    }

    @Override
    public void initialize() {
        m_drivetrain.configure();
        m_shooter.configure();
        m_shooterhood.configure();
        m_shooterhorizontal.configure();
        m_collector.configure();
        m_balllifter.configure();
    }
}
