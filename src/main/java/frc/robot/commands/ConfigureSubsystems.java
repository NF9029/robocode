package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ConfigureSubsystems extends InstantCommand {
    DriveTrain drivetrain;
    Shooter shooter;
    ShooterHood shooterhood;
    ShooterHorizontal shooterhorizontal;
    Collector collector;
    BallLifter balllifter;

    public ConfigureSubsystems(DriveTrain p_drivetrain, 
    Shooter p_shooter, 
    ShooterHood p_shooterhood, 
    ShooterHorizontal p_shooterhorizontal, 
    Collector p_collector, 
    BallLifter p_balllifter) {
        drivetrain = p_drivetrain;
        shooter = p_shooter;
        shooterhood = p_shooterhood;
        shooterhorizontal = p_shooterhorizontal;
        collector = p_collector;
        balllifter = p_balllifter;
    }

    @Override
    public void initialize() {
        drivetrain.configure();
        shooter.configure();
        shooterhood.configure();
        shooterhorizontal.configure();
        collector.configure();
        balllifter.configure();
    }
}
