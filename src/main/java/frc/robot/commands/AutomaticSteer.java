package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterHorizontal;

public class AutomaticSteer extends CommandBase {
    private final ShooterHorizontal m_subsystem;
    private final Joystick m_controller;

    public AutomaticSteer(ShooterHorizontal subsystem, Joystick controller, double distanceToTarget) {
        m_subsystem = subsystem;
        m_controller = controller;

        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("auto init");
    }

    @Override
    public void execute() {

    }

    @Override 
    public boolean isFinished() {
        return true;
    }
}