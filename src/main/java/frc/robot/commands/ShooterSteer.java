package frc.robot.commands;

import static frc.robot.Constants.HorizontalConstants.*;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterHorizontal;

public class ShooterSteer extends CommandBase {
    private final ShooterHorizontal m_subsystem;
    private final Joystick m_controller;

    private final SlewRateLimiter m_filter = new SlewRateLimiter(FILTER);

    public ShooterSteer(ShooterHorizontal subsystem, Joystick controller) {
        m_subsystem = subsystem;
        m_controller = controller;

        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        /*
        if (m_controller.getZ() > 0.1 && m_controller.getZ() < -0.1) {
            return;
        }
        System.out.println("Manual steering off");
        end(true);
        */
    }

    @Override
    public void execute() {
        System.out.println("Manual steering on");
        final var speed = m_filter.calculate(m_controller.getZ()) * MAX_SPEED;

        m_subsystem.steer(speed);
    }
    
    @Override
    public boolean isFinished() {
        if (-0.1 < m_controller.getZ() && m_controller.getZ() < 0.1) {
            return true;
        }
        System.out.println("Manual steering off");
        return false;
    }
}
