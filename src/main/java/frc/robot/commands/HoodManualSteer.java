package frc.robot.commands;

import static frc.robot.Constants.HoodConstants.*;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterHood;

public class HoodManualSteer extends CommandBase {
    private final ShooterHood m_subsystem;
    private final Joystick m_controller;

    private final SlewRateLimiter m_filter = new SlewRateLimiter(FILTER);

    public HoodManualSteer(ShooterHood subsystem, Joystick controller) {
        m_subsystem = subsystem;
        m_controller = controller;

        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        final var speed = m_filter.calculate(m_controller.getZ()) * MAX_SPEED;
        m_subsystem.manualSteer(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public void test() {
        final var speed = m_filter.calculate(m_controller.getZ() * TEST_SPEED);
        m_subsystem.manualSteer(speed);
    }
}