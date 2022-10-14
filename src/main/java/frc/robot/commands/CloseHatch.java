package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollectorHatch;

public class CloseHatch extends CommandBase {
    private final CollectorHatch m_hatch; 
    private final DoubleSolenoid m_solenoidLeft;
    private final DoubleSolenoid m_solenoidRight;

    public CloseHatch(CollectorHatch hatch) {
        m_hatch = hatch;
        m_solenoidLeft = m_hatch.m_solenoid1;
        m_solenoidRight = m_hatch.m_solenoid2;

        addRequirements(m_hatch);
    }

    @Override
    public void initialize() {
        m_solenoidLeft.set(kReverse);
        m_solenoidRight.set(kReverse);
    }

    @Override
    public void execute() {
        System.out.println("closing hatch");
    }
}
