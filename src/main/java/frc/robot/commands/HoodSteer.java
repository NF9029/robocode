package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class HoodSteer extends CommandBase {
    private final Encoder m_encoder = new Encoder(2,3);
    public HoodSteer() {
        m_encoder.reset();
    }
}
