package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Did this because i want to see if i can move the shooter horizontally and vertically at the same time.
 */
public class ShooterHorizontal extends SubsystemBase {
    private final Encoder m_encoder = new Encoder(4, 5); 

    public ShooterHorizontal() {
        m_encoder.setDistancePerPulse(360./400.);
    }

    public void initTest() {
        m_encoder.reset();
    }
    
    public void configure() {}

    public void printEncoderData() {
        System.out.println(m_encoder.getDistance());
    }
}
