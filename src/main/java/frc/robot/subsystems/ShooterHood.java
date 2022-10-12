package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Did this because i want to see if i can move the shooter horizontally and vertically at the same time.
 */
public class ShooterHood extends SubsystemBase {
    private TalonSRX m_controller = new TalonSRX(0);
    private final Encoder m_encoder = new Encoder(4, 5); 

    public ShooterHood() {
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
