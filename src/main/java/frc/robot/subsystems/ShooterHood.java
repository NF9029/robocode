package frc.robot.subsystems;

import static frc.robot.Constants.HoodConstants.*;



import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Did this because i want to see if i can move the shooter horizontally and vertically at the same time.
 */
public class ShooterHood extends SubsystemBase {
    private Talon m_motorController = new Talon(MOTOR_PORT);
    private final Encoder m_encoder = new Encoder(ENCODER_PORT_A, ENCODER_PORT_B, false, Encoder.EncodingType.k1X); 

    public ShooterHood() {
        // başlangıçta açıyı sıfırla
        m_encoder.reset();
        m_encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    }

    public void initTest() {
        m_encoder.reset();
    }

    public void setSpeed() {
        // getAngle() kullanarak pid sistemi 
    }

    public double getAngle() {
        return m_encoder.getDistance();
    }
    
    public void configure() {}

    public void printEncoderData() {
        System.out.println(m_encoder.getDistance());
    }
}
