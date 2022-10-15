package frc.robot.subsystems;

import static frc.robot.Constants.HoodConstants.*;



import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Did this because i want to see if i can move the shooter horizontally and vertically at the same time.
 */
public class ShooterHood extends SubsystemBase {
    public Talon m_motorController = new Talon(MOTOR_PORT);
    private final Encoder m_encoder = new Encoder(ENCODER_PORT_A, ENCODER_PORT_B, false, Encoder.EncodingType.k1X); 

    public ShooterHood() {
        // başlangıçta açıyı sıfırla
        m_encoder.reset();
        m_encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    }

    public void manualSteer(double speed) {
        setSpeed(speed);
    }

    public void autoSteer() {
        // siktir bok
    }

    public void initTest() {
        m_encoder.reset();
    }

    public double getAngle() {
        return m_encoder.getDistance();
    }
    
    public int checkAngle(double targetAngle, double currentAngle) {
        if (Math.abs(targetAngle - currentAngle) <= ANGLE_TOLERANCE) {
            System.out.println("[HOOD] TARGET IS REACHED");
            return 0;
        } else if (targetAngle < currentAngle) {
            return -1;
        // } else if (targetAngle > currentAngle) {
        } else{
            return 1;
        } 
    }

    public boolean setAngle(double targetAngle) {
        return setAngle(targetAngle, getAngle());
    }

    public boolean setAngle(double targetAngle, double currentAngle) {
        return setAngle(targetAngle, currentAngle, MAX_SPEED);
    }

    public boolean setAngle(double targetAngle, double currentAngle, double speed) {
        // hedefe ulaştıysak
        int rv = checkAngle(targetAngle, currentAngle);
        if (rv == 0) {
            System.out.println("[HOOD] TARGET IS REACHED");
            return true;
        } else if (rv == -1) {
            m_motorController.set(speed);
            return false;
        // } else if (rv == 1) {
        } else {
            m_motorController.set(-speed);
            return false;
        } 
    }

    public void setSpeed(double speed) {
        double angle = getAngle();
        // alt sınırlar
        if (angle <= MAX_ANGLE_TOLERANCE) {
            if (angle <= 0) {
                System.out.println("[HOOD] angle already minimum");
            } else {
                m_motorController.set(speed/2);
            }
        }
        // üst sınırlar
        else if (angle >= MAX_ANGLE-MAX_ANGLE_TOLERANCE) {
            if (angle >= MAX_ANGLE) {
                System.out.println("[HOOD] angle already maximum");
            } else {
                m_motorController.set(speed/2);
            }
        }
        // normal
        else {
            m_motorController.set(speed);
        }
    }

    public void configure() {}

    public void printEncoderData() {
        System.out.println(m_encoder.getDistance());
    }
}
