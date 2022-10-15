package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    // JOYSTICK BUTTON 1
    private final Talon m_motorController = new Talon(MOTOR_PORT); 
    private boolean m_isActive = false;

    public Shooter() {
        
    }

    public void toggleShoot() {
        if (m_isActive) {
            stop();
            m_isActive = false;
        }
        else {
            shoot();
            m_isActive = true;
        }
    }

    public void shoot() {
        m_motorController.set(MOTOR_SPEED);
    }

    public void stop() {
        m_motorController.set(0);
    }

    public boolean isActive() {
        return m_isActive;
    }
    
    public void configure() {}
}