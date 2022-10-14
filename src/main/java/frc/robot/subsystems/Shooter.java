package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    // JOYSTICK BUTTON 1
    private final Talon m_motorController = new Talon(MOTOR_PORT); 

    public Shooter() {
        
    }

    public void shoot() {
        m_motorController.set(MOTOR_SPEED);
    }
    public void configure() {}
}