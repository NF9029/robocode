package frc.robot.subsystems;

import static frc.robot.Constants.HorizontalConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.mpu6050.MPU6050;

/**
 * Did this because i want to see if i can move the shooter horizontally and vertically at the same time.
 */
public class ShooterHorizontal extends SubsystemBase {
    private final Spark m_spark = new Spark(MOTOR_PORT);
    private final DigitalInput m_switch = new DigitalInput(DIGITAL_PORT);

    // CHANGE
    private final MPU6050 m_mpu6050 = new MPU6050(I2C_ADDRESS);

    public ShooterHorizontal() {
        m_mpu6050.reset();
        
    }

    
    public void steer(double speed) {

    }

    public void setSpeed() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    public void configure() {}
}
