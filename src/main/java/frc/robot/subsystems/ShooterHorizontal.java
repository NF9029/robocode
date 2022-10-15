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

    // Default olarak 0 derece
    public void autoFollow() {
        autoFollow(0);
    }

    public void autoFollow(double targetAngle) {
        // Debug icin
        // m_mpu6050.printAngles();
        
        
        // gyro ile aramızda olan açıyla orantılı olarak motora güç ver
        double angle = m_mpu6050.getAngleX();
        setSpeed(
            m_mpu6050._map(angle-targetAngle, 0, 360, 0, MAX_SPEED),
            angle
        );
    }

    public void steer(double speed) {
        
    }

    public void setSpeed(double speed) {
        setSpeed(speed, m_mpu6050.getAngleX());
    }

    public void setSpeed(double speed, double currentAngle) {
        // eğer fonksiyon çağırılmadan önce yakın bir zamanda açı 
        // ölçülmüşse tekrar okumakla uğraşmamak için angle parametresi var

        // Açı çok geldiyse
        if (currentAngle >= MAX_ANGLE-ANGLE_TOLERANCE) {
            // shooterı son hızda geri döndür

        }

        // Açı az geldiyse
        else if (ANGLE_TOLERANCE <= currentAngle) {
            // shooterı son hızda geri döndür

        }
        
        // sorun yoksa 
        else {
            m_spark.set(speed);
        }
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    public void configure() {

    }
}
