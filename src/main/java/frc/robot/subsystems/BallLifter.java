package frc.robot.subsystems;

import static frc.robot.Constants.BallLifterConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallLifter extends SubsystemBase {
    public final WPI_VictorSPX m_victor = new WPI_VictorSPX(MOTOR_PORT);
    public BallLifter() {

    }
    public void lift() {
        m_victor.setVoltage(VOLTAGE);
    }

    public void stop() {
        m_victor.setVoltage(0);
    }

    public double getSpeed() {
        return m_victor.getMotorOutputVoltage();
    }

    public void test(double test) {
        m_victor.setVoltage(test);
    }

    public void configure() {}
}
