package frc.robot.subsystems;

import static frc.robot.Constants.CollectorConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Collector extends SubsystemBase {
    public final WPI_VictorSPX m_motorController = new WPI_VictorSPX(MOTOR_PORT);

    public Collector() {

    }

    public void setSpeed() {
        m_motorController.setVoltage(VOLTAGE);
    }

    public void stop() {
        m_motorController.setVoltage(0);
    }

    public double getSpeed() {
        return m_motorController.getMotorOutputVoltage();
    }

    public void test() {
        m_motorController.setVoltage(TEST_VOLT);
    }

    public void configure() {}

}
