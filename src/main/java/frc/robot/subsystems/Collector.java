package frc.robot.subsystems;

import static frc.robot.Constants.CollectorConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Collector extends SubsystemBase {
    private final WPI_VictorSPX m_motorController = new WPI_VictorSPX(MOTOR_PORT);

    public Collector() {

    }

    public void configure() {}

}
