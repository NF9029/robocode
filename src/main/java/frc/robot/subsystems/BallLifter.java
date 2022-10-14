package frc.robot.subsystems;

import static frc.robot.Constants.BallLifter.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallLifter extends SubsystemBase {
    private final WPI_VictorSPX m_victor = new WPI_VictorSPX(MOTOR_PORT);
    public void configure() {}
}
