package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallLifter extends SubsystemBase {
    private final Spark m_spark = new Spark(0);
    public void configure() {}
}
