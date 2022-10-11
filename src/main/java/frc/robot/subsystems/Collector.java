package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Collector extends SubsystemBase {
    private final MotorController collectorController = new Spark(0);

    public Collector() {
        // just because i didnt want to see '1 problem in this file'
        collectorController.notify();
    }

    public void configure() {}

}
