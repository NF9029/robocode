package frc.robot.subsystems;

import static frc.robot.Constants.CollectorConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CollectorHatch extends SubsystemBase {
    public final DoubleSolenoid m_solenoid1 = new DoubleSolenoid(P_MODULE_TYPE, PORT_1, PORT_2);
    public final DoubleSolenoid m_solenoid2 = new DoubleSolenoid(P_MODULE_TYPE, PORT_3, PORT_4);
    
    public CollectorHatch() {

    }
}
