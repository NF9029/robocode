package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This command group will first -in a paralel command group included in this one- aim the shooter steering
 * vertically via the hood and horizontally (at the same time) and then shoot.
 */
public class ShooterCommandGroup extends SequentialCommandGroup{
    public ShooterCommandGroup() {
        addCommands(new ParallelCommandGroup(new ShooterSteer(), new HoodSteer()), new Shooting());
    }
}
