package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHorizontal;

public class ShooterSteerGroup extends SequentialCommandGroup {
    public ShooterSteerGroup(Joystick controller, ShooterHorizontal shooter, double DISTANCE_TO_TARGET) {
        addCommands(new AutomaticSteer(shooter, controller, DISTANCE_TO_TARGET), new ShooterSteer(shooter, controller));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
