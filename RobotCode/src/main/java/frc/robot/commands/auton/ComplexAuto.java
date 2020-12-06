package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ComplexAuto extends SequentialCommandGroup {

    public ComplexAuto() {
        addCommands(
                new DriveStraight(6.2, 0.8),
                new WaitCommand(1),
                new DriveStraight(13, -0.6)
        );
    }

}
