package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.commands.IndexerPushCommand;

public class DoubleShootCommand extends SequentialCommandGroup{

    private final double timeForLaunch = 1.5;
    public DoubleShootCommand(double distance){
        addCommands(
            new SingleShootCommand(),
            //new IndexerPushCommand(),
            new SingleShootCommand()
            //new IndexerPushCommand()
        );/*indexerPush is simply to push the ball into contact with the motor,
        motors will continue moving at velocity speed set by SingleShootCommand(), and will return to normal with ShooterConstantPowerCommand().
        This could run into issues with multiple ShooterConstantPowerCommands layered, will need testing*/
    }
}
