package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.IndexerPushCommand;
import frc.robot.commands.StartIntakeCommand;

//this command turns shooter, intake, and indexer on at the same time. Should only be for testing of shooter and other subsystems.
public class IntakeToShooterCommand extends ParallelCommandGroup{
    public IntakeToShooterCommand(double distance){
        addCommands(
            new StartIntakeCommand(),
            new SingleShootCommand(),
            new IndexerPushCommand()
        );
    }
}
