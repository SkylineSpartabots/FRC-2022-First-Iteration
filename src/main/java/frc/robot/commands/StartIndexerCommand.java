package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

//turns on the intake. TODO: figure out optimal/prefered intake speed.
public class StartIndexerCommand extends CommandBase{
    private final IndexerSubsystem m_subsystem;

    public StartIndexerCommand(){
        m_subsystem = IndexerSubsystem.getInstance();
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize(){
        m_subsystem.setIndexerPercentPower(0.3);
    }

    @Override
    public void end(boolean interrupted){
        m_subsystem.setIndexerPercentPower(0.0);
    }
}
