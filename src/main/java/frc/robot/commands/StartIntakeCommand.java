package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

//turns on the intake. TODO: figure out optimal/prefered intake speed.
public class StartIntakeCommand extends CommandBase{
    private final IntakeSubsystem m_subsystem;
    private final Timer m_timer = new Timer();
    private final double m_durationPush = 4;

    public StartIntakeCommand(){
        m_subsystem = IntakeSubsystem.getInstance();
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
        m_subsystem.setIntakePowerPercent(Constants.IndexerConstants.indexerSpeedPercent);
    }

    @Override
    public void execute(){
        //implement some sort of encoder in Indexer to make sure it doesn't jam.
    }

    @Override
    public void end(boolean interrupted){
        m_timer.stop();
    }

    @Override
    public boolean isFinished(){
        return m_timer.get() >= m_durationPush;
    }
}
