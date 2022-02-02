package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.ClimbControlState;
import edu.wpi.first.wpilibj.Timer;

public class ClimbCommand extends CommandBase {
    private static final double CLIMB_TIMEOUT = 5.0;
    private final ClimbSubsystem m_subsystem; 
    private final Timer m_timer = new Timer();
    private final ClimbControlState m_state;
    public ClimbCommand(ClimbSubsystem subsystem, ClimbControlState state){
        m_subsystem = subsystem;
        m_state = state;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }
    @Override
    public void execute(){
        boolean interrupted = m_subsystem.setState(m_state);
        if(interrupted)
            end(false);
    }
    @Override
    public void end(boolean interrupted){
        m_subsystem.setState(ClimbControlState.OFF);
    }
    @Override
    public boolean isFinished(){
        return m_timer.hasElapsed(CLIMB_TIMEOUT);
    }
}