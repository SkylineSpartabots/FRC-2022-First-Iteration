package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

//turns on the intake. TODO: figure out optimal/prefered intake speed.
public class StartIntakeCommand extends CommandBase{
    private final IntakeSubsystem m_subsystem;
    private final Timer m_timer = new Timer();
    private final double m_durationPush = 4;
    private BooleanSupplier buttonPressed;

    public StartIntakeCommand(BooleanSupplier buttonPressed){
        this.buttonPressed = buttonPressed;
        m_subsystem = IntakeSubsystem.getInstance();
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
        m_subsystem.setIntakePowerPercent(Constants.IntakeConstants.intakeSpeedPercent);
    }

    @Override
    public void execute(){
        //implement some sort of encoder in Indexer to make sure it doesn't jam.
    }

    @Override
    public void end(boolean interrupted){
        m_timer.stop();
        m_subsystem.setIntakePowerPercent(0);
    }

    @Override
    public boolean isFinished(){
        return !buttonPressed.getAsBoolean();
    }
}
