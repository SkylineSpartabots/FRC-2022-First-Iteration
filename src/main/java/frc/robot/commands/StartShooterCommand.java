package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

//turns on the intake. TODO: figure out optimal/prefered intake speed.
public class StartShooterCommand extends CommandBase{
    private final ShooterSubsystem m_subsystem;

    public StartShooterCommand(){
        m_subsystem = ShooterSubsystem.getInstance();
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize(){
        m_subsystem.setShooterPercentPower(0.5);
    }

    @Override
    public void end(boolean interrupted){
        m_subsystem.setShooterPercentPower(0.0);
    }
}
