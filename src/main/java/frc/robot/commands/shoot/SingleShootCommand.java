package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;

//basic command for using shooter. TODO: test power to distance and fine tune.
public class SingleShootCommand extends CommandBase{
    private final ShooterSubsystem m_subsystem;
    //private final Intake m_intake = Intake.getInstance();

    private final Timer m_timer = new Timer();

    private double shooterPower;
    private final double m_durationPowerOn = 2;
    //replace this value later and put in constants

    public double shooterPowerCalc(double distance){
        return distance*Constants.ShooterConstants.distanceToVelocity + 5.930;
    }

    public SingleShootCommand(double distance){
        m_subsystem = ShooterSubsystem.getInstance();
        addRequirements(m_subsystem);
        shooterPower = shooterPowerCalc(distance);
        
    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
        m_subsystem.setMotorPowerVelocity(shooterPower);
    }

    @Override
    public void execute(){
        /*
        if(getIsBallReady){
            
            }
        }
        m_intake needs to be a boolean obtained from the intake subsystem.
        wait about 1.5 seconds for two ball, for one ball 1 second should be enough.
        */
    }

    @Override
    public void end(boolean interrupted){
        m_timer.stop();
        //Set percent to low constant power TBD later
        m_subsystem.setMotorPowerPercent(0);
    }

    @Override
    public boolean isFinished(){
        return m_timer.get() >= m_durationPowerOn;
    }
}