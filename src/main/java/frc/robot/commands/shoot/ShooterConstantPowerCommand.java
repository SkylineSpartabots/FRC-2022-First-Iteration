
package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

//this command should be starting in the beginning of teleop to keep the shooter at a constant power, less tiem to rev up?
public class ShooterConstantPowerCommand extends CommandBase {
    private final ShooterSubsystem m_subsystem;
    //private final Intake m_intake = Intake.getInstance();
    private final double constantPowerPercent = 0.5;

    public ShooterConstantPowerCommand(){
        m_subsystem = ShooterSubsystem.getInstance();
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize(){
        m_subsystem.setMotorPowerPercent(constantPowerPercent);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
        //Set motor percent to low constant power TBD later
        m_subsystem.setMotorPowerPercent(0.0);
    }
}