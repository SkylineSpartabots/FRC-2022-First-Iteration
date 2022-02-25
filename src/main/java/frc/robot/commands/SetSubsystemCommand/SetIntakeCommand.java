package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeCommand extends CommandBase {
    private final IntakeSubsystem m_subsystem;
    private double percentPower;

    public SetIntakeCommand(double percentPower) {
        m_subsystem = IntakeSubsystem.getInstance();
        addRequirements(m_subsystem);
        this.percentPower = percentPower;
    }

    @Override
    public void initialize() {
        m_subsystem.setIntakePercentPower(percentPower);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}
