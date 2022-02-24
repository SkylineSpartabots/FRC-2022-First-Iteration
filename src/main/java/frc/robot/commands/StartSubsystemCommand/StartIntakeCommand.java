package frc.robot.commands.StartSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class StartIntakeCommand extends CommandBase {
    private final IntakeSubsystem m_subsystem;

    public StartIntakeCommand() {
        m_subsystem = IntakeSubsystem.getInstance();
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.setIntakePercentPower(0.8);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setIntakePercentPower(0.0);
    }
}
