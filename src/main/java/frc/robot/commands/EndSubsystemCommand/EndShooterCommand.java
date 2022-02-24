package frc.robot.commands.EndSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class EndShooterCommand extends CommandBase {
    private final ShooterSubsystem m_subsystem;

    public EndShooterCommand() {
        m_subsystem = ShooterSubsystem.getInstance();
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.setShooterPercentPower(0.0);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}
