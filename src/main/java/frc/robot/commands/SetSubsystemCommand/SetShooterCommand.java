package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterCommand extends CommandBase {
    private final ShooterSubsystem m_subsystem;
    private double percentPower;

    public SetShooterCommand(double percentPower) {
        m_subsystem = ShooterSubsystem.getInstance();
        addRequirements(m_subsystem);
        this.percentPower = percentPower;
    }

    @Override
    public void initialize() {
        m_subsystem.setShooterPercentPower(percentPower);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}
