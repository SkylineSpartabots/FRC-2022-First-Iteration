package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.commands.StartIntakeCommand;

public class IntakeSubsystem extends SubsystemBase{   

    private final LazyTalonFX m_IntakeMotor;

    //get instance of subsystem    
    private static IntakeSubsystem instance = null;
    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    public IntakeSubsystem() {
        m_IntakeMotor = TalonFXFactory.createDefaultFalcon("Shooter Motor", 31);//creates motor
        //configure motor
        PheonixUtil.checkError(m_IntakeMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
            m_IntakeMotor.getName() + " failed to set voltage compensation", true);
        m_IntakeMotor.enableVoltageCompensation(true);        
        m_IntakeMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    public void setIntakePowerPercent(double power) {
        m_IntakeMotor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void simulationPeriodic() {
        this.periodic();//calls periodic during simulation
    }
}
