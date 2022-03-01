package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.Constants.Ports;

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
        m_IntakeMotor = TalonFXFactory.createDefaultFalcon("Intake Motor", Ports.INTAKE_MOTOR);//creates motor
        //configure motor
        m_IntakeMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        m_IntakeMotor.enableVoltageCompensation(true);        
        m_IntakeMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    public void setIntakePercentPower(double power) {
        m_IntakeMotor.set(ControlMode.PercentOutput, power);
    }
}
