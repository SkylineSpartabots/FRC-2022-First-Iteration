package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXFactory;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase{   

    private final LazyTalonFX m_IndexerMotor;

    //get instance of subsystem    
    private static IndexerSubsystem instance = null;
    public static IndexerSubsystem getInstance() {
        if (instance == null) {
            instance = new IndexerSubsystem();
        }
        return instance;
    }

    public IndexerSubsystem() {
        m_IndexerMotor = TalonFXFactory.createDefaultFalcon("Shooter Motor", 32);//creates motor
        //configure motor
        PheonixUtil.checkError(m_IndexerMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
            m_IndexerMotor.getName() + " failed to set voltage compensation", true);
        m_IndexerMotor.enableVoltageCompensation(true);        
        m_IndexerMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    public void setIndexerPercentPower(double power) {
        m_IndexerMotor.set(ControlMode.PercentOutput, power);
    }
}
