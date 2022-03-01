package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.Constants.Ports;

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
        m_IndexerMotor = TalonFXFactory.createDefaultFalcon("Indexer Motor", Ports.INDEXER_MOTOR);//creates motor
        //configure motor
        m_IndexerMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        m_IndexerMotor.enableVoltageCompensation(true);        
        m_IndexerMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    public void setIndexerPercentPower(double power) {
        m_IndexerMotor.set(ControlMode.PercentOutput, power);
    }
}
