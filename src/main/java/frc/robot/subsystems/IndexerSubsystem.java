package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase{
    private static IndexerSubsystem instance = null;

    private final LazyTalonFX m_IndexerMotor;

    //variables for distance sensor
    private final AnalogInput mIndexerSensor;
    private final int INDEXER_SENSOR_PORT = 0;
    private final double mIndexerBeginTimeStamp = Double.POSITIVE_INFINITY;
    private final int mIndexerSensorThreshhold = 2;
    private boolean isBallDetectedBoolean = false;

    public static IndexerSubsystem getInstance(){
        if(instance == null){
            instance = new IndexerSubsystem();
        }
        return instance;
    }

    private IndexerSubsystem(){
        m_IndexerMotor = TalonFXFactory.createDefaultFalcon("Indexer Motor", Constants.IndexerConstants.INDEXER_MOTOR);
        configureIndexerMotor(m_IndexerMotor, false);
        mIndexerSensor = new AnalogInput(INDEXER_SENSOR_PORT);
    }

    private void configureIndexerMotor (LazyTalonFX talon, boolean inversion){
        talon.setInverted(inversion);
        PheonixUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
            talon.getName() + " failed to set voltage compensation", true);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Coast);
    }

    private boolean isBallDetected() {
        return mIndexerSensor.getValue() < mIndexerSensorThreshhold;
    }

    public void setIndexerPowerPercent(double power){
        m_IndexerMotor.set(ControlMode.PercentOutput, power);
    }

    private ShuffleboardTab debugTab = Shuffleboard.getTab("Indexer");

    @Override
    public void periodic() {
        debugTab.add("Indexer Talon Velocity", m_IndexerMotor.getSelectedSensorVelocity());
        debugTab.add("Indexer Talon Power", m_IndexerMotor.getMotorOutputPercent());
        debugTab.add("Indexer On?", true);
        isBallDetectedBoolean = isBallDetected();
        ifBallInIndexer(isBallDetectedBoolean);
    }

    private void ifBallInIndexer(boolean isBallDetectedBoolean){
        
    }

    @Override
    public void simulationPeriodic(){
        this.periodic();
    }
}