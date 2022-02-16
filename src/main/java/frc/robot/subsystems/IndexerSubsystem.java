package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase{
    private static IndexerSubsystem instance = null;

    private final LazyTalonFX mIndexerMotor;

    public static IndexerSubsystem getInstance(){
        if(instance == null){
            instance = new IndexerSubsystem();
        }
        return instance;
    }

    private IndexerSubsystem(){
        mIndexerMotor = TalonFXFactory.createDefaultFalcon("Indexer Motor", Constants.IndexerConstants.INDEXER_MOTOR);
        configureIndexerMotor(mIndexerMotor, false);
    }

    private void configureIndexerMotor (LazyTalonFX talon, boolean inversion){
        talon.setInverted(inversion);
        PheonixUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
            talon.getName() + " failed to set voltage compensation", true);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Coast);
    }

    public void setIndexerPowerPercent(double power){
        mIndexerMotor.set(ControlMode.PercentOutput, power);
    }

    private ShuffleboardTab debugTab = Shuffleboard.getTab("Shooter");
    private NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = tableInstance.getTable("Shooter");

    @Override
    public void periodic() {
        table.getEntry("Indexer Talon Velocity").setDouble(mIndexerMotor.getSelectedSensorVelocity());
        table.getEntry("Indexer Talon Power").setDouble(mIndexerMotor.getMotorOutputPercent());
        table.getEntry("Indexer On?").setBoolean(true);
        //table.getEntry("Indexer Subsystem").setValue(IndexerSubsystem.instance);
    }

    @Override
    public void simulationPeriodic(){
        this.periodic();
    }
}