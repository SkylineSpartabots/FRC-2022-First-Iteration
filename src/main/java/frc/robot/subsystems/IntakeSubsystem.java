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

public class IntakeSubsystem extends SubsystemBase{
    private static IntakeSubsystem instance = null;

    private final LazyTalonFX mIntakeMotor;

    public static IntakeSubsystem getInstance(){
        if(instance == null){
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    private IntakeSubsystem(){
        mIntakeMotor = TalonFXFactory.createDefaultFalcon("Intake Motor", Constants.IntakeConstants.INTAKE_MOTOR);
        configureIntakeMotor(mIntakeMotor, false);
    }

    private void configureIntakeMotor (LazyTalonFX talon, boolean inversion){
        talon.setInverted(inversion);
        PheonixUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
            talon.getName() + " failed to set voltage compensation", true);
        talon.enableVoltageCompensation(false);
        talon.setNeutralMode(NeutralMode.Coast);
    }

    public void setIntakePowerPercent(){
        mIntakeMotor.set(ControlMode.PercentOutput, 0.5);
    }
    /*
    private ShuffleboardTab debugTab = Shuffleboard.getTab("Shooter");
    private NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = tableInstance.getTable("Shooter");

    @Override
    public void periodic() {
        table.getEntry("Intake Talon Velocity").setDouble(mIntakeMotor.getSelectedSensorVelocity());
        table.getEntry("Intake Talon Power").setDouble(mIntakeMotor.getMotorOutputPercent());
        table.getEntry("Intake On?").setBoolean(true);
        table.getEntry("Intake Subsystem").setValue(IntakeSubsystem.instance);
    }
    */
    @Override
    public void simulationPeriodic(){
        this.periodic();
    }
}