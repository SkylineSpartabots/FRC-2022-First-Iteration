package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXFactory;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class IntakeSubsystem extends SubsystemBase {

    private static IntakeSubsystem instance = null;

    //hardware
    private final LazyTalonFX m_IntakeMotor;
    //private final Solenoid m_RightSolenoidMotor, m_LeftSolenoidMotor;

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    public IntakeSubsystem() {
        m_IntakeMotor = TalonFXFactory.createDefaultFalcon("Shooter Motor", Constants.IntakeConstants.INTAKE_MOTOR_ID);
		configureMotor(m_IntakeMotor, InvertType.None);
        /*
        m_RightSolenoidMotor = new Solenoid(PneumaticsModuleType.REVPH, Constants.IntakeConstants.RIGHT_SOLENOID_CHANNEL);
        m_LeftSolenoidMotor = new Solenoid(PneumaticsModuleType.REVPH, Constants.IntakeConstants.LEFT_SOLENOID_CHANNEL);
        */
    }

    private void configureMotor(LazyTalonFX m_IntakeMotor2, InvertType inversion) {
        m_IntakeMotor2.setInverted(inversion);
        PheonixUtil.checkError(m_IntakeMotor2.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
                m_IntakeMotor2.getName() + " failed to set voltage compensation", true);
        m_IntakeMotor2.enableVoltageCompensation(true);
        m_IntakeMotor2.setNeutralMode(NeutralMode.Coast);
    }

    public void setIntakePowerPercent(double power) {
        m_IntakeMotor.set(ControlMode.PercentOutput, power);
    }

    /*
    public void setDeployState(boolean isDeployed) {
        m_RightSolenoidMotor.set(isDeployed);
        m_LeftSolenoidMotor.set(isDeployed);
    }
    */

    private ShuffleboardTab debugTab = Shuffleboard.getTab("Intake");

    @Override
    public void periodic() {
        debugTab.add("Intake Supply Current", m_IntakeMotor.getSupplyCurrent());
        debugTab.add("Intake Stator Current", m_IntakeMotor.getStatorCurrent());
        debugTab.add("Intake Last Set", m_IntakeMotor.getLastSet());
        //debugTab.add("Intake Solenoid State", m_RightSolenoidMotor.get());
    }

    @Override
    public void simulationPeriodic() {
        this.periodic();
    }
}