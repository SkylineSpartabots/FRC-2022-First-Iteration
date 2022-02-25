package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.TalonFXFactory;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    //get instance
    private static ShooterSubsystem instance = null;          
    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

    private final LazyTalonFX mMasterShooter, mSlaveShooter;  

    private ShooterSubsystem() {
        mMasterShooter = TalonFXFactory.createDefaultFalcon("Master Shooter Motor", 21);
        configureMotor(mMasterShooter, true);
        mSlaveShooter = TalonFXFactory.createSlaveFalcon("Slave Shooter Motor", 22, 21);
        mSlaveShooter.setMaster(mMasterShooter);
        configureMotor(mSlaveShooter, false);

    }

    private void configureMotor(LazyTalonFX talon, boolean b){
        talon.setInverted(b);
        talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Coast);
    }

    public void setShooterPercentPower(double power) {
        mMasterShooter.set(ControlMode.PercentOutput, power);
    }

    public boolean shooterAtVelocityRPS(double velocity){
        if(mMasterShooter.getSelectedSensorVelocity()/10 > velocity){
            return true;
        }
        else{
            return false;
        }
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter Velocity (RPS)", mMasterShooter.getSelectedSensorVelocity()/10);
    }
}