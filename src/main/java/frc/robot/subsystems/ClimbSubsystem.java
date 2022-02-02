package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.LazyTalonSRX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXFactory;
import frc.lib.drivers.TalonFXUtil;
import frc.lib.drivers.TalonSRXFactory;
import frc.lib.drivers.TalonSRXUtil;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    private static ClimbSubsystem mInstance = null;
    
    public static ClimbSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ClimbSubsystem();
        }
        return mInstance;
    }    
    
    //hardware
    private final LazyTalonSRX mHookSlideMotor;
    private final LazyTalonFX mMasterWinch, mFollowerWinch;
    private final Encoder mClimbHookEncoder;
     //hardware states
     private ClimbControlState mCurrentState;
     private boolean mStateChanged = false;
     private double mStateChangeTimestamp = 0.0;
     
     public enum ClimbControlState {
        OFF(0.0, 0.0),
        RAISE_HOOK(0.35, 0.0),
        LOWER_HOOK(-0.2, 0.0),
        OVERRIDE_RAISE_HOOK(0.25, 0.0),
        OVERRIDE_LOWER_HOOK(-0.2, 0.0),
        WINCH_UP(0.0, 0.95),
        AUTO_WINCH_UP(-0.4, 0.95);

        private double hookSlideSpeed = 0.0;
        private double winchSpeed = 0.0;

        private ClimbControlState(double hookSlideSpeed, double winchSpeed) {
            this.hookSlideSpeed = hookSlideSpeed;
            this.winchSpeed = winchSpeed;
        }
    }         
    
    private void configureHookMotor(InvertType inversion) {
        mHookSlideMotor.setInverted(inversion);

        PheonixUtil.checkError(mHookSlideMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs), 
             "Hook Slide Motor failed to set voltage compensation", true);
        mHookSlideMotor.enableVoltageCompensation(true);

        TalonSRXUtil.setCurrentLimit(mHookSlideMotor, 5);

        mHookSlideMotor.setNeutralMode(NeutralMode.Brake);
    }
    private void configureWinchMotor(LazyTalonFX falcon, InvertType inversion) {
        falcon.setInverted(inversion);

        TalonFXUtil.setStatorCurrentLimit(falcon, 40);
        TalonFXUtil.setSupplyCurrentLimit(falcon, 60);

        PheonixUtil.checkError(falcon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs), 
             falcon.getName() + " failed tp set voltage compensation", true);
        mHookSlideMotor.enableVoltageCompensation(true);

        falcon.setNeutralMode(NeutralMode.Coast);
    }

    private ClimbSubsystem() {
        mHookSlideMotor = TalonSRXFactory.createDefaultTalon("Hook Slide Motor", Constants.Ports.CLIMB_HOOK_ID);
        configureHookMotor(InvertType.None);
    
        mMasterWinch = TalonFXFactory.createDefaultFalcon("Master Winch Motor", Constants.Ports.CLIMB_MASTER_WINCH_ID);
        configureWinchMotor(mMasterWinch, InvertType.None);

        mFollowerWinch = TalonFXFactory.createSlaveFalcon("Follower Winch Motor", Constants.Ports.CLIMB_SLAVE_WINCH_ID, Constants.Ports.CLIMB_MASTER_WINCH_ID);
        mFollowerWinch.setMaster(mMasterWinch);
        configureWinchMotor(mFollowerWinch, InvertType.OpposeMaster);

        mClimbHookEncoder = new Encoder(Constants.Ports.CLIMB_ENCODER_B, Constants.Ports.CLIMB_ENCODER_A);
    }
    public synchronized ClimbControlState getState() {
        return mCurrentState;
    }

    public synchronized void setHookSlideSpeed(double percentOutput) {
        mHookSlideMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public synchronized void setWinchSpeed(double percentOutput) {
        mMasterWinch.set(ControlMode.PercentOutput, Math.abs(percentOutput));
    }

    public synchronized boolean setState(ClimbControlState desiredState) {
        boolean interrupted = false;
        mCurrentState = desiredState;
        if(mCurrentState == ClimbControlState.RAISE_HOOK && mClimbHookEncoder.getDistance() > Constants.ClimbConstants.kClimbMaxHeight) {
            setHookSlideSpeed(0.0);
            interrupted = true;
        } else if(mCurrentState == ClimbControlState.LOWER_HOOK && mClimbHookEncoder.getDistance() < 0.0) {
            setHookSlideSpeed(0.0);
            interrupted = true;
        } else {
            setHookSlideSpeed(desiredState.hookSlideSpeed);
        }

        setWinchSpeed(desiredState.winchSpeed);
        return interrupted;
    }
    @Override
    public void periodic() {
        //TODO;
        //SHUFFLEBOARD HERE
    }

    @Override
    public void simulationPeriodic() {
        this.periodic();
    }
}