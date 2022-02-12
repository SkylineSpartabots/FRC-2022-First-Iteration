package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    public static final short deviceID = 0;
    public static final TalonFX shooterMotor = new TalonFX(deviceID);
    public static final double turnCoefficent = Constants.Turret.turnCoefficient;
    private static DrivetrainSubsystem instance = null;

    public static DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }
        return instance;
    }

    public static void turnTurret(double percent) {  // I don't know what im doing with TalonFX, so this is going to go great
        shooterMotor.set(TalonFXControlMode.PercentOutput, percent * turnCoefficent);
    }

    @Override
    public void periodic() {
        //TODO: shuffleboard integration
    }

    @Override
    public void simulationPeriodic() {
        this.periodic(); //TODO: shuffleboard integration part 2
    }

}
