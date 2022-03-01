package frc.robot.subsystems;


import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Objects;

public class ColorSensorSubsystem extends SubsystemBase {
    private static final I2C.Port i2cPort = I2C.Port.kOnboard; // unsure what this is, TODO figureout if this works
    private static final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private static final ColorMatch m_colorMatcher = new ColorMatch();
    private static ColorSensorSubsystem mInstance;
    private static ColorSensorSubsystem getmInstance = null;
    private ShuffleboardTab debugTab = Shuffleboard.getTab("Shooter");
    private NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = tableInstance.getTable("Shooter");

    private ColorSensorSubsystem() {

        m_colorMatcher.addColorMatch(Constants.ColorSensor.blue);
        m_colorMatcher.addColorMatch(Constants.ColorSensor.red);

//        SmartDashboard.putNumber("Red", detected.red);
//        SmartDashboard.putNumber("Blue", detected.blue);
//        SmartDashboard.putNumber("Confidence", match.confidence);
//        SmartDashboard.putString("Detected Color", detectedColor);
    }

    public static ColorSensorSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ColorSensorSubsystem();
        }
        return mInstance;
    }

    private static ColorMatchResult MatchedColor() {
        Color detected = m_colorSensor.getColor();
        Color detectedColor = m_colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        return match;
    }

    private static String RedOrBlue() {
        Color detected = m_colorSensor.getColor();
        String detectedColor;
        final ColorMatchResult match = MatchedColor();
        if (Objects.equals(match.color, Constants.ColorSensor.blue)) {
            detectedColor = "Blue";
        } else if (Objects.equals(match.color, Constants.ColorSensor.red)) {
            detectedColor = "Red";
        } else {
            detectedColor = "Unknown";
        }
        return detectedColor;
    }

/*  @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        this.periodic();
    }
 */
    // TODO figure out how to do periodic lol
}
