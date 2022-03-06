package frc.robot.subsystems;


import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
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
    private static final double distToBallLoaded = 10; // this is the distance from the color sensor to the ball
    private static ColorSensorSubsystem mInstance;
    private static ColorSensorSubsystem getmInstance = null;
    private final ShuffleboardTab debugTab = Shuffleboard.getTab("Color");
    private static Color detectedColor;

    private ColorSensorSubsystem() {
        m_colorMatcher.addColorMatch(Constants.ColorSensor.blue);
        m_colorMatcher.addColorMatch(Constants.ColorSensor.red);
    }

    public static ColorSensorSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ColorSensorSubsystem();
        }
        return mInstance;
    }

    private static ColorMatchResult MatchedColor() {
        return m_colorMatcher.matchClosestColor(detectedColor);
    }

    private static String RedOrBlue() {
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

    public static int getProximity() {
        return m_colorSensor.getProximity();
    }

    public static boolean ballLoaded() {
        return getProximity() == distToBallLoaded;
    }


    @Override
    public void periodic() {
        detectedColor = m_colorSensor.getColor();
        debugTab.add("proximity to next object", getProximity());
        debugTab.add("red", detectedColor.red); //gives raw red value
        debugTab.add("blue", detectedColor.blue);
        debugTab.add("green", detectedColor.green);
        debugTab.add("detected color", RedOrBlue());
    }

    /* TODO (in general)
     *  implement a way to check if a ball is loaded - done, see isBallLoaded
     *  create 3 color sensors rather than the 1 in code
     *  periodic
     *  tune distToBallLoaded var and the Color constants
     *  ensure the i2cport var works
     */
}
