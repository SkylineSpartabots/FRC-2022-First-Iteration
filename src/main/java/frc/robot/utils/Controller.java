package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public class Controller {
    private XboxController xbox;
    private Button a, b, x, y, rb, lb, lstick, rstick, back, start;
    public Controller(XboxController xbox){
        this.xbox = xbox;
        a = new Button(xbox::getAButton);
        b = new Button(xbox::getBButton);
        x = new Button(xbox::getXButton);
        y = new Button(xbox::getYButton);
        lb = new Button(xbox::getLeftBumper);
        rb = new Button(xbox::getRightBumper);
        lstick = new Button(xbox::getLeftStickButton);
        rstick = new Button(xbox::getRightStickButton);
        back = new Button(xbox::getBackButton);
        start = new Button(xbox::getStartButton);
    }
    public Button getAButton(){ return a;}
    public Button getBButton(){ return b;}
    public Button getXButton(){ return x;}
    public Button getYButton(){ return y;}
    public Button getLeftBumper(){ return lb;}
    public Button getRightBumper(){ return rb;}
    public Button getLeftStickButton(){ return lstick;}
    public Button getRightStickButton(){ return rstick;}
    public Button getBackButton(){ return back;}
    public Button getStartButton(){ return start;}

    public double getLeftX(){ return xbox.getLeftX();}
    public double getLeftY(){ return xbox.getLeftY();}
    public double getRightX(){ return xbox.getRightX();}
    public double getRightY(){ return xbox.getRightY();}
}
