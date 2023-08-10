package frc.robot.systems.leds;

import frc.robot.systems.leds.LedVars.Constants;

public class LedIO {


    public void setColor ( int r, int g, int b) {
        Constants.k_r = r;
        Constants.k_g = g;
        Constants.k_b = b;
    }
}
