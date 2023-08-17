package frc.robot.systems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedVars {
    public static final class Constants {
        public static int k_r = 200;
        public static int k_g = 233;
        public static int k_b = 233;
        public static int k_r2 = 0;
        public static int k_g2 = 0;
        public static int k_b2 = 0;
    }

    public static class Objects {
        public static AddressableLED ledStrip = new AddressableLED(1);
        public static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(18);
    }
}
