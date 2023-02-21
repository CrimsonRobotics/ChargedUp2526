package frc.robot;

public class Helpers {

    /**
     * Returns the degree difference between two headings if the shortest distance
     * was taken.
     * 
     * @param h1 A number between 0 and 360
     * @param h2 A number between 0 and 360
     * @return double A number between 0 and 360
     */
    public static double hdgDiff(double h1, double h2) {
        double diff = (h1 - h2 + 3600) % 360;
        return diff <= 180.0 ? diff : 360.0 - diff;
    }

    /**
     * Returns true if its faster to go counter clocker wise to the new heading,
     * false otherwise.
     * 
     * @param hdg    A number between 0 and 360
     * @param newHdg A number between 0 and 360
     * @return boolean
     */
    public static boolean isTurnCCW(double hdg, double newHdg) {
        double diff = newHdg - hdg;
        return diff > 0.0 ? diff > 180 : diff >= -180;
    }
}
