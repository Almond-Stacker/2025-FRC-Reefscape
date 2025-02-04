package frc.lib.util;

public class Utilities {
    public static final double sparkFlexResolution = 7168;

    public static double polynomialAccleration(double x) {
        return Math.pow(x, 3) * 0.795903 + x * 0.203938;
    }
}