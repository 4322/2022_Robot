package frc.robot.utility;

public class OrangeMath {
    public static boolean equalToTwoDecimal(double num1, double num2) {
        double epsilon = 0.01;
        
        return Math.abs(num1 - num2) < epsilon;
    }
}