package frc.robot.FiringSolution;

import java.util.*;

import frc.robot.Constants;

// https://github.com/Team3309/FRC2020/blob/master/src/main/java/frc/robot/util/FiringSolutionManager.java

public class FiringSolutionManager { 

    private static FiringSolutionManager singleton;
    private ArrayList<FiringSolution> solutions;

    private FiringSolutionManager() {
        singleton = this;
        solutions = new ArrayList<FiringSolution>(); // new ArrayList to organize based on distance

        for (FiringSolution solution : Constants.firingSolutions) {
            addSolution(solution);
        }
    }

    public void addSolution(FiringSolution add) { 
        int i;
        // iterate through ArrayList until desired position found
        for (i = 0; i < solutions.size(); i++) {
            if (solutions.get(i).getDistance() == add.getDistance()) {
                break;
            } else if (solutions.get(i).getDistance() > add.getDistance()) { 
                solutions.add(i, add);
                break;
            }
        }
        if (i == solutions.size()) {
            solutions.add(i, add);
        }
    }

    public FiringSolution calcNewSolution(double currentDistance) {
        int upper = -1; // position of upper value (initialized to -1 as -1 is not a valid list index)
        int lower = -1; // position of lower value

        for (int i = 0; i < solutions.size(); i++) {
            if (solutions.get(i).getDistance() == currentDistance) {
                return solutions.get(i);
            }
            if (solutions.get(i).getDistance() > currentDistance) {
                upper = i;
                lower = i-1;
                break;
            }
        }

        if (upper == -1) {
            return solutions.get(solutions.size() - 1); // returns highest/lowest solution
        }
        if (lower == -1) {
            return solutions.get(0);
        }
        else {
            double d = (currentDistance - solutions.get(lower).getDistance()) / 
              (solutions.get(upper).getDistance() - solutions.get(lower).getDistance());
            return new FiringSolution(
              solutions.get(upper).getKickerSpeed() * d + solutions.get(lower).getKickerSpeed() * (1 - d),
              solutions.get(upper).getFlywheelSpeed() * d + solutions.get(lower).getFlywheelSpeed() * (1 - d),
              solutions.get(upper).getHoodPosition() * d + solutions.get(lower).getHoodPosition() * (1 - d)
            );
        }
    }

    public static FiringSolutionManager getSingleton() {

        if (singleton == null) {
            singleton = new FiringSolutionManager();
        }
 
        return singleton;
    }
}