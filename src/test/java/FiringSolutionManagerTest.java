import static org.junit.Assert.*;
import org.junit.*;

import frc.robot.Constants;
import frc.robot.FiringSolution.*;

public class FiringSolutionManagerTest {
    FiringSolutionManager manager;

    @Before
    public void setup() {
        manager = FiringSolutionManager.getSingleton();
    }

    @After
    public void shutdown() {
        return; // nothing to shut down
    }

    // Test a solution for an existing firing solution (should give back the same firing solution)
    @Test 
    public void testCalcNewSolutionExisting_insideTarmac() {
        assertEquals(manager.calcNewSolution(Constants.FiringSolutions.insideTarmac.getDistance()),
        Constants.FiringSolutions.insideTarmac);
    }

    // Test a solution for a distance value that is below the shortest firing solution
    // (should give back lowest firing solution, in this case middleTarmac)
    @Test
    public void testCalcNewSolutionBelowMinimum_middleTarmac() {
        assertEquals(manager.calcNewSolution(0), Constants.FiringSolutions.middleTarmac);
    }

    // Test a solution for a distance value that is above the longest firing solution
    // (should give back highest firing solution, in this case reallyFar)
    @Test
    public void testCalcNewSolutionAboveMaximum_reallyFar() {
        assertEquals(manager.calcNewSolution(1000), Constants.FiringSolutions.reallyFar);
    }

    @Test
    public void testCalcNewSolution_insideMiddleTarmac() {
        FiringSolution insideTarmac = Constants.FiringSolutions.insideTarmac;
        FiringSolution middleTarmac = Constants.FiringSolutions.middleTarmac;

        // Since this is the midpoint, find the average between the two values for the new firing solution
        double newKickerSpeed = (insideTarmac.getKickerSpeed() + middleTarmac.getKickerSpeed()) / 2;
        double newFlywheelSpeed = (insideTarmac.getFlywheelSpeed() + middleTarmac.getFlywheelSpeed()) / 2;
        double newHoodPosition = (insideTarmac.getHoodPosition() + middleTarmac.getHoodPosition()) / 2;
        double newAimingTolerance = (insideTarmac.getAimingToleranceDeg() + middleTarmac.getAimingToleranceDeg()) / 2;
        double newDistance = (insideTarmac.getDistance() + middleTarmac.getDistance()) / 2;

        FiringSolution insideMiddleTarmac = new FiringSolution(
            newKickerSpeed, 
            newFlywheelSpeed, 
            newHoodPosition,
            newAimingTolerance,
            newDistance
        );

        assertEquals(manager.calcNewSolution(newDistance), insideMiddleTarmac);
    }
}
