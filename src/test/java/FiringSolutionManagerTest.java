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

    @Test 
    public void testCalcNewSolutionExisting_insideTarmac() {
        assertEquals(manager.calcNewSolution(53), Constants.FiringSolutions.insideTarmac);
    }

    @Test
    public void testCalcNewSolution_insideMiddleTarmac() {
        FiringSolution insideTarmac = Constants.FiringSolutions.insideTarmac;
        FiringSolution middleTarmac = Constants.FiringSolutions.middleTarmac;

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
