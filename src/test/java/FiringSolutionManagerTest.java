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
}
