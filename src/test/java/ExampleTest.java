import static org.junit.Assert.*;
import org.junit.*;

// unit test naming convention: test<MethodToTest>_desiredState

public class ExampleTest {
  
  // this code will run before each test
  @Before
  public void setup() {
    return;
  }

  // this code will run after each test 
  @After 
  public void shutdown() {
    return;
  }

  // this test will pass, as 1 + 1 = 2
  @Test
  public void testOnePlusOne_Two() {
    assertEquals(1+1, 2);
  }

  // this test would fail if run (which would cause the build to fail), so it is skipped
  @Ignore
    @Test
    public void testOnePlusOne_Three() {
      assertEquals(1+1, 3);
    }

}