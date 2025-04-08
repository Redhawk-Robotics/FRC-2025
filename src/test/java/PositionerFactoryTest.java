package frc.robot.Commands;
// https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html#writing-tests

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;
import frc.robot.Commands.PositionerFactory;
// import org.junit.jupiter.api.AfterEach;
// import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

// TODO test proper creation of trajectory

class PositionerFactoryTest {
    @Test // marks this method as a test
    void testTrajectory() {
        PositionerFactory.State start =
                new PositionerFactory.State(0., 70., null, null, 1., null, null);
        PositionerFactory.State end =
                new PositionerFactory.State(100., 100., null, null, null, null, null);
        Optional<PositionerFactory.State[]> result =
                PositionerFactory.GoToState.getMidList(start, end, null, null);

        if (result.isEmpty()) {
            System.out.println("empty");
            return;
        }
        PositionerFactory.State[] list = result.get();
        System.out.println(start);
        for (int i = 0; i < list.length; i++) {
            System.out.println(list[i]);
        }
        System.out.println(end);
    }
}
