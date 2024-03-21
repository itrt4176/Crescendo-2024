package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millisecond;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Velocity;

public final class UnitsITRT {
    private UnitsITRT() {
    }
    
    public static final Velocity<Distance> MetersPerMs = Meters.per(Millisecond);
}
