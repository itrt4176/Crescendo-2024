package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.subsystems.ClimberSim.ArmConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;


public class ClimberSim {
    public static final class ArmConstants {
        public static final int kMotorPort = 4;
      
        public static final double kP = 1;
      
        // These are fake gains; in actuality these must be determined individually for each robot
        public static final double kSVolts = 1;
        public static final double kGVolts = 1;
        public static final double kVVoltSecondPerRad = 0.5;
        public static final double kAVoltSecondSquaredPerRad = 0.1;
      
        public static final double kMaxVelocityRadPerSecond = 3;
        public static final double kMaxAccelerationRadPerSecSquared = 10;
      
        public static final int[] kEncoderPorts = new int[] {4, 5};
        public static final int kEncoderPPR = 256;
        public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;
      
        // The offset of the arm from the horizontal in its neutral position,
        // measured from the horizontal
        public static final double kArmOffsetRads = 0.5;
      }

        

    /** A robot arm subsystem that moves with a motion profile. */
    public class ArmSubsystem extends ProfiledPIDSubsystem {
    private final PWMSparkMax m_motor = new PWMSparkMax(ArmConstants.kMotorPort);
    private final Encoder m_encoder =
        new Encoder(ArmConstants.kEncoderPorts[0], ArmConstants.kEncoderPorts[1]);
    private final ArmFeedforward m_feedforward =
        new ArmFeedforward(
            ArmConstants.kSVolts, ArmConstants.kGVolts,
            ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

    /** Create a new ArmSubsystem. */
    public ArmSubsystem() {
        super(
            new ProfiledPIDController(
                ArmConstants.kP,
                0,
                0,
                new TrapezoidProfile.Constraints(
                    ArmConstants.kMaxVelocityRadPerSecond,
                    ArmConstants.kMaxAccelerationRadPerSecSquared)),
            0);
        m_encoder.setDistancePerPulse(ArmConstants.kEncoderDistancePerPulse);
        // Start arm at rest in neutral position
        setGoal(ArmConstants.kArmOffsetRads);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        m_motor.setVoltage(output + feedforward);
    }

    @Override
    public double getMeasurement() {
        return m_encoder.getDistance() + ArmConstants.kArmOffsetRads;
    }
    }

    


}
