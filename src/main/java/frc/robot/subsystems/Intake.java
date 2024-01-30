// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private static final double kFlywheelGearing = 1.0;
  private TalonFX mainFx;
  private TalonFX followFx;
  private AnalogInput sharp;
  private double fakeDistance = 1;
  
  private static final double kFlywheelMomentOfInertia =
        0.5 * Units.lbsToKilograms(1.5) * Math.pow(Units.inchesToMeters(4), 2);

  private final FlywheelSim m_flywheelSim =
      new FlywheelSim(DCMotor.getNEO(1), kFlywheelGearing, kFlywheelMomentOfInertia);
//    intake.isNoteLoaded().toggleOnTrue(new StartEndCommand(intake :: stop, intake :: stop));

  public void setFakeDistance(double x){
    fakeDistance = x;
  }

  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated velocities to our simulated encoder
    m_flywheelSim.setInputVoltage(mainFx.get() * RobotController.getInputVoltage());
    m_flywheelSim.update(0.02);
    SmartDashboard.putNumber("Fly Wheel Speed", m_flywheelSim.getAngularVelocityRPM());
  }
  

  public Intake() {
    mainFx = new TalonFX(Constants.IntakeConstants.INTAKE_MAIN);
    followFx = new TalonFX(Constants.IntakeConstants.INTAKE_FOLLOW);
    sharp = new AnalogInput(Constants.IntakeConstants.SHARP);

    mainFx.setNeutralMode(NeutralModeValue.Brake);
    followFx.setNeutralMode(NeutralModeValue.Brake);

    followFx.setInverted(true);
    
  }


  public void setIntakeSpeed(double speed)
  {
    mainFx.set(speed);
    followFx.set(mainFx.get());//to make up for lack of follow method now


  }

  public void on()
  {
    setIntakeSpeed(.4);
  }

  public void stop()
  {
    mainFx.set(0);
    followFx.set(mainFx.get());
  }


  public double getDistance()
  {
    if (Robot.isSimulation()){
      return fakeDistance;
    }
    else{
      return (Math.pow(sharp.getAverageVoltage(), -1.2045)) * 27.726;
    }
  }

  public boolean isNoteLoaded()
  {
    SmartDashboard.putNumber("Distance", getDistance());
    if(getDistance() < 0)//placeholder condition that nedds to be tested
    {
      return true;
    }

    return false;
  }

  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Set Point", mainFx.get());
    
    if (isNoteLoaded())
    {
      this.stop();
    }
  }
}
