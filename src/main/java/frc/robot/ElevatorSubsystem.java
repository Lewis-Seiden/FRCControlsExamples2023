// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Monologue.LogNT;

public class ElevatorSubsystem extends SubsystemBase implements Logged {
  // https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A200%2C%22u%22%3A%22A%22%7D&efficiency=100&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A10%2C%22u%22%3A%22kg%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22Falcon%20500%22%7D&ratio=%7B%22magnitude%22%3A5%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A1%2C%22u%22%3A%22m%22%7D
  ElevatorSim sim = new ElevatorSim(
    DCMotor.getFalcon500(2), 
    5.0, 
    10.0, 
    Units.inchesToMeters(1.0), 
    0, 
    2.0, 
    true, 
    0);
    
  // Static friction isnt modeled
  // These gains dont exactly match the recalc gains
  // Not sure if thats a sim or units issue or something else
  ElevatorFeedforward ff = new ElevatorFeedforward(0.0, 0.64, 3.6);
  ProfiledPIDController ppid = new ProfiledPIDController(40.0, 0.0, 5.0, new Constraints(1.5, 15.0));
  @LogNT
  double ref = 0.0;
  @LogNT
  Mechanism2d elevatorM2d = new Mechanism2d(2.0, 3.0);
  MechanismRoot2d root = elevatorM2d.getRoot("Elevator Root", 1.0, 0.0);
  MechanismLigament2d lig = new MechanismLigament2d("Elevator", 0.0, 90.0);
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    root.append(lig);
  }

  public CommandBase setHeight(double meters) {
    return this.runOnce(() -> {ref = meters; ppid.reset(getPosition(), getVelocity()); ppid.setGoal(meters);}).andThen(this.run(() -> {
        var state = ppid.getSetpoint();
        sim.setInput(ff.calculate(state.velocity) + ppid.calculate(getPosition()));
      }));
  }

  @LogNT
  public double getPosition() {
    return sim.getPositionMeters();
  }

  @LogNT
  public double getVelocity() {
    return sim.getVelocityMetersPerSecond();
  }

  @LogNT
  public double getSetpoint() {
    return ppid.getSetpoint().position;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sim.update(0.020);
    lig.setLength(getPosition());
  }
}
