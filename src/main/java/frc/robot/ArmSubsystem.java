// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Monologue.LogNT;

public class ArmSubsystem extends SubsystemBase implements Logged {
  // https://www.reca.lc/arm?armMass=%7B%22s%22%3A15%2C%22u%22%3A%22kg%22%7D&comLength=%7B%22s%22%3A20%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A100%2C%22u%22%3A%22A%22%7D&efficiency=100&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22Falcon%20500%22%7D&ratio=%7B%22magnitude%22%3A100%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
  SingleJointedArmSim sim = new SingleJointedArmSim(
    DCMotor.getFalcon500(2), 
    100, 
    SingleJointedArmSim.estimateMOI(Units.inchesToMeters(20), 15), 
    Units.inchesToMeters(40), 
    -Math.PI, 
    Math.PI, 
    true, 
    0);
  // Static friction isnt modeled
  // These gains dont exactly match the recalc gains
  // Not sure if thats a sim or units issue or something else
  ArmFeedforward ff = new ArmFeedforward(0.0, 0.24, 2.0);
  ProfiledPIDController ppid = new ProfiledPIDController(40.0, 0.0, 5.0, new Constraints(Math.PI, 15.0));
  @LogNT
  double ref = 0.0;
  @LogNT
  Mechanism2d armM2d = new Mechanism2d(3.0, 3.0);
  MechanismRoot2d root = armM2d.getRoot("Arm Root", 1.5, 1.5);
  MechanismLigament2d lig = new MechanismLigament2d("Arm", Units.inchesToMeters(40), 0.0);
  /** Creates a new ElevatorSubsystem. */
  public ArmSubsystem() {
    root.append(lig);
  }

  public CommandBase setAngle(double radians) {
    return this.runOnce(() -> {ref = radians; ppid.reset(getPosition(), getVelocity()); ppid.setGoal(radians);}).andThen(this.run(() -> {
        var state = ppid.getSetpoint();
        sim.setInput(Math.min(Math.max(ff.calculate(getPosition(), state.velocity) + ppid.calculate(getPosition()), -12.0), 12.0));
      }));
  }

  @LogNT
  public double getPosition() {
    return sim.getAngleRads();
  }

  @LogNT
  public double getVelocity() {
    return sim.getVelocityRadPerSec();
  }

  @LogNT
  public double getSetpoint() {
    return ppid.getSetpoint().position;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sim.update(0.020);
    lig.setAngle(Rotation2d.fromRadians(getPosition()));
  }
}
