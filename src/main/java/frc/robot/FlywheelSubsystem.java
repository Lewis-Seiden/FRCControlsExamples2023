// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Monologue.LogNT;

public class FlywheelSubsystem extends SubsystemBase implements Logged {
  // https://www.reca.lc/flywheel?currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&flywheelMomentOfInertia=%7B%22s%22%3A3%2C%22u%22%3A%22in2%2Albs%22%7D&flywheelRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&flywheelRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Reduction%22%7D&flywheelWeight=%7B%22s%22%3A1.5%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Falcon%20500%22%7D&motorRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Step-up%22%7D&projectileRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&projectileWeight=%7B%22s%22%3A5%2C%22u%22%3A%22lbs%22%7D&shooterMomentOfInertia=%7B%22s%22%3A4.5%2C%22u%22%3A%22in2%2Albs%22%7D&shooterRadius=%7B%22s%22%3A3%2C%22u%22%3A%22in%22%7D&shooterTargetSpeed=%7B%22s%22%3A5000%2C%22u%22%3A%22rpm%22%7D&shooterWeight=%7B%22s%22%3A1%2C%22u%22%3A%22lbs%22%7D&useCustomFlywheelMoi=0&useCustomShooterMoi=0
  // Isnt behaving how i expect
  FlywheelSim sim = new FlywheelSim(DCMotor.getFalcon500(2), 1, 8.0);
  // Static friction is not modelled
  SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.0, 0.12);
  PIDController pid = new PIDController(0.0, 0.0, 0.0);

  @LogNT
  double reference = 0.0;

  @LogNT
  double volts = 0.0;

  /** Creates a new FlywheelSubsystem. */
  public FlywheelSubsystem() {}

  private void setOutput(double volts) {
    // Clamp voltage
    volts = Math.max(-12.0, Math.min(12.0, volts));
    sim.setInput(volts);
    this.volts = volts;
  }

  public CommandBase setTarget(double rpm) {
    return this.run(() -> {reference = rpm / 60; setOutput(ff.calculate(rpm / 60));});
  }

  @LogNT
  public double getRPM() {
    return sim.getAngularVelocityRPM();
  }

  @Override
  public void periodic() {
    sim.update(0.020);
  }
}
