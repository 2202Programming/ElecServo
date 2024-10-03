// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;


public class ElecServo extends SubsystemBase {
  PIDController pid = new PIDController(0.0, 0.0, 0.0);
  PIDFController hwAngleVelPID = new PIDFController(0, 0, 0, 0);
  NeoServo elec = new NeoServo(10, pid, hwAngleVelPID, false);
  final double AngleGearRatio = 14.0;
  final int STALL_CURRENT = 5;
  final int FREE_CURRENT = 15;
  final double maxVel = 1.0;
  final double maxAccel = 0.5;
  final double posTol = 1.0;
  final double velTol = 1.0;
  /** Creates a new ElecServo. */
  public ElecServo() {
    hwAngleVelPID.copyTo(elec.getController().getPIDController(), 0);
    elec.setConversionFactor(360.0 / AngleGearRatio) // [deg] for internal encoder behind gears
              // .setConversionFactor(360.0) // [deg] external encoder on arm shaft
              .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
              .setVelocityHW_PID(maxVel, maxAccel)
              .setTolerance(posTol, velTol)
              .setMaxVelocity(maxVel)
              .burnFlash();
  }

  public void move(){
    elec.setVelocityCmd(0.3);
  }
  public void setSetpoint(double setpoint){
    elec.setSetpoint(setpoint);
  }
  public void setPosition(double pos){
    elec.setPosition(pos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
