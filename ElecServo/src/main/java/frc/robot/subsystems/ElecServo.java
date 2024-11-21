// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;
import edu.wpi.first.wpilibj.AnalogInput;


public class ElecServo extends SubsystemBase {
  private SparkAnalogSensor m_analogSensor;
  private AnalogEncoder analogEncoder = new AnalogEncoder(3); 
   private SparkPIDController m_pidController;
  private AnalogInput analogInput = new AnalogInput(0);
  PIDController pid = new PIDController(1.0, 1.0, 1.0);
  PIDFController hwAngleVelPID = new PIDFController(1, 1, 1, 1);
  NeoServo elec = new NeoServo(18, pid, hwAngleVelPID, false);
  final double AngleGearRatio = 800.0;
  final int STALL_CURRENT = 5;
  final int FREE_CURRENT = 15;
  final double maxVel = 1.0;
  final double maxAccel = 0.5;
  final double posTol = 1.0;
  final double velTol = 1.0;
  /** Creates a new ElecServo. */
  public ElecServo() {
  m_analogSensor = elec.getController().getAnalog(SparkAnalogSensor.Mode.kRelative);
  m_pidController = elec.getController().getPIDController();
  m_pidController.setFeedbackDevice(m_analogSensor);
    ntcreate();
    hwAngleVelPID.copyTo(elec.getController().getPIDController(), 0);
    elec.setConversionFactor(360.0 / AngleGearRatio) // [deg] for internal encoder behind gears
              // .setConversionFactor(360.0) // [deg] external encoder on arm shaft
              .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
              .setVelocityHW_PID(maxVel, maxAccel)
              .setTolerance(posTol, velTol)
              .setMaxVelocity(maxVel)
              .burnFlash();
  }
  public void updateAnalogPosition() {
    double analogValue = analogInput.getVoltage();
    double position = mapAnalogToServo(analogValue);
    elec.setPosition(position);
  }

  private double mapAnalogToServo(double analogValue) {
    return (analogValue / 3.3) * 360.0;
  }

  public void move(double vel){
    elec.setVelocityCmd(vel);
    System.out.println(getDesiredVel());
  }
  public void setSetpoint(double setpoint){
    elec.setSetpoint(setpoint);
  }
  public void setPosition(double pos){
    elec.setPosition(pos);
  }
  public double getPos(){
    updateAnalogPosition();
    return elec.getPosition();
  }
  public double getDesiredPos(){
    return elec.getSetpoint();
  }
  public double getVel(){
    return elec.getVelocity();
  }
  public double getDesiredVel(){
    return elec.getVelocityCmd();
  }

  @Override
  public void periodic() {
    ntupdate();
    // This method will be called once per scheduler run
  }
  NetworkTable table = NetworkTableInstance.getDefault().getTable("elecServo");
    NetworkTableEntry nt_pos;
    NetworkTableEntry nt_cmdPos;
    NetworkTableEntry nt_vel;
    NetworkTableEntry nt_cmdVel;
    
    private void ntcreate(){
      nt_pos = table.getEntry("pos");
      nt_cmdPos = table.getEntry("cmdPos");
      nt_vel = table.getEntry("vel");
      nt_cmdVel = table.getEntry("cmdVel"); 
    }

    private void ntupdate(){
      nt_pos.setDouble(getPos());
      nt_cmdPos.setDouble(getDesiredPos());
      nt_vel.setDouble(getVel());
      nt_cmdVel.setDouble(getDesiredVel());
    }
  }
