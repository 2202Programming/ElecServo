// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElecServo extends SubsystemBase {
  private SparkAnalogSensor m_analogSensor;
  private SparkPIDController m_pidController;
  private static final int deviceID = 18;
  private CANSparkMax m_motor;
  public double kP;
  public double kI;
  public double kD;
  public double kIz;
  public double kFF;
  public double kMaxOutput;
  public double kMinOutput;

  public ElecServo() {
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_analogSensor = m_motor.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
    m_motor.restoreFactoryDefaults();
    m_pidController = m_motor.getPIDController();
    m_pidController.setFeedbackDevice(m_analogSensor);
    kP = 0.1;
    kI = 0;
    kD = 0;
    kIz = 0.0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    SmartDashboard.putNumber("P Gain", getkP());
    SmartDashboard.putNumber("I Gain", getkI());
    SmartDashboard.putNumber("D Gain", getkD());
    SmartDashboard.putNumber("I Zone", getkIz());
    SmartDashboard.putNumber("Feed Forward", getkFF());
    // SmartDashboard.putNumber("Max Output", servo.);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0.0);
    SmartDashboard.putNumber("Current Pos", getPosition());
    SmartDashboard.putNumber("Current Vel", getVelocity());
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Current Pos", getPosition());
    SmartDashboard.putNumber("Current Vel", getVelocity());
  }

  public double getPosition(){
    return m_analogSensor.getPosition();
  }
  public double getkP() {
    return m_pidController.getP();
  }
  public double getkI(){
    return m_pidController.getI();
  }
  public double getkD(){
    return m_pidController.getD();
  }
  public double getkIz(){
    return m_pidController.getIZone();
  }
  public double getkFF(){
    return m_pidController.getFF();
  }

  public void setkP(double kP) {
    m_pidController.setP(kP);
  }
  public void setkI(double kI){
    m_pidController.setI(kI);
  }
  public void setkD(double kD){
    m_pidController.setD(kD);
  }
  public void setkIz(double kIz){
    m_pidController.setIZone(kIz);
  }
  public void setkFF(double kFF){
    m_pidController.setFF(kFF);
  }
  public double getVelocity(){
    return m_analogSensor.getVelocity();
  }

  public void setRef(double speed){
    m_pidController.setReference(speed, ControlType.kVelocity);
  }
}