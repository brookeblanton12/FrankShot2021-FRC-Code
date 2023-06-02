// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Settings;

public class Shooter extends SubsystemBase {
  public enum COLOR{Red, Yellow, Blue, Green}
  private final WPI_TalonFX _shooterMotor1 = new WPI_TalonFX(Constants.ShooterConstants.shooterMotor1);
  private final WPI_TalonFX _shooterMotor2 = new WPI_TalonFX(Constants.ShooterConstants.shooterMotor2);
  private double _targetRPM;

  private double _kF = Constants.ShooterConstants.defaultkF;
  private double _kP = Constants.ShooterConstants.defaultkP;

  /** Creates a new Shooter. */
  public Shooter() {
    setDefaultCommand(new RunCommand(this::stop, this));
    _shooterMotor2.follow(_shooterMotor1);
    _shooterMotor1.setInverted(true);
    _shooterMotor2.setInverted(false);

    _shooterMotor1.configFactoryDefault();
    _shooterMotor1.config_kF(0, _kF);
    _shooterMotor1.config_kP(0, _kP);
    _shooterMotor1.setNeutralMode(NeutralMode.Coast);
  }

  public void setTargetRPM (double rpm) {
    _targetRPM = rpm;
  }

  public void setkF(double kF) {
    _kF = kF;
    _shooterMotor1.config_kF(0, _kF);
  }

  public void setkP(double kP) {
    _kP = kP;
    _shooterMotor1.config_kP(0, _kP);
  }

  public double getTargetRPM() {
    return _targetRPM;
  }

  public double getkF() {
    return _kF;
  }

  public double getkP() {
    return _kP;
  }

  public boolean isAtTargetRPM() {
    return true;//(Math.abs(_shooterMotor1.getClosedLoopError()) <= _targetRPM * .05);
  }

  public void shoot() {
    // _shooterMotor1.set(ControlMode.Velocity, _targetRPM);
    //TODO: velocity control mode not operating as expected; for now, treat the value as a percent output
    _shooterMotor1.set(_targetRPM);
  }

  public void stop() {
    _shooterMotor1.set(0);
  }

  public void setRedVelocity(double redVelocity){
    Constants.ShooterConstants.redVelocity = redVelocity;
  }

  public void setBlueVelocity(double blueVelocity){
    Constants.ShooterConstants.blueVelocity = blueVelocity;
  }

  public void setYellowVelocity(double yellowVelocity){
    Constants.ShooterConstants.yellowVelocity = yellowVelocity;
  }

  public void setGreenVelocity(double greenVelocity){
    Constants.ShooterConstants.greenVelocity = greenVelocity;
  }

  public double getRedVelocity(){
    return Constants.ShooterConstants.redVelocity;
  }

  public double getBlueVelocity(){
    return Constants.ShooterConstants.blueVelocity;
  }

  public double getYellowVelocity(){
    return Constants.ShooterConstants.yellowVelocity;
  }

  public double getGreenVelocity(){
    return Constants.ShooterConstants.greenVelocity;
  }

  public void setColor(COLOR color){
    switch(color){
      case Red: 
        _targetRPM = getRedVelocity();
        break;
      case Blue: 
        _targetRPM = getBlueVelocity();
        break; 
      case Yellow: 
        _targetRPM = getYellowVelocity();
        break;
      case Green:
        _targetRPM = getGreenVelocity();
        break;
    }
  }

  @Override
  public void periodic() {
    setkF(Settings.getLiveDouble("Shooter", "kF", Constants.ShooterConstants.defaultkF));
    setkP(Settings.getLiveDouble("Shooter", "kP", Constants.ShooterConstants.defaultkP));
    setRedVelocity(Settings.getLiveDouble("Shooter", "RedVelocity", Constants.ShooterConstants.redVelocity));
    setGreenVelocity(Settings.getLiveDouble("Shooter", "GreenVelocity", Constants.ShooterConstants.greenVelocity));
    setBlueVelocity(Settings.getLiveDouble("Shooter", "BlueVelocity", Constants.ShooterConstants.blueVelocity));
    setYellowVelocity(Settings.getLiveDouble("Shooter", "YellowVelocity", Constants.ShooterConstants.yellowVelocity));

    SmartDashboard.putNumber("Shooter.V", _shooterMotor1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter.T", _targetRPM);
    SmartDashboard.putBoolean("Shooter.AtV", isAtTargetRPM());
  }
}
