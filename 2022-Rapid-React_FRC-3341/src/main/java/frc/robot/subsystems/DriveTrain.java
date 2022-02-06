// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase 
{
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX _leftDriveTalon;
  private final WPI_TalonSRX _rightDriveTalon;

  private final VictorSPX _leftVictor;
  private final VictorSPX _rightVictor;

  private AHRS navx = new AHRS(SPI.Port.kMXP);
  private double ticksToCm  = 80.0/10180.5; //will test constant later
  private final int ticksInOneRevolution = 4096; 
 
  private DifferentialDrive _diffDrive;

  public DriveTrain() 
  {
    _leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    _rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);

    _leftVictor = new VictorSPX(Constants.LeftDriveVictorPort);
    _rightVictor = new VictorSPX(Constants.RightDriveVictorPort); 

    _leftDriveTalon.setInverted(true);
    _rightDriveTalon.setInverted(false);

    _leftVictor.setInverted(InvertType.FollowMaster);
    _rightVictor.setInverted(InvertType.FollowMaster);

    _diffDrive = new DifferentialDrive(_leftDriveTalon, _rightDriveTalon);

    _leftDriveTalon.configFactoryDefault();
    _leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    _rightDriveTalon.configFactoryDefault();


    _rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    _diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double speed, double turn) {
    _diffDrive.arcadeDrive(speed, turn);
  }

  public void resetEncoders() {
    _leftDriveTalon.setSelectedSensorPosition(0,0,10);
    _rightDriveTalon.setSelectedSensorPosition(0,0,10);
  }

  public double getPosition() {
    return ((_leftDriveTalon.getSelectedSensorPosition() + _rightDriveTalon.getSelectedSensorPosition())/2) * (ticksToCm);
    //average distance of both left and right
  }

  public double getTicks() {
    return (_leftDriveTalon.getSelectedSensorPosition(0) + _rightDriveTalon.getSelectedSensorPosition(0)) / 2;
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }



  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
}
