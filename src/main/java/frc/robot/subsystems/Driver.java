// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Driver extends SubsystemBase {
  /** Creates a new Driver. */
   // Motors

   private WPI_TalonFX m_leftLeader;
   private WPI_TalonFX m_leftFollower;
   private WPI_TalonFX m_rightLeader;
   private WPI_TalonFX m_rightFollower;

   //for the controllers to work together 
   private MotorControllerGroup rightGroup;
   private MotorControllerGroup leftGroup;
   private DifferentialDrive diffDrive;

   

  public Driver() {

/*
create new objects for motors,sensors 
*/

    this.m_leftLeader = new WPI_TalonFX(0);   //port(0)
    this.m_leftFollower = new WPI_TalonFX(0);
    this.m_rightLeader = new WPI_TalonFX(0);
    this.m_rightFollower = new WPI_TalonFX(0);

    this.leftGroup = new MotorControllerGroup(m_leftLeader, m_leftFollower);
    this.rightGroup = new MotorControllerGroup(m_rightLeader, m_rightFollower);

    this.diffDrive = new DifferentialDrive(leftGroup, rightGroup);
  }
  
  /** 
  *moves robot by Arcade drive
  *@poram speed robot's speed
  *@poram rottation robot's rotatation 
  */

  public void d_control (double speed , double rotatation){
    /*
    gets speed and rotatation and set data in the differantialDrive object 
    */

    this.diffDrive.arcadeDrive(speed, rotatation);
  }

  public double d_getEncoderValue(){
    
      return (this.m_leftLeader.getSelectedSensorPosition());

      // how many ticks in a raund (aka "distance")
  }
  





  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

 
  



}
