// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Driver;

public class DrivePID extends CommandBase {
  /** Creates a new DrivePID. */
  /*
  summery:
  force = error * kp + I(integral-erorrSum) * ki + D(derivative- eroorRate) * kd
                present        past      futere 
  */

    private Driver driver;               // the subSystem Driver 
    private double error;               // how far are you from target 
    private double outPutMotor;       // force going out 
    private double setPoint;         // distance to target 
    private double current;         // hoe much distance you alrady went throw 
  

    // for the integral  
  private double errorSum; // the integral (past - until now )-- (width-dt * lengh-erorr) +  (width * lengh) .........
  private double dt ;     //  now -the time from last time i checked  - the with of each sqewre       
  private double lastTime;    // last time i checked 

  // for the derivative
  
  private double lastError ;    // you need in order to calc delta erorr
  private double errorRate ;   //   the derivative


  private double kp;    //  should be final 
  private double ki ;  //  should be final
  private double kd;  //  should be fina 


  public DrivePID(Driver driver) {
    this.driver = driver;
    addRequirements(driver);      // prevents multiple commands running on same subsystem 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {//* only - hapends once  here you need to define vars that do not change acirding to time
    
    //

    error = 0;          // not requaired  because it  is always changes 
    current = 0;       // not requaired  because it  is always changes
    kp =0.5;          // requaired it will  affect the formula later and here we can mess with the numbers 
    setPoint =0;     // requaired  because the target will not change 

    // for integral

    errorSum= 0;                          // requaired for calc 
    dt = 0;                              // requaired for calc - ask 
    lastTime= Timer.getFPGATimestamp(); // requaired in order to know the time
    ki =0.5;                           // requaired it will  affect the formula later and here we can mess with the numbers




    //
    
    errorRate =0;  //not requaired  because it  is always changes
    lastError=0;  //requaired for clac actions
    kd =0.5;     // requaired it will  affect the formula later and here we can mess with the numbers



  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {//*  hapends all the time   here you need to define vars that change acording to time  to time 

    // belongs tp every one 

    current = driver.d_getEncoderValue();       //   gets the value from the encoder throw the subSystem  there you have a func that can do it
    error = setPoint- current;                 // how mach distance from target = distance to target -  how much distance you wenr throw

    dt = Timer.getFPGATimestamp() - lastTime; // delta(difrence) = this current time - lest time we checked 
    errorSum += dt * error; // integral = sum sequers += width (dt) * length(error)

    errorRate = (error -lastError) / dt ; // find the derivative = delta (difrence)  /  

    outPutMotor = error * kp + errorSum * ki + errorRate * kd;       //  PID -calc of  force 
    
    driver.d_control(outPutMotor, 0);     // gives the force 

    lastTime =Timer.getFPGATimestamp() ; // up date
    lastError = error;  // upDate

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { // need to end the action of motor so give zero force 
    driver.d_control(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (setPoint == current){
      return true;      // go to end 
    }
    return false;       // keep execute
  }
}
