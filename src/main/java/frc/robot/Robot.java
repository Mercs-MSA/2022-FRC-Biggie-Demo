// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.*;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

//not sure if this is needed
import edu.wpi.first.wpilibj.AnalogInput; 
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;






/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  //private final Subsystems my_subsystem = new Subsystems();
  private final WPI_TalonFX shooter_motor1 = new WPI_TalonFX(7);
  private final WPI_TalonFX shooter_motor2 = new WPI_TalonFX(8);

  private final WPI_TalonFX driver_leftmotor1 = new WPI_TalonFX(2);
  private final WPI_TalonFX driver_leftmotor2 = new WPI_TalonFX(3);
  private final WPI_TalonFX driver_rightmotor1 = new WPI_TalonFX(4);
  private final WPI_TalonFX driver_rightmotor2 = new WPI_TalonFX(5);
  private final WPI_TalonFX climber_motor1 = new WPI_TalonFX(14);
  private final WPI_TalonFX climber_motor2 = new WPI_TalonFX(15);

  private final WPI_TalonFX intake_motor1 = new WPI_TalonFX(19);
  private final WPI_TalonFX conveyer1 = new WPI_TalonFX(20);
  
  //Joysticks
  private final Joystick driver_joystick = new Joystick(0);
  //private final Joystick copilot_joystick = new Joystick(1);
  DifferentialDrive tarzan_robot = new DifferentialDrive(driver_leftmotor1, driver_rightmotor1);

  // Creates UsbCamera
  UsbCamera driver_camera = new UsbCamera("USB Camera 0", 0);

  /// Setup the digital inputs
  private final DigitalInput conveyor_loc_1 = new DigitalInput(0);

  // Setup the pneumatics devices
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  Solenoid IntakeLeftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
  Solenoid IntakeRightSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);
  Solenoid TopLeftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 5);
  Solenoid TopRightSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 3);
  Solenoid BottomLeftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 4);
  Solenoid BottomRightSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);


  // Setup the color sensor
  private final ColorSensorV3 color_sensor = new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget = new Color(0.561, 0.100, 0.340);   // Dan adjusted these values based on measurements of the cargo

  private Robot_Cargo_State cargo_status = Robot_Cargo_State.Idle;
  private Intake_Deployment_State intake_status = Intake_Deployment_State.up;
  private Climber_State Climber_status = Climber_State.start;
  private final Timer state4_Timer = new Timer();
  private final Timer state2_Timer = new Timer();
  private double tx_angle;
  private double ty_angle = -1000.0; //target degrees above the center of the camera 

  private final double cameraPitch = 10; //degrees above horizon ||
  private final double pupilCameraHeight = 12.8; //inches above the ground ||
  private final double goalHeight = 104; //inches above the ground to the top of the goal
  private double distanceFromGoal = 0; //inches parallel from shooter to the center of the goal
  private final double goalRadius = 26.7716535; //inches 
  private final double pupilDistanceToShooter = -6; //inches, in relation to distance from goal ||
  private final double desiredDistanceFromGoal = 132; //inches, distance from the shooter to the center of goal (114.75in - 24in) ||
  
  @Override
  public void robotInit() {

    shooter_motor1.configFactoryDefault();
    shooter_motor2.configFactoryDefault();

    driver_leftmotor1.configFactoryDefault();
    driver_leftmotor2.configFactoryDefault();
    driver_rightmotor1.configFactoryDefault();
    driver_rightmotor2.configFactoryDefault();

    climber_motor1.configFactoryDefault();
    climber_motor2.configFactoryDefault();

    intake_motor1.configFactoryDefault();
    conveyer1.configFactoryDefault();

    //Mechanism Inversion settings 
    shooter_motor2.setInverted(true);
    driver_rightmotor1.setInverted(true);
    driver_rightmotor2.setInverted(true);
    climber_motor2.setInverted(true);
    conveyer1.setInverted(true);
    intake_motor1.setInverted(true);

    //Shooter and Driver Follow
    shooter_motor2.follow(shooter_motor1);
    driver_rightmotor2.follow(driver_rightmotor1);
    driver_leftmotor2.follow(driver_leftmotor1);

    //Climber Follow
    climber_motor2.follow(climber_motor1);

    // Shooter Control Loop Settings
    shooter_motor1.configNeutralDeadband(0.001);
		shooter_motor1.config_kP(0, 0.015, 30);
		shooter_motor1.config_kI(0, 0.000, 30);
		shooter_motor1.config_kD(0, 0, 30);
    shooter_motor1.config_kF(0, 2048/22000, 30);

    //Front camera one time setup
    CameraServer.startAutomaticCapture();

    //Setup color sensor
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);

    phCompressor.enableAnalog(115, 120);   //enableAnalog(cameraPitch, cameraPitch); correct or no?
    phCompressor.enabled();

  }

  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("RED", color_sensor.getRed());
    SmartDashboard.putNumber("BLUE", color_sensor.getBlue());
    SmartDashboard.putNumber("GREEN", color_sensor.getGreen());

  }
    
  @Override
  public void autonomousInit() {
    state4_Timer.start();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
    cargo_status = Robot_Cargo_State.Cargo_being_intaked;
  }

  @Override
  public void autonomousPeriodic() {
    moveIntakeUptoDown();
    tx_angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty_angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    if (camAngletoDistance(ty_angle) <= desiredDistanceFromGoal){
      tarzan_robot.tankDrive(-1*0.8, -1*0.8);
    }
    else if (camAngletoDistance(ty_angle) > desiredDistanceFromGoal){
      if (Math.abs(tx_angle) > 0.5){
        tarzan_robot.tankDrive(-1*tx_angle, 1*tx_angle);
      }
      else{
        tarzan_robot.tankDrive(0, 0);
        cargo_status = Robot_Cargo_State.Cargo_awaiting_shooter;
      }
    }
    if (cargo_status == Robot_Cargo_State.Cargo_awaiting_shooter){
      shooter_motor1.set(ControlMode.Velocity, 18000);
      if ((shooter_motor1.getSelectedSensorVelocity() >= 17500) && (shooter_motor1.getSelectedSensorVelocity() <= 18500)){
        cargo_status = Robot_Cargo_State.Cargo_being_shot;
      }
    }
    else if (cargo_status == Robot_Cargo_State.Cargo_being_shot)
    {
      state4_Timer.start();
      conveyer1.set(0.8); //running conveyer 
      if (state4_Timer.get() > 2){
        conveyer1.set(0);
        shooter_motor1.set(0);
        state4_Timer.stop();
        cargo_status = Robot_Cargo_State.Idle;
      }
    }
  }

  @Override
  public void teleopInit() {
    state4_Timer.start();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
  }

  @Override
  public void teleopPeriodic() {
        //Tank Drive
        tarzan_robot.tankDrive(-1*driver_joystick.getRawAxis(1), -1*driver_joystick.getRawAxis(5));

        //Climber
        //climber_motor1.set(0.5*driver_joystick.getRawAxis(5));

        //Intake (positive inputs intake a cargo)
        if (intake_status == Intake_Deployment_State.down){
          autoIntake();
        }
        //replaced by autoIntake()
        /*
        if(driver_joystick.getRawButton(5) == true){
          intake_motor1.set(-1);
        }
        else if(driver_joystick.getRawButton(5) == false){
          intake_motor1.set(driver_joystick.getRawAxis(2));
        }
        

        //Conveyor (positive inputs bring cargo in)
        if ((driver_joystick.getRawButton(4) == true) && (driver_joystick.getRawButton(2) == false)) {
          conveyer1.set(1);
        }
        else if((driver_joystick.getRawButton(4) == false) && driver_joystick.getRawButton(2) == true){
          conveyer1.set(-1);
        }
        else {
          conveyer1.set(0);
        }

        //Shooter (positive inputs shoot cargo out)
        shooter_motor1.set(driver_joystick.getRawAxis(3)*0.8);
       */ 

        // Read color sensor
        Color detectedColor = color_sensor.getColor();
        String colorString;
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        if (match.color == kBlueTarget) {
          colorString = "Blue";
        } else if (match.color == kRedTarget) {
          colorString = "Red";
        }
        else {
          colorString = "Unknown";
        }
        SmartDashboard.putString("color sensor output", colorString);
        SmartDashboard.putNumber("Timer", state4_Timer.get());

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        if (driver_joystick.getRawButton(7) && driver_joystick.getRawButton(8)){
          moveIntakeDowntoUp();
        }
        if (driver_joystick.getRawButton(7) && driver_joystick.getRawButton(3)){
          initiateMiddleRungClimb();
        }
        if (driver_joystick.getRawButton(7) && driver_joystick.getRawButton(1)){
          finalizeMiddleRungClimb();
        }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  // This subroutine performs a semi-autonomous intake and shooting process
  public void autoIntake() {
    if((cargo_status == Robot_Cargo_State.Idle) && (driver_joystick.getRawButton(6) == true)){
      cargo_status = Robot_Cargo_State.Cargo_being_intaked;
      state2_Timer.start();
    }

    if ((cargo_status == Robot_Cargo_State.Cargo_being_intaked) && (conveyor_loc_1.get() == true)) {
      if (driver_joystick.getRawButton(6) == true){
        state2_Timer.start();
      }
      }
      if (state2_Timer.get() > 4){
        intake_motor1.set(0);
        conveyer1.set(0);
        shooter_motor1.set(0);
        cargo_status = Robot_Cargo_State.Idle;
      
      intake_motor1.set(0.8); //running intake
      conveyer1.set(0.8); //running conveyer
      //shooter_motor1.set(1*0.8); //starting shooter at 80%
      shooter_motor1.set(ControlMode.Velocity, 18000);
    } 
    if ((cargo_status == Robot_Cargo_State.Cargo_being_intaked) && (conveyor_loc_1.get() == false)) {
      cargo_status = Robot_Cargo_State.Cargo_awaiting_shooter;
      intake_motor1.set(0); //stopping intake
      conveyer1.set(0); //stopping conveyer
    }
    else if (cargo_status == Robot_Cargo_State.Cargo_awaiting_shooter){
      if ((shooter_motor1.getSelectedSensorVelocity() >= 17500) && (shooter_motor1.getSelectedSensorVelocity() <= 18500)){
        cargo_status = Robot_Cargo_State.Cargo_being_shot;
      }
    }
    else if (cargo_status == Robot_Cargo_State.Cargo_being_shot)
    {
      state4_Timer.start();
      conveyer1.set(0.8); //running conveyer 
      if (state4_Timer.get() > 2){
        conveyer1.set(0);
        shooter_motor1.set(0);
        state4_Timer.stop();
        cargo_status = Robot_Cargo_State.Idle;
      }
    }  
  }

  // This is is a custom type used to track the state of Cargo intake and shooting
  enum Robot_Cargo_State {
    Idle,    // This state means that the robot has no cargo in it and all intake/conveyor/shooter motors are off
    Cargo_being_intaked,     // This state means that a cargo is in the process of being intaked, but still in transit
    Cargo_awaiting_shooter,    // This state means that a cargo is in the robot and awaiting to be shot out
    Cargo_being_shot,    // This state means that the cargo is being shot out
    Error   // This is an error state or condition
  }

  enum Intake_Deployment_State {
    up, 
    down
  }
  
  enum Climber_State {
    start, 
    part1ClimbMiddleRung,
    part2ClimbMiddleRung,
    part1ClimbTopRung,
    part2ClimbTopRung,
    part3ClimbTopRung,
    part1ClimbTraversal,
    part2ClimbTraversal,
    part3ClimbTraversal,
    part4ClimbTraversal
  }
  // This method converts a target pitch angle into an estimated robot distance away from the target
  double camAngletoDistance(double a2) {
    return ((goalHeight - pupilCameraHeight)/(Math.tan(Math.toRadians(cameraPitch + a2))) + pupilDistanceToShooter + goalRadius);
  }

  void moveIntakeUptoDown() {
    if (intake_status == Intake_Deployment_State.up) {
      intake_motor1.set(0);
      conveyer1.set(0);
      shooter_motor1.set(0);
      IntakeLeftSolenoid.set(true);
      IntakeRightSolenoid.set(true);
      intake_status = Intake_Deployment_State.down;
    }
  }
  void initiateMiddleRungClimb() {
    if ((Climber_status == Climber_State.start) && (intake_status == Intake_Deployment_State.up)) {
      climber_motor1.set(ControlMode.Position, 512);
      if ((climber_motor1.getSelectedSensorVelocity() >= 500) && (shooter_motor1.getSelectedSensorVelocity() <= 520)){
        TopLeftSolenoid.set(true);
        TopRightSolenoid.set(true);
        Climber_status = Climber_State.part1ClimbMiddleRung;
      }
    }
  }
  void finalizeMiddleRungClimb() {
    if (Climber_status == Climber_State.part1ClimbMiddleRung) {
      //magic happens
      Climber_status = Climber_State.part2ClimbMiddleRung;
    }
  }
  void moveIntakeDowntoUp() {
    if (intake_status == Intake_Deployment_State.down) {
      intake_motor1.set(0);
      conveyer1.set(0);
      shooter_motor1.set(0);
      IntakeLeftSolenoid.set(false);
      IntakeRightSolenoid.set(false);
      intake_status = Intake_Deployment_State.up;
    }  
  }
}


