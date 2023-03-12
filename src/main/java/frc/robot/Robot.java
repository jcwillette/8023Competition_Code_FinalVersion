// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

   //Auto Selection
   private static final String kNoAuto = "No Auto";
   private static final String kBalanceAuto = "Balance";
   private static final String kMobilityAuto = "Mobility";
   private String m_autoSelected;
   private final SendableChooser<String> m_chooser = new SendableChooser<>();

   //Drive Motors
   private CANSparkMax leftFrontMotor = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
   private CANSparkMax leftBackMotor = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
   private CANSparkMax rightFrontMotor = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
   private CANSparkMax rightBackMotor = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);

   MotorControllerGroup leftControllerGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
   MotorControllerGroup rightControllerGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

   DifferentialDrive drive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

  //Intake Motors
  private WPI_VictorSPX rollerMotor = new WPI_VictorSPX(5);
  private WPI_VictorSPX raisingMotor = new WPI_VictorSPX(6);

  //Sensors
  RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
  RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();

  private ADIS16470_IMU gyro = new ADIS16470_IMU();

  // Controllers
  private XboxController driveController = new XboxController(0);
  private XboxController intakeController = new XboxController(1);

  //Constants
  private final double encoder2inches = 1/8.46; //Encoder to distance conversion factor
  
  private final double driveSpeed = 0.90; //driving constants
  private final double driveTurn = 0.30;
  private final double slowMode = 0.5;

  private final double armSpeed = 0.60; //operating constants
  private final double armDeadband = 0.05; 
  private final double intakeSpeed = -0.7;
  private final double outtakeSpeed = 1;

  @Override
  public void robotInit() {

    //Auto Selection
    m_chooser.setDefaultOption("Mobility", kMobilityAuto);
    m_chooser.addOption("Balance", kBalanceAuto);
    m_chooser.addOption("No Auto", kNoAuto);
    SmartDashboard.putData("Auto Selection",m_chooser);

    //All motors to breakmode
    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftBackMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightBackMotor.setIdleMode(IdleMode.kBrake);
    raisingMotor.setNeutralMode(NeutralMode.Brake);
    rollerMotor.setNeutralMode(NeutralMode.Brake);

    //Make motors go in correct direction
    rightControllerGroup.setInverted(true);
    leftControllerGroup.setInverted(false);

    //Reset encoders
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    //calibrate gyro
    gyro.calibrate();
    gyro.reset();

  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {

    //All motors to breakmode
    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftBackMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightBackMotor.setIdleMode(IdleMode.kBrake);
    raisingMotor.setNeutralMode(NeutralMode.Brake);
    rollerMotor.setNeutralMode(NeutralMode.Brake);

    //choose auto
    m_autoSelected = m_chooser.getSelected();

    //Init state
    machineState = 1;
    // approach1 = 1
    // ascent1 = 2
    // descent = 3
    // mobilityDrive = 4
    // approach2 = 5
    // autoLevel = 6

    //Reset gyro
    gyro.reset();

    //Init Values
    startTime = Timer.getFPGATimestamp();

  }

  //pre-auto declarations
  public int machineState; //Auto State
  public double endStateEncoder; //Encoder Position for decent
  public double startTime; //Timer
  public double dT; //time difference between now and last loop
  public double pidError; //Error off setpoint
  public double errorRate; // rate of change of error
  double angleSetpoint = 0.00; //Target angle (balanced)
  double lastError = 0.00; //error from previous loop
  double lastTime = 0.00; //timestamp from previous loop
  double errorSum = 0.00; //sum of errors
  //PID constants
  final double kP = 0.00;
  final double kI = 0.00;
  final double kD = 0.00;
  final double iZone = 0.00;
  public double pidSpeed; //speed of motors during autolevel


  //AUTO TUNING LEVERS

  //state machine triggers
  final double ascentTriggerAngle = 5;
  final double descentTriggerAngle = -5;
  final double mobilityTriggerAngle = 5;
  final double mobilityDistance = 5;
  final double autoLevelTriggerAngle = -5;

  //State Machine Constants
  final Double ascentSpeed = 0.55;
  final Double descendSpeed = 0.3;

  //Other Constants
  final double ejectTime = 2;
  final Double ejectSpeedAuto = 0.7;
  final double driveAwayDist = 102;
  final Double driveAwaySpeed = 0.3;

  @Override
  public void autonomousPeriodic() {

    //check variables
    Double time = Timer.getFPGATimestamp();
    Double robotDisplacement = (leftEncoder.getPosition()+rightEncoder.getPosition())/2*encoder2inches;
    double currentAngle = gyro.getYComplementaryAngle();

    switch(m_autoSelected){
      case kNoAuto:

        //Do nothing!

        break;
      case kMobilityAuto:
        
        //Eject Cube
        if(time - startTime < ejectTime){
          rollerMotor.set(ejectSpeedAuto);
         }else{
          rollerMotor.set(0);
         }

         //Move Forward 102 inches
         if(time - startTime > ejectTime){
          if (robotDisplacement < driveAwayDist) {
              drive.tankDrive(driveAwaySpeed, driveAwaySpeed);
            } else {
              drive.tankDrive(0, 0);
            }
         }

        break;
      case kBalanceAuto:
        
         //State Machine
         switch(machineState){
          case 1:

            //Eject Cube
            if(time - startTime < ejectTime){
              rollerMotor.set(ejectSpeedAuto);
            }else{
              rollerMotor.set(0);
            }
            
            //Approach Station
            if(time - startTime > ejectTime){
              drive.tankDrive(0.55, 0.55);
             }

            //Check for pitch up
            if(currentAngle > ascentTriggerAngle){
              machineState = machineState+1;
            }

          break;
          case 2:

            //Ascend
            drive.tankDrive(ascentSpeed, ascentSpeed);

            //check for pitch down
              if(currentAngle<descentTriggerAngle){
                machineState=machineState+1;
              }

          break;
          case 3:

            //Descend
            drive.tankDrive(descendSpeed, descendSpeed);

            //check for level
            if(Math.abs(currentAngle)<mobilityTriggerAngle){
              endStateEncoder = robotDisplacement;
              machineState=machineState+1;
            }

          break;
          case 4:

            //Move Forward to clear community
            if (robotDisplacement < endStateEncoder + mobilityDistance) {
                drive.tankDrive(ascentSpeed, ascentSpeed);
            }else{
              machineState = machineState+1;
            }

          break;
          case 5:

            //Back up onto charging station
            drive.tankDrive(-ascentSpeed, -ascentSpeed);

            //check for pitch down
            if(currentAngle<autoLevelTriggerAngle){
                machineState=machineState+1;
            }

          break;
          case 6:

          //Calculate motor speed with PID
          
          pidError = angleSetpoint - currentAngle; //distance from target, used for kP term
          
          dT=time-lastTime; // Time between loops, used for kD term

          if(Math.abs(pidError)< iZone){  //checks if close enough to use integral
            errorSum +=pidError*dT; // time integral of errors, used for kI
          }

          errorRate = (pidError-lastError)/dT; //rate of change of error, used for kD term

          pidSpeed = kP*pidError + kI*errorSum + kD*errorRate;

          //drive to correct error
          drive.tankDrive(-pidSpeed, -pidSpeed);

          lastError=pidError;
          lastTime = time;

          break;
         }

      break;
    }

  }

  @Override
  public void teleopInit() {

    //All motors to breakmode
    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftBackMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightBackMotor.setIdleMode(IdleMode.kBrake);
    raisingMotor.setNeutralMode(NeutralMode.Brake);
    rollerMotor.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void teleopPeriodic() {

    //Driver inputs
    double speed = -driveController.getRawAxis(1)*driveSpeed;
    double turn = -driveController.getRawAxis(4)*driveTurn;

    //Operator Inputs
    double armInput = intakeController.getRawAxis(1)*armSpeed;

    //Driver Control
    if(driveController.getRightBumper()){ //Slow mode conditional
      drive.arcadeDrive(speed*slowMode, turn);
    }else{
      drive.arcadeDrive(speed, turn);
    }

    //operator control
    if(Math.abs(armInput) < armDeadband){  //Arm control
      raisingMotor.set(0);
    }else{
      raisingMotor.set(armInput);
    }

    if(intakeController.getYButton()){  //Roller control
      rollerMotor.set(ControlMode.PercentOutput, intakeSpeed);
    }else if (intakeController.getAButton()){
      rollerMotor.set(ControlMode.PercentOutput, outtakeSpeed);
    }

  }

  @Override
  public void disabledInit() {

    //All motors to breakmode
    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftBackMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightBackMotor.setIdleMode(IdleMode.kBrake);
    raisingMotor.setNeutralMode(NeutralMode.Brake);
    rollerMotor.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
