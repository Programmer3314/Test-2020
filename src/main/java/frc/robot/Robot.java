/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import org.opencv.core.Rect;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionRunner;
//import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive myRobot;
  private Joystick xboxController;
  private CANPIDController leftPidController, rightPidController;
  private CANEncoder leftEncoder, rightEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;


  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public CANSparkMax spark1, spark2, spark3, spark4, spark5, spark6;

  //This is the GRIP Vision Variables
  private static final int IMG_WIDTH = 480;
  private static final int IMG_HEIGHT = 320;

  private VisionThread visionThread;
  private double centerX = 0.0;
  private VisionRunner visionRunner;

  private final Object imgLock = new Object();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    spark1 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    spark2 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    spark3 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    spark4 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    spark5 = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    spark6 = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

    spark1.restoreFactoryDefaults();
    spark2.restoreFactoryDefaults();
    spark3.restoreFactoryDefaults();
    spark4.restoreFactoryDefaults();
    spark5.restoreFactoryDefaults();
    spark6.restoreFactoryDefaults();

    spark4.setInverted(true);

    spark2.follow(spark1);
    spark3.follow(spark1);
    spark5.follow(spark4);
    spark6.follow(spark4);

    //myRobot = new DifferentialDrive(spark1, spark4);

    xboxController = new Joystick(5);

    leftPidController = spark1.getPIDController();
    rightPidController = spark4.getPIDController();

    leftEncoder = spark1.getEncoder();
    rightEncoder = spark4.getEncoder();


    kP = 5e-5;  
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    leftPidController.setP(kP);
    leftPidController.setI(kI);
    leftPidController.setD(kD);
    leftPidController.setIZone(kIz);
    leftPidController.setFF(kFF);
    leftPidController.setOutputRange(kMinOutput, kMaxOutput);

    rightPidController.setP(kP);
    rightPidController.setI(kI);
    rightPidController.setD(kD);
    rightPidController.setIZone(kIz);
    rightPidController.setFF(kFF);
    rightPidController.setOutputRange(kMinOutput, kMaxOutput);

    //GRIP vision
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

    //GripPipeline pipeline;

    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
         if(!pipeline.findBlobsOutput().empty()){
           synchronized(imgLock){
             centerX =  pipeline.findBlobsOutput().toArray()[0].pt.x;
           }
          }
      });
    //visionRunner.runForever();
      
    //  visionRunner
    // visionThread = new VisionThread(
    //   camera, new GripPipeline(), pipeline -> {
    //   if(!pipeline.findBlobsOutput().empty()){
    //     synchronized(imgLock){
    //       centerX = pipeline.findBlobsOutput().toArray()[0].pt.x;
    //     }
    //   }
    // });
    visionThread.start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //myRobot.tankDrive(xboxController.getRawAxis(1), -xboxController.getRawAxis(5));
  
    double leftSetPoint = (xboxController.getRawAxis(1) - (xboxController.getRawAxis(4) * 0.5)) * maxRPM;
    double rightSetPoint = (xboxController.getRawAxis(1) + xboxController.getRawAxis(4) * 0.5) * maxRPM;
    leftPidController.setReference(leftSetPoint, ControlType.kVelocity);
    rightPidController.setReference(rightSetPoint, ControlType.kVelocity);

    SmartDashboard.putNumber("Position", centerX);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
