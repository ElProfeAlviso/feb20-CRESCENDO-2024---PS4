// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// TITANIUM RAMS 5959, CHIMUELO
// by: Beatriz Marún




package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import java.util.Optional;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX; //esta es la buena, la otra tiene bug por favor (dejenos bailar) no la uses.
import edu.wpi.first.math.util.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


import edu.wpi.first.cameraserver.CameraServer;


//limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


//  =================
//       Red CAN
//  =================
//   0 · roboRIO
//   1 · PDP


//   2 · rearRight
//   3 · rearLeft
//   4 · frontRight
//   5 · frontLeft

//   6 · intakeLeft
//   7 · intakeRight

//   8 · shooterLeft
//   9 · shooterRight

//   10 · pivotLeft +sube
//   11 · pivotRight -sube

//   12 · climberLeft
//   13 · climberRight
//   ==================


//-100 posición para shooter


public class Robot extends TimedRobot {


  Timer cronos;


  PS4Controller control;
  GenericHID operador;


  AHRS navx;


  Optional<Alliance> ally;

  
  double tiempoMatch;


  //AUTÓNOMOS
 
  double rotatcionAutonomo; //lograr rotación durante autónomo
 
  // PRUEBAS
  double sega; //segundos para avanzar (pruebas)
  double segnavx; //segundo en el que activa la navx (prueba)
  double rotar;//segundos que rota (prueba)
 
 
  //CHASSIS
  WPI_VictorSPX rearRight;
  WPI_VictorSPX rearLeft;
  WPI_VictorSPX frontRight;
  WPI_VictorSPX frontLeft;
 
  MecanumDrive myRobot;
 
  // declarar variables de datos de movimiento.
  double des_x;
  double des_y;
  double rot;

  int angulosChassis;
 
  //PID PARA EL CHASSIS


  PIDController chassisPID; //cambio de nombre una vez empezando a agregar más pid


  static final double kP = 0.018;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kToleranceDegrees = 2.0f;
 
  //MECANISMOS
  CANSparkMax intakeRight, intakeLeft;
  CANSparkMax shooterRight, shooterLeft;

  CANSparkMax  climberRight, climberLeft;

  CANSparkMax pivotRight, pivotLeft;
  boolean pivotSwitchPID;
  double pivotPIDsetpoint;
  PIDController pivotPID;


  static final double pivotkP = 0.02;
  static final double pivotkI = 0.00;
  static final double pivotkD = 0.00;
  static final double pivotkToleranceDegrees = 2.0f;
 
 
  //DASHBOARD
  double chassisPote;
  boolean fod;


  double intakePote;
  double shooterPote;
  double shooterAmpPote;

  double pivotPote;
  double pivotLimiteInferior;
  double pivotLimiteSuperior; 

  double climberPote;
  double climberLimiteInferior;
  double climberLimiteSuperior;

  double pivotPotePID;


  // valores encoders
  double pivotRotacion;
  double climberRotacion;


  RelativeEncoder pivotEncoder;
  RelativeEncoder climberEncoder;


  //LIMELIGHT
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  // leds
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  int redLeds, bluLeds;
  double intervaloLeds = 0.25;
  double tiempoAnterior;
  boolean ledBlink = true;
  int m_rainbowFirstPixelHue;


  @Override
  public void robotInit() {// declarar todos los objetos


    cronos = new Timer();


    rearRight = new WPI_VictorSPX(2);
    rearLeft = new WPI_VictorSPX(3);
    frontRight = new WPI_VictorSPX(4);
    frontLeft = new WPI_VictorSPX(5);


    rearRight.setInverted(false); // dos ejes invertidos por sentido común (lo olvidamos ayer xd 18/09)
    rearLeft.setInverted(true);
    frontRight.setInverted(false); // puede que cambie pq lo copié tal cuál
    frontLeft.setInverted(true);


    rearLeft.setNeutralMode(NeutralMode.Coast);
    rearRight.setNeutralMode(NeutralMode.Coast);
    frontLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);


    myRobot = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
   
    intakeLeft = new CANSparkMax(6, MotorType.kBrushless);
    intakeRight = new CANSparkMax(7, MotorType.kBrushless);
    shooterLeft = new CANSparkMax(8, MotorType.kBrushless);
    shooterRight = new CANSparkMax(9, MotorType.kBrushless);
    pivotLeft = new CANSparkMax(10, MotorType.kBrushless);
    pivotRight = new CANSparkMax(11, MotorType.kBrushless);
    climberLeft = new CANSparkMax(12, MotorType.kBrushless);
    climberRight = new CANSparkMax(13, MotorType.kBrushless);


    //ENCODERS
    pivotEncoder = pivotRight.getEncoder(); //se obtiene encoder del motor controlador
    pivotEncoder.setPosition(0); //se declara que la posición inicie en 0
    // pivotEncoder.setInverted(false);
    climberEncoder = climberRight.getEncoder();
    climberEncoder.setPosition(0);


    //prueba de reemplazo de Motor group
    intakeLeft.follow(intakeRight, true); //se invierte para que los motores giren en un mismo sentido
    shooterLeft.follow(shooterRight);
    pivotLeft.follow(pivotRight,true);
    climberLeft.follow(climberRight, true);


    control = new PS4Controller(0);
    operador = new GenericHID(1);
   
    navx = new AHRS(SPI.Port.kMXP); // SPI es el puerto en el q se conecta la navx


    chassisPID = new PIDController(kP, kI, kD);
    chassisPID.enableContinuousInput(-180.0f, 180.0f);
    chassisPID.setIntegratorRange(-1.0, 1.0);
    chassisPID.setTolerance(kToleranceDegrees);
    chassisPID.isContinuousInputEnabled();


    pivotPID = new PIDController(pivotkP, pivotkI, pivotkD);
    pivotPID.setTolerance(pivotkToleranceDegrees);


    // una vez declarado todos los constructores y objetos, ahora se pueden mandar
    // datos al SmartDashboard (por conveniencia)
    SmartDashboard.putNumber("Potencia", 1);
    SmartDashboard.putNumber("Segundos rotar", 1);
    SmartDashboard.putNumber("Segundos navx", 1.1);
    SmartDashboard.putNumber("Potencia intake", 0.9);
    SmartDashboard.putNumber("Potencia shooter", 0.7); //0.3 para amp
    SmartDashboard.putNumber("Potencia shooter amp", 0.3);
    SmartDashboard.putNumber("Potencia pivot", 0.7);
    SmartDashboard.putNumber("Potencia Pivot PID", 0.7);
    SmartDashboard.putNumber("Potencia climber", 0.9);
    SmartDashboard.putNumber("rotar angulo", 45);
    SmartDashboard.putBoolean("FOD", true);
    SmartDashboard.putNumber("Rotacion pivot", 0);
    SmartDashboard.putNumber("Rotacion climber", 0);
    SmartDashboard.putNumber("Pivot Limite Superior", 35);
    SmartDashboard.putNumber("Pivot Limite Inferior", -155);
    SmartDashboard.putNumber("Climber Limite Superior", 225);
    SmartDashboard.putNumber("Climber Limite Inferior", 1);
    SmartDashboard.putNumber("Seleccion de Autonomo", 0);
   
    CameraServer.startAutomaticCapture();

    // leds
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(27);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

  }


  @Override
  public void robotPeriodic() {// lo que el robot ejecuta desde que enciende hasta que se apaga.
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);


    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);


    SmartDashboard.putNumber("Voltaje", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Angulo", navx.getYaw());
   
   }


  @Override
  public void autonomousInit() {

    cronos.start();
    cronos.reset();

    rearLeft.setNeutralMode(NeutralMode.Brake);
    rearRight.setNeutralMode(NeutralMode.Brake);
    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);


    navx.reset();
    
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.get() == Alliance.Red) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 0, 0);
      }
    }
    else {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 0, 255);
      }
    }
    m_led.setData(m_ledBuffer);

  }


  @Override
  public void autonomousPeriodic() {
//Variables del dashboard
    double autoSelec;
    int autoSelectInt;
    autoSelec = SmartDashboard.getNumber("Seleccion de Autonomo", 0);
    shooterPote = SmartDashboard.getNumber("Potencia shooter", 0);
    pivotPote = SmartDashboard.getNumber("Potencia pivot", 0);
    pivotPotePID = SmartDashboard.getNumber("Potencia Pivot PID", 0.5);
    pivotLimiteSuperior = SmartDashboard.getNumber("Pivot Limite Superior", 0);
    pivotLimiteInferior = SmartDashboard.getNumber("Pivot Limite Inferior", 0);
    intakePote = SmartDashboard.getNumber("Potencia intake", 0);
    autoSelectInt = (int)autoSelec;
    sega = SmartDashboard.getNumber("Segundos rotar", 0);
    segnavx = SmartDashboard.getNumber("Segundos navx", 0);
    rotar = SmartDashboard.getNumber("rotar angulo", 0);

    // valores encoders
    pivotRotacion = -pivotEncoder.getPosition(); //se obtiene valor del enconder, el negativo es para invertir la posición
    climberRotacion = -climberEncoder.getPosition();

    //DASHBOARD
    SmartDashboard.putNumber("Rotacion pivot", pivotRotacion); //posteriormente se pone en el dashboard para poder ver en que posición está
    SmartDashboard.putNumber("Rotacion climber", climberRotacion);

  switch (autoSelectInt) {
    //Autonomo que no hace nada solo para no usar el cero en el dashboard.
    case 0:
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      break;
//Autonomo de lado azul izquierdo
      case 1:

 if (cronos.get()<=2) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;
    }
    else if (cronos.get()<=3) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.set(shooterPote);
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;

    }
    else if (cronos.get()<=4.5) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      intakeRight.set(intakePote);
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;
    }

    else if (cronos.get()<=5.5){//un segundo para avanzar
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0.3, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.stopMotor();
      intakeRight.stopMotor();
    }
    else if (cronos.get()<=7) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),65);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
    }
    else if (cronos.get()<8) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),65);
      myRobot.driveCartesian(0, 0.5, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
    }
       else if (cronos.get()<9) {
     myRobot.stopMotor();
    }
    

     if (pivotRotacion >= pivotLimiteSuperior){ //si el angulo del pivot es mayor que el limite superior, se hace una "abrazadera" que sólo lo permite ir hacia el lado contrario
    pivotPote = MathUtil.clamp(pivotPote, 0, 1);
    } 
    else if (pivotRotacion <= pivotLimiteInferior){
    pivotPote = MathUtil.clamp(pivotPote, -1, 0);
    }

    if(pivotSwitchPID){
    pivotPote = -pivotPID.calculate(pivotRotacion, pivotPIDsetpoint);
    pivotPote = MathUtil.clamp(pivotPote, -pivotPotePID, pivotPotePID);
    } 
    pivotRight.set(pivotPote);

      break;
//Autonomo de lado azul al centro

      case 2:

if (cronos.get()<=2) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;
    }
    else if (cronos.get()<=3) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.set(shooterPote);
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;
    }
    else if (cronos.get()<=4.5) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      intakeRight.set(intakePote);
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;
    }
    else if (cronos.get()<=5.5){//un segundo para avanzar
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0.5, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.stopMotor();
      intakeRight.stopMotor();
    }
    else if (cronos.get()<=6) {
      myRobot.stopMotor();
    }


     if (pivotRotacion >= pivotLimiteSuperior){ //si el angulo del pivot es mayor que el limite superior, se hace una "abrazadera" que sólo lo permite ir hacia el lado contrario
    pivotPote = MathUtil.clamp(pivotPote, 0, 1);
    } 
    else if (pivotRotacion <= pivotLimiteInferior){
    pivotPote = MathUtil.clamp(pivotPote, -1, 0);
    }

    if(pivotSwitchPID){
    pivotPote = -pivotPID.calculate(pivotRotacion, pivotPIDsetpoint);
    pivotPote = MathUtil.clamp(pivotPote, -pivotPotePID, pivotPotePID);
    } 
    pivotRight.set(pivotPote);
//Autonomo de lado azul 

    break;

      case 3:

if (cronos.get()<=2) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;
    }
    else if (cronos.get()<=3) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.set(shooterPote);
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;

    }
    else if (cronos.get()<=4.5) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      intakeRight.set(intakePote);
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;
    }

    else if (cronos.get()<=6.5){//un segundo para avanzar
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0.4, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.stopMotor();
      intakeRight.stopMotor();
    }
    else if (cronos.get()<=8) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),-65);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
    }
    else if (cronos.get()<9) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),-65);
      myRobot.driveCartesian(0, -0.5, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
    }
       else if (cronos.get()<10) {
     myRobot.stopMotor();
    }
    

     if (pivotRotacion >= pivotLimiteSuperior){ //si el angulo del pivot es mayor que el limite superior, se hace una "abrazadera" que sólo lo permite ir hacia el lado contrario
    pivotPote = MathUtil.clamp(pivotPote, 0, 1);
    } 
    else if (pivotRotacion <= pivotLimiteInferior){
    pivotPote = MathUtil.clamp(pivotPote, -1, 0);
    }

    if(pivotSwitchPID){
    pivotPote = -pivotPID.calculate(pivotRotacion, pivotPIDsetpoint);
    pivotPote = MathUtil.clamp(pivotPote, -pivotPotePID, pivotPotePID);
    } 
    pivotRight.set(pivotPote);

      break;
        case 4:

        if (cronos.get()<=2) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;
    }
    else if (cronos.get()<=3) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.set(shooterPote);
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;

    }
    else if (cronos.get()<=4.5) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      intakeRight.set(intakePote);
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;
    }

    else if (cronos.get()<=6.5){//un segundo para avanzar
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0.4, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.stopMotor();
      intakeRight.stopMotor();
    }
    else if (cronos.get()<=8) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),65);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
    }
    else if (cronos.get()<9) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),65);
      myRobot.driveCartesian(0, 0.5, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
    }
       else if (cronos.get()<10) {
     myRobot.stopMotor();
    }
    

     if (pivotRotacion >= pivotLimiteSuperior){ //si el angulo del pivot es mayor que el limite superior, se hace una "abrazadera" que sólo lo permite ir hacia el lado contrario
    pivotPote = MathUtil.clamp(pivotPote, 0, 1);
    } 
    else if (pivotRotacion <= pivotLimiteInferior){
    pivotPote = MathUtil.clamp(pivotPote, -1, 0);
    }

    if(pivotSwitchPID){
    pivotPote = -pivotPID.calculate(pivotRotacion, pivotPIDsetpoint);
    pivotPote = MathUtil.clamp(pivotPote, -pivotPotePID, pivotPotePID);
    } 
    pivotRight.set(pivotPote);

        break;

        case 5:

        if (cronos.get()<=2) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;
    }
    else if (cronos.get()<=3) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.set(shooterPote);
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;

    }
    else if (cronos.get()<=4.5) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      intakeRight.set(intakePote);
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;
    }
    else if (cronos.get()<=5.5){//un segundo para avanzar
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0.5, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.stopMotor();
      intakeRight.stopMotor();
    }
    else if (cronos.get()<=6) {
      myRobot.stopMotor();
    }


     if (pivotRotacion >= pivotLimiteSuperior){ //si el angulo del pivot es mayor que el limite superior, se hace una "abrazadera" que sólo lo permite ir hacia el lado contrario
    pivotPote = MathUtil.clamp(pivotPote, 0, 1);
    } 
    else if (pivotRotacion <= pivotLimiteInferior){
    pivotPote = MathUtil.clamp(pivotPote, -1, 0);
    }

    if(pivotSwitchPID){
    pivotPote = -pivotPID.calculate(pivotRotacion, pivotPIDsetpoint);
    pivotPote = MathUtil.clamp(pivotPote, -pivotPotePID, pivotPotePID);
    } 
    pivotRight.set(pivotPote);

        break;
        case 6:

        if (cronos.get()<=2) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;
    }
    else if (cronos.get()<=3) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.set(shooterPote);
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;

    }
    else if (cronos.get()<=4.5) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      intakeRight.set(intakePote);
      pivotPIDsetpoint = -92;
      pivotSwitchPID = true;
    }

    else if (cronos.get()<=5.5){//un segundo para avanzar
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0.3, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.stopMotor();
      intakeRight.stopMotor();
    }
    else if (cronos.get()<=7) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),-65);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
    }
    else if (cronos.get()<8) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),-65);
      myRobot.driveCartesian(0, -0.5, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
    }
       else if (cronos.get()<9) {
     myRobot.stopMotor();
    }
    

     if (pivotRotacion >= pivotLimiteSuperior){ //si el angulo del pivot es mayor que el limite superior, se hace una "abrazadera" que sólo lo permite ir hacia el lado contrario
    pivotPote = MathUtil.clamp(pivotPote, 0, 1);
    } 
    else if (pivotRotacion <= pivotLimiteInferior){
    pivotPote = MathUtil.clamp(pivotPote, -1, 0);
    }

    if(pivotSwitchPID){
    pivotPote = -pivotPID.calculate(pivotRotacion, pivotPIDsetpoint);
    pivotPote = MathUtil.clamp(pivotPote, -pivotPotePID, pivotPotePID);
    } 
    pivotRight.set(pivotPote);

        break;

    case 7:

    //   if (cronos.get()<=2) {
    //   rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
    //   myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
    //   pivotPIDsetpoint = pivotLimiteSuperior;
    //   pivotSwitchPID = true;
    // }
    // else if (cronos.get()<=3) {
    //   rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
    //   myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
    //   shooterRight.set(shooterPote);
    //   pivotPIDsetpoint = -100;
    //   pivotSwitchPID = true;
    // }
    // else if (cronos.get()<=4.5) {
    //   rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
    //   myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
    //   intakeRight.set(intakePote);
    //   pivotPIDsetpoint = -100;
    //   pivotSwitchPID = true;
    // }
    // else if (cronos.get()<=5.5){//un segundo para avanzar
    //   rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
    //   myRobot.driveCartesian(0.5, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
    //   shooterRight.stopMotor();
    //   intakeRight.stopMotor();
    // }
    // else if (cronos.get()<=6) {
    //   myRobot.stopMotor();
    // }


    //  if (pivotRotacion >= pivotLimiteSuperior){ //si el angulo del pivot es mayor que el limite superior, se hace una "abrazadera" que sólo lo permite ir hacia el lado contrario
    // pivotPote = MathUtil.clamp(pivotPote, 0, 1);
    // } 
    // else if (pivotRotacion <= pivotLimiteInferior){
    // pivotPote = MathUtil.clamp(pivotPote, -1, 0);
    // }

    // if(pivotSwitchPID){
    // pivotPote = -pivotPID.calculate(pivotRotacion, pivotPIDsetpoint);
    // pivotPote = MathUtil.clamp(pivotPote, -pivotPotePID, pivotPotePID);
    // } 
    // pivotRight.set(pivotPote);
    break;

      default:
      break;
      }
   }
  

    
    


  @Override
  public void teleopInit() {

    rearLeft.setNeutralMode(NeutralMode.Brake);
    rearRight.setNeutralMode(NeutralMode.Brake);
    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);

    pivotSwitchPID = false;

    ally = DriverStation.getAlliance(); //obtener alianza

    if (ally.get() == Alliance.Red) { //seleccionar colores
      redLeds = 255;
      bluLeds = 0;
    }
    else {
      redLeds = 0;
      bluLeds = 255;
    }

    for (var i = 0; i < m_ledBuffer.getLength(); i++) { //mandar colores
      m_ledBuffer.setRGB(i, redLeds, 0, bluLeds);
    }

    m_led.setData(m_ledBuffer);

    tiempoAnterior = 0; //resetear timer parpadeo leds

  }


  @Override
  public void teleopPeriodic() {
    // se ejecuta constantemente en modo teleoperado
    // que la primera parte sea de recolección de datos (actualizar
    // datos de variables), clasificando estas.

    //CHASSIS
    chassisPote = SmartDashboard.getNumber("Potencia", 0);
    fod = SmartDashboard.getBoolean("FOD", fod);

    //MECANISMOS
    intakePote = SmartDashboard.getNumber("Potencia intake", 0);
    shooterPote = SmartDashboard.getNumber("Potencia shooter", 0);
    shooterAmpPote = SmartDashboard.getNumber("Potencia shooter amp", 0);
    pivotPote = SmartDashboard.getNumber("Potencia pivot", 0);
    climberPote = SmartDashboard.getNumber("Potencia climber", 0);
    pivotPotePID = SmartDashboard.getNumber("Potencia Pivot PID", 0.5);
    pivotLimiteSuperior = SmartDashboard.getNumber("Pivot Limite Superior", 0);
    pivotLimiteInferior = SmartDashboard.getNumber("Pivot Limite Inferior", 0);
    climberLimiteSuperior = SmartDashboard.getNumber("Climber Limite Superior", 0);
    climberLimiteInferior = SmartDashboard.getNumber("Climber Limite Inferior", 0);
   
    //valores desde el control
    des_x = -control.getLeftY() * chassisPote; //de default viene multiplicado por la potencia
    des_y = control.getLeftX() * chassisPote;
    rot = -control.getRightX() * chassisPote;
    angulosChassis = control.getPOV();

    // valores encoders
    pivotRotacion = -pivotEncoder.getPosition(); //se obtiene valor del enconder, el negativo es para invertir la posición
    climberRotacion = -climberEncoder.getPosition();

    //DASHBOARD
    SmartDashboard.putNumber("Rotacion pivot", pivotRotacion); //posteriormente se pone en el dashboard para poder ver en que posición está
    SmartDashboard.putNumber("Rotacion climber", climberRotacion);


//SETPOINT MANEJO

    // if (control.getR3Button()) {
    //   rot = chassisPID.calculate(navx.getYaw(), 0);
    // }


    //FOD
    if (fod) {
         
      //POV
        if (angulosChassis != -1) { //si el POV es diferente a -1, está presionado
          if (angulosChassis > 180) {
            angulosChassis = angulosChassis-360; //le restamos 360 para obtener un ángulo coterminal admitible por el PID
        }
          rot = -chassisPID.calculate(navx.getYaw(), angulosChassis);
        }

        myRobot.driveCartesian(des_x, des_y, rot, Rotation2d.fromDegrees(navx.getAngle())); //manejo con navx
       
    } else {

      myRobot.driveCartesian(des_x, des_y, rot); //manejo normal

    }


//RESETEO DE NAVX

    if (control.getOptionsButtonPressed()) { 
      navx.reset(); 
    }

// INTAKE

  if (control.getR2Button() || operador.getRawAxis(3) > 0.5){ // Succionar
     intakePote = intakePote;
    }
  else if
    (control.getL2Button() || operador.getRawAxis(2) > 0.5){ // Regresar
      intakePote = -intakePote;
    }
  else {
      intakePote = 0;
    }
    intakeRight.set(intakePote); //solo se declara una vez el "set", en el if lo que cambia es la variable de potencia

//SHOOTER

  if (pivotRotacion > 0) {
      shooterPote = shooterAmpPote;
    }
  else{
    shooterPote = shooterPote;
    }

  if (control.getSquareButton() && control.getL3Button()==true || operador.getRawButton(3) && operador.getRawButton(9)==true){ //
      shooterPote = -shooterPote;
    } 
  else if (control.getSquareButton()&& control.getL3Button()==false || operador.getRawButton(3)&& operador.getRawButton(9)==false){
      shooterPote = shooterPote;
    }
  else {
      shooterPote = 0;
    }
  
  //Shooter automático
 

    shooterRight.set(shooterPote);
    

  //PIVOT

  if ((control.getCrossButtonPressed() || operador.getRawButtonPressed(1))) {
    pivotPIDsetpoint = pivotLimiteInferior;
    pivotSwitchPID = true;
    }
  if (control.getPSButtonPressed() || operador.getRawButtonPressed(8)) {
    pivotPIDsetpoint = 0;
    pivotSwitchPID = true;
    }
  if (control.getCircleButton() || operador.getRawButtonPressed(2)) {
    pivotPIDsetpoint = -92;
    pivotSwitchPID = true;
    }
  if (control.getTriangleButton() || operador.getRawButtonPressed(4)) {
    pivotPIDsetpoint = pivotLimiteSuperior;
    pivotSwitchPID = true;
    }
   
  if (control.getR1Button() || operador.getRawButton(6)){ //SUBE
    pivotPote = -pivotPote;
    pivotSwitchPID = false;
    }  
  else if (control.getL1Button() || operador.getRawButton(5)) { //BAJA
    pivotPote = pivotPote; //no es necesario cambiarla
    pivotSwitchPID = false;
    } 
  else {
    pivotPote = 0;
    }

  if (pivotRotacion >= pivotLimiteSuperior){ //si el angulo del pivot es mayor que el limite superior, se hace una "abrazadera" que sólo lo permite ir hacia el lado contrario
    pivotPote = MathUtil.clamp(pivotPote, 0, 1);
    } 
  else if (pivotRotacion <= pivotLimiteInferior){
    pivotPote = MathUtil.clamp(pivotPote, -1, 0);
    }

  if(pivotSwitchPID){
    pivotPote = -pivotPID.calculate(pivotRotacion, pivotPIDsetpoint);
    pivotPote = MathUtil.clamp(pivotPote, -pivotPotePID, pivotPotePID);
    } 

  pivotRight.set(pivotPote);

  //CLIMBER
 
  if (operador.getRawAxis(1) < -0.5  && operador.getRawAxis(5) < -0.5) {
    climberPote = -climberPote;
  } 
  else if (operador.getRawAxis(1) > 0.5 && operador.getRawAxis(5) > 0.5){
    climberPote = climberPote;
  } 
  else{
    climberPote = 0;
  }
  
  if (climberRotacion >= climberLimiteSuperior){
    climberPote = MathUtil.clamp(climberPote, 0, 1);
  } 
  else if (climberRotacion <= climberLimiteInferior){
    climberPote = MathUtil.clamp(climberPote, -1, 0);
  }

  climberRight.set(climberPote);

  // climberRight.set(climberPote);

  //   pivotRight.set(-pivotPote);
  //   }
  // else if (control.getL1Button()){ //
  //   pivotRight.set(pivotPote);
  // }
  // else {
  //   pivotRight.stopMotor();
  // }


  tiempoMatch = DriverStation.getMatchTime();
  // System.out.println(tiempoMatch);

  // leds
  
  if (tiempoMatch < 10 && tiempoMatch != -1) {
     if ((999 - tiempoMatch) - tiempoAnterior >= intervaloLeds) {
      tiempoAnterior = 999 - tiempoMatch;
      if (ledBlink) {
        ledBlink = false;

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, 0, 0, 0);
        }
      }
      else {
        ledBlink = true;

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, 0, 255, 0);
        }
      }
  }

  else if (tiempoMatch < 20 && tiempoMatch != -1) { // menos de 10 seg, verde
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 255, 0);
    }
    m_led.setData(m_ledBuffer);
  }
  else if (tiempoMatch < 40 && tiempoMatch != -1) { // menos de 30 seg, parpadea
    
    if ((999 - tiempoMatch) - tiempoAnterior >= intervaloLeds) {
      tiempoAnterior = 999 - tiempoMatch;
      if (ledBlink) {
        ledBlink = false;

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, 0, 0, 0);
        }
      }
      else {
        ledBlink = true;

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, redLeds, 0, bluLeds);
        }
      }
      
    }

    m_led.setData(m_ledBuffer);
  }
}
  

  

  }


  @Override
  public void disabledInit() {
    rearLeft.setNeutralMode(NeutralMode.Coast);
    rearRight.setNeutralMode(NeutralMode.Coast);
    frontLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
  }


  @Override
  public void disabledPeriodic() {

    // arcoiris led
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;

    m_led.setData(m_ledBuffer);
  }


  @Override
  public void testInit() {}


  @Override
  public void testPeriodic() {}


}


