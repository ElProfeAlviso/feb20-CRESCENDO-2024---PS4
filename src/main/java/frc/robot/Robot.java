/*  Copyright (c) FIRST and other WPILib contributors.
    Open Source Software; you can modify and/or share it under the terms of
    the WPILib BSD license file in the root directory of this project.

    TITANIUM RAMS 5959, FRC CHIMUELO 2024
    Original Code by: Beatriz Marún
    Mentor: Sebastian Leon.
    Sensors Update: Profe Alviso
*/

package frc.robot; //Paquete principal de directorios de archivos del proyecto.

import edu.wpi.first.wpilibj.TimedRobot; //Framework de un robot tipo Iterativo  por tiempo de ejecucion de 20ms.
import edu.wpi.first.wpilibj.drive.MecanumDrive; //Libreria de control de un chasis tipo mecanum Drive.
import edu.wpi.first.wpilibj.RobotController; //Libreria de obtencion de datos del controlador roborio.
import edu.wpi.first.wpilibj.DriverStation; //Libreria para obtener datos del software de Driver Station.
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser; //Libreria para hacer un menu de seleccion de autonomo
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //Libreria para enviar y recibir datos desde y hacia el Shuffleboard.
import edu.wpi.first.wpilibj.Timer; //Libreria para crear timers (Cronometros)
import edu.wpi.first.wpilibj.DriverStation.Alliance; //Libreria para obtener el color de alianza desde la cancha.
import edu.wpi.first.wpilibj.PS4Controller; //Libreria para usar un control de PS4
import edu.wpi.first.wpilibj.AddressableLED; //Libreria para controlar tiras led programables Neopixel.
import edu.wpi.first.wpilibj.AddressableLEDBuffer; //Libreria para darle longitud de leds de la tira neopixel.
import edu.wpi.first.wpilibj.GenericHID; //Libreria para usar un control Generico Logitech.
import edu.wpi.first.math.geometry.Rotation2d; //Libreria para convertir un angulo en coordenadas X y Y.
import edu.wpi.first.math.MathUtil; //Libreria para usar herramientas de matematicas.
import edu.wpi.first.math.controller.PIDController; //Libreria para usar controladores PID.

import java.util.Optional; //libreria para alternar entre un dato presente y otro que no o null.
import com.ctre.phoenix.motorcontrol.NeutralMode; //Libreria para activar el modo Brake
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX; //Libreria para utilizar controladores Victor SPX y motores CIM
import com.revrobotics.CANSparkMax; //Libreria para usar controladores SPARK con motores NEO.
import com.revrobotics.RelativeEncoder; //Libreria para obtener el valor del encoder realativo de los NEO.
import com.revrobotics.CANSparkLowLevel.MotorType; //Libreria para indicar el tipo de motor conectado a los sparks.
import com.kauailabs.navx.frc.AHRS; //Libreria para obtener los datos del gyroscopio NAVX.
import edu.wpi.first.wpilibj.SPI; //Libreria para conectar el gyroscopio en el puerto MXP SPI del roborio.

//Librerias para sensores 
import edu.wpi.first.wpilibj.DigitalInput; //Libreria para usar entradas digitales.
import edu.wpi.first.wpilibj.DutyCycleEncoder; //Libreria para encoder absoluto.
import edu.wpi.first.cameraserver.CameraServer; //Libreria para enviar la imagen de la web cam al dashboard.
import edu.wpi.first.cscore.UsbCamera; //Libreria para crear un objeto de camara USB y modificar parametros de imagen.
//limelight Librerias para obtener datos de posicion de la camara Lime Light.
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/*===================================================================================================
 =================
 Red CAN (ID . Dispositivo)
 =================
  0 · roboRIO
  1 · PDP

  2 · rearRight
  3 · rearLeft
  4 · frontRight
  5 · frontLeft

  6 · intakeLeft
  7 · intakeRight

  8 · shooterLeft
  9 · shooterRight

  10 · pivotLeft +sube
  11 · pivotRight -sube

  12 · climberLeft
  13 · climberRight
  ===================================================================================================
  ============================
  SENSORES DIGITALES
  ============================
    Pivot Abajo DIO 0
    Pivot Arriba DIO 1
    Nota detectada DIO 2
    Encoder Absoluto DIO 3
    
*/

public class Robot extends TimedRobot { //Declaracion de variables y Objetos.

  boolean ShooterState;

  double PIDLimeOutGiro; //Variable Para obtener la potencia del giro del robot usando Limelight
  double PIDLimeOutAvance; //Variable para obtener la potencia de avance del robot usando Limelight.

  DutyCycleEncoder pivotAbsEncoder; //Variable para DIO Sensor absoluto del pivot.
  Double pivotAbsPosition;          //Lectura de sensor en revolucion 0-1
  Double pivotAbsPositionGrados;    //Lectura del sensor en grados 0-360

  DigitalInput LimitSwitchPivotdown; //Sensor infrarojo de deteccion de pivot abajo
  DigitalInput LimitSwitchPivotup;   //Sensor infrarojo de deteccion de pivot arriba
  DigitalInput LimitSwitchNote;      //Sensor infrarojo de deteccion de nota en intake.

  boolean LimitSwitchPivotAbajo;      //variable para guardar el estado del sensor de pivot abajo
  boolean LimitSwitchPivotArriba;     //Variable para guardar el estado del sensor de pivot arriba
  boolean LimitSwitchNoteDetected;    //Variable para guardar el estado de la nota en el intake.

  Timer cronos; //Timer para iniciar un cronometro de tiempo  para control de autonomos por tiempo.
  Timer shooterTime; //Timer para inciiar un cronometro de retardo en el shotter.

  PS4Controller control;   //Crear el control de operador de chasis.
  GenericHID operador;     //Crear el control de operador de mecanismos.

  AHRS navx; //Crear objeto de Gyroscopio NavX

  Optional<Alliance> ally; //Objeto de lectura de seleccion de alianza.
  
  double tiempoMatch; //Variable para llevar la cuenta del tiempo del match para control de leds en end game.

  //AUTÓNOMOS
 
  double rotatcionAutonomo; //Variable para guardar el calculo de PID de rotacion en autonomos.
 
   
  //Creacion de objetos de motores del chasis con controlador VictorSPX y motores CIM.
  WPI_VictorSPX rearRight;
  WPI_VictorSPX rearLeft;
  WPI_VictorSPX frontRight;
  WPI_VictorSPX frontLeft;
 
  MecanumDrive myRobot; //Objeto para crear un robot con chasis mecanum drive.
 
  // Variables para guardar los valores de los joysticks provenientes del control de PS4.
  double des_x; 
  double des_y;
  double rot;

  int angulosChassis; //Variable para guardar los datos de los  graodos del POV.
 
  //PID PARA EL CHASSIS

  PIDController chassisPID; //Controlador PID para chasis y sus parametros.

  static final double kP = 0.018;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kToleranceDegrees = 2.0f;
 
  //Declaracion de objetos para mecanismos con controladores SPARK MAX y Motores NEO.
  CANSparkMax intakeRight, intakeLeft;
  CANSparkMax shooterRight, shooterLeft;
  CANSparkMax  climberRight, climberLeft;
  CANSparkMax pivotRight, pivotLeft;

  //Variables para control PID del Pivot
  boolean pivotSwitchPID;  //Habilita o deshabilita el control por PID
  double pivotPIDsetpoint; //El setpoint utilizado para las diferentes posiciones del pivot.
  PIDController pivotPID;  //Controlador PID del Pivo y sus parametros.

  static final double pivotkP = 0.03;
  static final double pivotkI = 0.00;
  static final double pivotkD = 0.001;
  static final double pivotkToleranceDegrees = 2.0f;
 
 
  //Variables utilizaras para enviar o recibir datos desde el Dashboard.

  
  double chassisPote; //Potencia maxima del chasis
  boolean fod;        //Habilitar o deshabilitar el control Field Oriented Drive.


  double intakePote;  //Potencia Maxima del Intake
  double shooterPote; //Potencia Maxima del shooter en posicion de tiro a Speaker
  double shooterAmpPote; //Potencia Maxima del shooter en posicion para anotar en Amplificador.

  double pivotPote;   //Potencia Maxima del Pivot en modo manual
  double pivotLimiteInferior;  //Limite inferior del encoder del pivot
  double pivotLimiteSuperior; //Limite superior del encoder del pivot.
  double pivotPosicionTiro; // Variable para guardar la posicion de tiro del pivot

  double climberPote;          //Potencia Maxima del Climber
  double climberLimiteInferior; //Limite inferior del Climber
  double climberLimiteSuperior; //Limite superior del climber.

  double pivotPotePID;         //Potencia Maxima del Pivot usando el controlador PID.


  // Valores de posicion de los encoders
  double pivotRotacion;    //Lectura de posicion del Pivot
  double climberRotacion;  //Lectura de posicion del Climber

  RelativeEncoder pivotEncoder;  //objeto encoder del pivot desde el sparkmax
  RelativeEncoder climberEncoder; //Objeto encoder del climber desde el sparkmax.


  //Creacion de objetos de protocolo Network Table para obtener los datos de la Limelight.
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");

  //PID para Apunte automatico.
  PIDController PIDLimeLightGiro; //PID para controlar giro respecto a tx de Limelight
  double kpLimeLightGiro ;   //Proporcional de PID de limelight
  
  double minGiroLimelight = 0.05;  //Potencia minima para mover el chasis.

  PIDController PIDLimeLightAvance; //PID para controlar rango de tiro usando Limelight.
  double kpLimeLightAvance ;   //Proporcional de giro usando limelight.

  // Variables para control de tiras led direccionables NeoPixel.
  AddressableLED m_led; //Creacion de objeto de la tira de leds
  AddressableLEDBuffer m_ledBuffer; //Buffer para almacenamiento de los colores que se enviaran a la tira
  int redLeds, bluLeds; //Valores de color azul y Rojo en RGB 255
  double intervaloLeds = 0.25; //Velocidad de parpadeo de los leds.
  double tiempoAnterior; //Almacena el ultimo tiempo de actualizacion de leds
  boolean ledBlink = true; //Activar o desactivar el parpadeo de leds.
  int m_rainbowFirstPixelHue; //Almacena el tono de color en modo (Hue, Saturation, Lightness)

  //Seleccion de lista de autonomos.
  private static final String kDefaultAuto = "Sin Autonomo";
  private static final String kAzulIzquierdaAuto = "Azul Izquierda";
  private static final String kAzulCentroAuto = "Azul Centro";
  private static final String kAzulDerechaAuto = "Azul Derecha";
  private static final String kRojoIzquierdaAuto = "Rojo Izquierda";
  private static final String kRojoCentroAuto = "Rojo Centro";
  private static final String kRojoDerechaAuto = "Rojo Derecha";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  


  @Override
  public void robotInit() {// Construir todos los objetos con sus puertos y valores iniciales.

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Azul Izquierda", kAzulIzquierdaAuto);
    m_chooser.addOption("Azul Centro", kAzulCentroAuto);
    m_chooser.addOption("Azul Derecha", kAzulDerechaAuto);
    m_chooser.addOption("Rojo Izquierda", kRojoIzquierdaAuto);
    m_chooser.addOption("Rojo Centro", kRojoCentroAuto);
    m_chooser.addOption("Rojo Derecha", kRojoDerechaAuto);

    SmartDashboard.putData("Seleccion Autonomo", m_chooser);



    pivotAbsEncoder = new DutyCycleEncoder(3); //Constructor de Encoder absoluto del pivot.
    
    //Declaracion de sensores Digitales
    LimitSwitchPivotdown = new DigitalInput(0);  //Sensor infrarojo conectado en DIO 0
    LimitSwitchPivotup = new DigitalInput(1);    // Sensor infrarojo conectado en DIO 1
    LimitSwitchNote = new DigitalInput(2);       // Sensor infrarojo conectado en DIO 2
  
    cronos = new Timer(); //Nuevo timer para cronometrar tiempo de autonomo.
    shooterTime = new Timer();

    //Asignacion de IDS de red CAN a motores del Chasis con controladores VictorSPX y motores CIM.
    rearRight = new WPI_VictorSPX(2);
    rearLeft = new WPI_VictorSPX(3);
    frontRight = new WPI_VictorSPX(4);
    frontLeft = new WPI_VictorSPX(5);

    //Invertir giro de motores segun su posicion en el chasis.
    rearRight.setInverted(false); 
    rearLeft.setInverted(true);
    frontRight.setInverted(false); 
    frontLeft.setInverted(true);

    //Cambio de modo NEUTRAL de los motores a modo COAST cuando no esta operando en algun OPMODE.
    rearLeft.setNeutralMode(NeutralMode.Coast);
    rearRight.setNeutralMode(NeutralMode.Coast);
    frontLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);

    //Creacion del objeto del chasis tipo Mecanum Drive, con los motores antes declarados del chasis.
    myRobot = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
   
    // Asignacion de IDS de red CAN a motores del Chasis con controladores SPARK MAX y Motores NEO.   
    intakeLeft = new CANSparkMax(6, MotorType.kBrushless);
    intakeRight = new CANSparkMax(7, MotorType.kBrushless);
    shooterLeft = new CANSparkMax(8, MotorType.kBrushless);
    shooterRight = new CANSparkMax(9, MotorType.kBrushless);
    pivotLeft = new CANSparkMax(10, MotorType.kBrushless);
    pivotRight = new CANSparkMax(11, MotorType.kBrushless);
    climberLeft = new CANSparkMax(12, MotorType.kBrushless);
    climberRight = new CANSparkMax(13, MotorType.kBrushless);

    // Habilitar seguimiento MAESTRO - ESCLAVO e invertir giro en los motores de los mecanismos.
    intakeLeft.follow(intakeRight, true); // Intake left es invertido y sigue a intakeRight.
    shooterLeft.follow(shooterRight);            //Shooter Left Sigue a Shooter right.
    pivotLeft.follow(pivotRight, true);   // pivot left es invertido y sigue a pivotRight.
    climberLeft.follow(climberRight, true); // climber left es invertido y sigue a climberRight.


    //Lectura de encoders de mecanismos con controlador PID.
    pivotEncoder = pivotRight.getEncoder(); //Se obtiene la lectura del encoder de pivot Derecho
    pivotEncoder.setPosition(0); //Se resetea la posicion del encoder a 0.
    
    climberEncoder = climberRight.getEncoder(); //Se obtiene la lectura del encoder de climber derecho.
    climberEncoder.setPosition(0); //Se resetea la posicion del encoder a 0.
  
    //Creacion de Joysticks 0 para control de chasis PS4 y 1 para control de mecanismos Logitech.
    control = new PS4Controller(0);
    operador = new GenericHID(1);

   //Creacion de objeto de la NAVX conectado al puerto SPI del roborio.
    navx = new AHRS(SPI.Port.kMXP); 
    navx.reset();

    //Creacion de controlador PID del chasis.
    chassisPID = new PIDController(kP, kI, kD); //Parametros PID del controlador
    chassisPID.enableContinuousInput(-180.0f, 180.0f); //Establece el rango de movimiento en forma continua del mecanismo
    chassisPID.setIntegratorRange(-1.0, 1.0); //Establece los valores max y min del integrador para no saturarse si el error persiste.
    chassisPID.setTolerance(kToleranceDegrees); //Establece la tolerancia minima para el error para que el PID no corrija con valores muy pequeños.
    chassisPID.isContinuousInputEnabled(); //Verifica que la funcion de entrada continua este habilitada.

    //Creacion del controlador PID del Pivot.
    pivotPID = new PIDController(pivotkP, pivotkI, pivotkD); // Parametros PID del controlador
    pivotPID.setTolerance(pivotkToleranceDegrees); // Establece la tolerancia minima para el error para que el PID no corrija valores muy pequeños.

    //Controlador PID de apunte automatico con limelight.
    
    PIDLimeLightGiro = new PIDController(0.03, 0, 0.001);
    PIDLimeLightGiro.setTolerance(1);
    PIDLimeLightAvance = new PIDController(0.03, 0, 0.001);
    PIDLimeLightAvance.setTolerance(0.1);
                                                 
    
    // inicializar datos para visualizacion en Dashboard con valores iniciales.
    SmartDashboard.putNumber("Potencia", 0.9);
    SmartDashboard.putNumber("Potencia intake", 0.8);
    SmartDashboard.putNumber("Potencia shooter", 0.7); //0.3 para amp
    SmartDashboard.putNumber("Potencia shooter amp", 0.3);
    SmartDashboard.putNumber("Potencia pivot", 0.7);
    SmartDashboard.putNumber("Potencia Pivot PID", 0.75);
    SmartDashboard.putNumber("Potencia climber", 0.9);
    SmartDashboard.putBoolean("FOD", true);
    SmartDashboard.putNumber("Rotacion pivot", 0);
    SmartDashboard.putNumber("Rotacion climber", 0);
    SmartDashboard.putNumber("Pivot Limite Superior", 107);
    SmartDashboard.putNumber("Pivot Limite Inferior", 1);
    SmartDashboard.putNumber("Climber Limite Superior", 225);
    SmartDashboard.putNumber("Climber Limite Inferior", 10);
    //SmartDashboard.putNumber("Seleccion de Autonomo", 0);
    SmartDashboard.putNumber("Angulo", (int) navx.getYaw());
    SmartDashboard.putNumber("Angulo Pivot", 0);
    SmartDashboard.putNumber("Posicion Tiro", 33);

   
    UsbCamera camera = CameraServer.startAutomaticCapture();// Inicia transmision de webcam.
    camera.setResolution(320, 240);
    camera.setFPS(15);


    // Configura la tira led Neopixel.
    m_led = new AddressableLED(0); //Indica el puerto PWM de la tira led.
    m_ledBuffer = new AddressableLEDBuffer(27); //indica el numero maximo de leds conectados
    m_led.setLength(m_ledBuffer.getLength()); //Obtiene la longitud maxima de leds de la tira.
    m_led.setData(m_ledBuffer); //Envia los datos a todos los leds del buffer
    m_led.start(); //inicia la transmision de datos a la tira.
    
  }


  @Override
  public void robotPeriodic() {

    pivotAbsPosition = pivotAbsEncoder.get(); //Obtener los datos del encoder en revoluciones.
    pivotAbsPositionGrados = pivotAbsPosition * 360; //Convertir a grados el encoder absoluto.

    //Obtener valores de la limelight.
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double target = tv.getDouble(0.0);

    //Envia los valores de la deteccion de la limelight al Dashboard.
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightTarget", target);
    

    //Envia los valores de voltaje y angulo del chasis al dashboard.
    SmartDashboard.putNumber("Voltaje", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Angulo", (int) navx.getYaw());
    
    //Lectura de los sensores infrarojos para visualizacion en dashboard.
    LimitSwitchPivotAbajo = !LimitSwitchPivotdown.get();
    LimitSwitchPivotArriba = !LimitSwitchPivotup.get();
    LimitSwitchNoteDetected = !LimitSwitchNote.get();

    SmartDashboard.putBoolean("Pivot abajo", LimitSwitchPivotAbajo);
    SmartDashboard.putBoolean("Pivot arriba", LimitSwitchPivotArriba);
    SmartDashboard.putBoolean("Nota Detectada", LimitSwitchNoteDetected);
    SmartDashboard.putNumber("Angulo Pivot", pivotAbsPositionGrados);
   }


  @Override
  public void autonomousInit() {


    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    //Reset de cronometro y navx
    cronos.start();
    cronos.reset();

    navx.reset();

    //Cambio de modo neutral de motores a modo BRAKE
    rearLeft.setNeutralMode(NeutralMode.Brake);
    rearRight.setNeutralMode(NeutralMode.Brake);
    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);

      //obtencion de color de alianza, si es roja los leds son rojos, si es azul los leds son azules.
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
    //Variables para leer seleccion de autonomo.
    double autoSelec;
    int autoSelectInt;

    //obtencion de datos desde el Dashboard.
    autoSelec = SmartDashboard.getNumber("Seleccion de Autonomo", 0);
    shooterPote = SmartDashboard.getNumber("Potencia shooter", 0);
    pivotPote = SmartDashboard.getNumber("Potencia pivot", 0);
    pivotPotePID = SmartDashboard.getNumber("Potencia Pivot PID", 0);
    pivotLimiteSuperior = SmartDashboard.getNumber("Pivot Limite Superior", 0);
    pivotLimiteInferior = SmartDashboard.getNumber("Pivot Limite Inferior", 0);
    intakePote = SmartDashboard.getNumber("Potencia intake", 0);
    pivotPosicionTiro = SmartDashboard.getNumber("Posicion Tiro", 0);
    autoSelectInt = (int)autoSelec;
   

    // Obtener valores de los encoders de pivot y climber (Se invierte el giro del encoder por la posicion del motor)
    // pivotRotacion = -pivotEncoder.getPosition(); 
   
    pivotAbsPosition = pivotAbsEncoder.get(); // Obtener los datos del encoder en revoluciones.
    pivotAbsPositionGrados = pivotAbsPosition * 360; // Convertir a grados el encoder absoluto.
    pivotRotacion = pivotAbsPositionGrados;

    climberRotacion = -climberEncoder.getPosition();

    //Se envia los valores de los encoders al Dashboard.
    SmartDashboard.putNumber("Rotacion pivot", (int)pivotRotacion); 
    SmartDashboard.putNumber("Rotacion climber", climberRotacion);

  switch (m_autoSelected) { //Variable de seleccion de autonomo.
    //Autonomo que no hace nada solo para no usar el cero en el dashboard.
    case kDefaultAuto:
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      break;
//Autonomo de lado azul izquierdo
      case kAzulIzquierdaAuto:

 if (cronos.get()<=2) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      pivotPIDsetpoint = pivotPosicionTiro;
      pivotSwitchPID = true;
    }
    else if (cronos.get()<=3) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.set(shooterPote);
      pivotPIDsetpoint = pivotPosicionTiro;
      pivotSwitchPID = true;

    }
    else if (cronos.get()<=4.5) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      intakeRight.set(intakePote);
      pivotPIDsetpoint = pivotPosicionTiro;
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
    

     if (pivotRotacion >= pivotLimiteSuperior || (LimitSwitchPivotArriba)){ //si el angulo del pivot es mayor que el limite superior, se hace una "abrazadera" que sólo lo permite ir hacia el lado contrario
    pivotPote = MathUtil.clamp(pivotPote, 0, 1);
    } 
    else if (pivotRotacion <= pivotLimiteInferior || (LimitSwitchPivotAbajo)){
    pivotPote = MathUtil.clamp(pivotPote, -1, 0);
    }

    if(pivotSwitchPID){
    pivotPote = -pivotPID.calculate(pivotRotacion, pivotPIDsetpoint);
    pivotPote = MathUtil.clamp(pivotPote, -pivotPotePID, pivotPotePID);
    } 
    pivotRight.set(pivotPote);

      break;
//Autonomo de lado azul al centro

      case kAzulCentroAuto:

if (cronos.get()<=2) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      pivotPIDsetpoint = pivotPosicionTiro;
      pivotSwitchPID = true;
    }
    else if (cronos.get()<=3) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.set(shooterPote);
      pivotPIDsetpoint = pivotPosicionTiro;
      pivotSwitchPID = true;
    }
    else if (cronos.get()<=4.5) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      intakeRight.set(intakePote);
      pivotPIDsetpoint = pivotPosicionTiro;
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


     if (pivotRotacion >= pivotLimiteSuperior || (LimitSwitchPivotArriba)){ //si el angulo del pivot es mayor que el limite superior, se hace una "abrazadera" que sólo lo permite ir hacia el lado contrario
    pivotPote = MathUtil.clamp(pivotPote, 0, 1);
    } 
    else if (pivotRotacion <= pivotLimiteInferior || (LimitSwitchPivotAbajo)){
    pivotPote = MathUtil.clamp(pivotPote, -1, 0);
    }

    if(pivotSwitchPID){
    pivotPote = -pivotPID.calculate(pivotRotacion, pivotPIDsetpoint);
    pivotPote = MathUtil.clamp(pivotPote, -pivotPotePID, pivotPotePID);
    } 
    pivotRight.set(pivotPote);
//Autonomo de lado azul 

    break;

      case kAzulDerechaAuto:

if (cronos.get()<=2) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      pivotPIDsetpoint = pivotPosicionTiro;
      pivotSwitchPID = true;
    }
    else if (cronos.get()<=3) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.set(shooterPote);
      pivotPIDsetpoint = pivotPosicionTiro;
      pivotSwitchPID = true;

    }
    else if (cronos.get()<=4.5) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      intakeRight.set(intakePote);
      pivotPIDsetpoint = pivotPosicionTiro;
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
    

     if (pivotRotacion >= pivotLimiteSuperior || (LimitSwitchPivotArriba)){ //si el angulo del pivot es mayor que el limite superior, se hace una "abrazadera" que sólo lo permite ir hacia el lado contrario
    pivotPote = MathUtil.clamp(pivotPote, 0, 1);
    } 
    else if (pivotRotacion <= pivotLimiteInferior || (LimitSwitchPivotAbajo)){
    pivotPote = MathUtil.clamp(pivotPote, -1, 0);
    }

    if(pivotSwitchPID){
    pivotPote = -pivotPID.calculate(pivotRotacion, pivotPIDsetpoint);
    pivotPote = MathUtil.clamp(pivotPote, -pivotPotePID, pivotPotePID);
    } 
    pivotRight.set(pivotPote);

      break;
        case kRojoIzquierdaAuto:

        if (cronos.get()<=2) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      pivotPIDsetpoint = pivotPosicionTiro;
      pivotSwitchPID = true;
    }
    else if (cronos.get()<=3) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.set(shooterPote);
      pivotPIDsetpoint = pivotPosicionTiro;
      pivotSwitchPID = true;

    }
    else if (cronos.get()<=4.5) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      intakeRight.set(intakePote);
      pivotPIDsetpoint = pivotPosicionTiro;
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
    

     if (pivotRotacion >= pivotLimiteSuperior || (LimitSwitchPivotArriba)){ //si el angulo del pivot es mayor que el limite superior, se hace una "abrazadera" que sólo lo permite ir hacia el lado contrario
    pivotPote = MathUtil.clamp(pivotPote, 0, 1);
    } 
    else if (pivotRotacion <= pivotLimiteInferior || (LimitSwitchPivotAbajo)){
    pivotPote = MathUtil.clamp(pivotPote, -1, 0);
    }

    if(pivotSwitchPID){
    pivotPote = -pivotPID.calculate(pivotRotacion, pivotPIDsetpoint);
    pivotPote = MathUtil.clamp(pivotPote, -pivotPotePID, pivotPotePID);
    } 
    pivotRight.set(pivotPote);

        break;

        case kRojoCentroAuto:

        if (cronos.get()<=2) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      pivotPIDsetpoint = pivotPosicionTiro;
      pivotSwitchPID = true;
    }
    else if (cronos.get()<=3) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.set(shooterPote);
      pivotPIDsetpoint = pivotPosicionTiro;
      pivotSwitchPID = true;

    }
    else if (cronos.get()<=4.5) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      intakeRight.set(intakePote);
      pivotPIDsetpoint = pivotPosicionTiro;
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


     if (pivotRotacion >= pivotLimiteSuperior || (LimitSwitchPivotArriba) ){ //si el angulo del pivot es mayor que el limite superior, se hace una "abrazadera" que sólo lo permite ir hacia el lado contrario
    pivotPote = MathUtil.clamp(pivotPote, 0, 1);
    } 
    else if (pivotRotacion <= pivotLimiteInferior || (LimitSwitchPivotAbajo)){
    pivotPote = MathUtil.clamp(pivotPote, -1, 0);
    }

    if(pivotSwitchPID){
    pivotPote = -pivotPID.calculate(pivotRotacion, pivotPIDsetpoint);
    pivotPote = MathUtil.clamp(pivotPote, -pivotPotePID, pivotPotePID);
    } 
    pivotRight.set(pivotPote);

        break;
        case kRojoDerechaAuto:

        if (cronos.get()<=2) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      pivotPIDsetpoint = pivotPosicionTiro;
      pivotSwitchPID = true;
    }
    else if (cronos.get()<=3) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      shooterRight.set(shooterPote);
      pivotPIDsetpoint = pivotPosicionTiro;
      pivotSwitchPID = true;

    }
    else if (cronos.get()<=4.5) {
      rotatcionAutonomo = -chassisPID.calculate(navx.getYaw(),0);
      myRobot.driveCartesian(0, 0, rotatcionAutonomo , Rotation2d.fromDegrees(navx.getAngle()));
      intakeRight.set(intakePote);
      pivotPIDsetpoint = pivotPosicionTiro;
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
    

     if (pivotRotacion >= pivotLimiteSuperior || (LimitSwitchPivotArriba)){ //si el angulo del pivot es mayor que el limite superior, se hace una "abrazadera" que sólo lo permite ir hacia el lado contrario
    pivotPote = MathUtil.clamp(pivotPote, 0, 1);
    } 
    else if (pivotRotacion <= pivotLimiteInferior || (LimitSwitchPivotAbajo)){
    pivotPote = MathUtil.clamp(pivotPote, -1, 0);
    }

    if(pivotSwitchPID){
    pivotPote = -pivotPID.calculate(pivotRotacion, pivotPIDsetpoint);
    pivotPote = MathUtil.clamp(pivotPote, -pivotPotePID, pivotPotePID);
    } 
    pivotRight.set(pivotPote);

        break;

    

      default:
      break;
      }
   }
  


  @Override
  public void teleopInit() {

    //Cambiar modo neutral de motores a modo Brake.
    rearLeft.setNeutralMode(NeutralMode.Brake);
    rearRight.setNeutralMode(NeutralMode.Brake);
    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);

    pivotSwitchPID = false; //Apagar PID del Pivot para poderlo mover.

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


    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double target = tv.getDouble(0.0);
    
    //Obtener datos del Dashboard 
    chassisPote = SmartDashboard.getNumber("Potencia", 0);
    fod = SmartDashboard.getBoolean("FOD", fod);

    intakePote = SmartDashboard.getNumber("Potencia intake", 0);
    shooterPote = SmartDashboard.getNumber("Potencia shooter", 0);
    shooterAmpPote = SmartDashboard.getNumber("Potencia shooter amp", 0);
    pivotPote = SmartDashboard.getNumber("Potencia pivot", 0);
    climberPote = SmartDashboard.getNumber("Potencia climber", 0);
    pivotPotePID = SmartDashboard.getNumber("Potencia Pivot PID", 0);
    pivotLimiteSuperior = SmartDashboard.getNumber("Pivot Limite Superior", 0);
    pivotLimiteInferior = SmartDashboard.getNumber("Pivot Limite Inferior", 0);
    climberLimiteSuperior = SmartDashboard.getNumber("Climber Limite Superior", 0);
    climberLimiteInferior = SmartDashboard.getNumber("Climber Limite Inferior", 0);
    pivotPosicionTiro = SmartDashboard.getNumber("Posicion Tiro", 0);


   
    //Obtener valores del joystick de control de chasis con un multiplicador de la potencia maxima.
    des_x = -control.getLeftY() * chassisPote; 
    des_y = control.getLeftX() * chassisPote;
    rot = -control.getRightX() * chassisPote;
    angulosChassis = control.getPOV();

    // Obtener valores de los encoders de pivot y climber.
    //pivotRotacion = -pivotEncoder.getPosition(); 

    pivotAbsPosition = pivotAbsEncoder.get(); // Obtener los datos del encoder en revoluciones.
    pivotAbsPositionGrados = pivotAbsPosition * 360; // Convertir a grados el encoder absoluto.

    pivotRotacion = pivotAbsPositionGrados;
    climberRotacion = -climberEncoder.getPosition();

    //Envio de valores de los encoders al dashboard.
    SmartDashboard.putNumber("Rotacion pivot", (int)pivotRotacion); 
    SmartDashboard.putNumber("Rotacion climber", climberRotacion);

    SmartDashboard.putData("Mecanum Chasis", myRobot);



    //intercambiar modo de manejo entre FOD y NORMAL.
    if (fod) {
         
      //POV
        if (angulosChassis != -1) { //si el POV es diferente a -1, está presionado
          if (angulosChassis > 180) {
            angulosChassis = angulosChassis-360; //le restamos 360 para obtener un ángulo coterminal admitible por el PID
        }
          rot = -chassisPID.calculate(navx.getYaw(), angulosChassis);
        }

        myRobot.driveCartesian(des_x, des_y, rot, Rotation2d.fromDegrees(navx.getAngle())); //manejo con navx FOD
       
    } else {

      myRobot.driveCartesian(des_x, des_y, rot); //manejo normal

    }


//Reset Manual de Navx Para ordenar orientacion del robot.
    if (control.getOptionsButtonPressed()) { 
      navx.reset(); 
    }

// Control del Intake

  if ( (control.getR2Button() || operador.getRawAxis(3) > 0.5)){ // Succionar
     intakePote = intakePote;
    }
  else if
    (control.getL2Button() || operador.getRawAxis(2) > 0.5){ // Regresar
      intakePote = -intakePote;
    }
    else {
     
        intakePote = 0;
     
    }
    //solo se declara una vez el "set", en el if lo que cambia es la variable de potencia

    //SHOOTER
    //intakeRight.set(intakePote);
    

  if (pivotRotacion > 90) {   //Cambio de potencia cuando esta en posicion de AMP.
      shooterPote = shooterAmpPote;
    }
  else{
    shooterPote = shooterPote;
    }

  if (control.getSquareButton() && control.getL3Button()==true || operador.getRawButton(3) && operador.getRawButton(9)==true){ //Regresar
      shooterPote = -shooterPote;
    } 
    else if (control.getSquareButton() && control.getL3Button() == false
        || operador.getRawButton(3) && operador.getRawButton(9) == false) { //Lanzar
      shooterPote = shooterPote;
      

      //=================================================================================

      x = tx.getDouble(0.0);
      y = ty.getDouble(0.0);
      area = ta.getDouble(0.0);
      target = tv.getDouble(0.0);

      if (target == 1) {

        if (x > 1) {
          PIDLimeOutGiro = MathUtil.clamp(PIDLimeLightGiro.calculate(x, 0) - minGiroLimelight, -0.8, 0.8);

        } else if (x < -1) {
          PIDLimeOutGiro = MathUtil.clamp(PIDLimeLightGiro.calculate(x, 0) + minGiroLimelight, -0.8, 0.8);

        }

        PIDLimeOutAvance = MathUtil.clamp(-PIDLimeLightAvance.calculate(area, 0.3), -0.8, 0.8);

        myRobot.driveCartesian(PIDLimeOutAvance, 0, PIDLimeOutGiro);

        //================================================================================

      }
      
      
    } 
  else {
    shooterPote = 0;
        
   
    }
  
    shooterRight.set(shooterPote);
    intakeRight.set(intakePote);
    
    
    
    

  //Control de Pivot Automatico con PID

  if ((control.getCrossButtonPressed() || operador.getRawButtonPressed(1))) {
    pivotPIDsetpoint = pivotLimiteInferior;
    pivotSwitchPID = true;
    }
  if (control.getPSButtonPressed() || operador.getRawButtonPressed(8)) {
    pivotPIDsetpoint = 77;
    pivotSwitchPID = true;
    }
  if (control.getCircleButton() || operador.getRawButtonPressed(2)) {
    pivotPIDsetpoint = pivotPosicionTiro;
    pivotSwitchPID = true;
    }
    if (control.getTriangleButton() || operador.getRawButtonPressed(4)) {
      pivotPIDsetpoint = pivotLimiteSuperior;
      pivotSwitchPID = true;
    }
   
    //Control de Pivot Manual sin PID
    if (control.getR1Button() || operador.getRawButton(6)) { //SUBE
    pivotSwitchPID = false;
    pivotPote = -pivotPote;
    
    }  
    else if (control.getL1Button() || operador.getRawButton(5)) { //BAJA
    pivotSwitchPID = false;
    pivotPote = pivotPote; //no es necesario cambiarla
    
    } 
  else {
    pivotPote = 0;
    }

  if ((pivotRotacion >= pivotLimiteSuperior  ) || (LimitSwitchPivotArriba)){ //si el angulo del pivot es mayor que el limite superior, se hace una "abrazadera" que sólo lo permite ir hacia el lado contrario
    pivotPote = MathUtil.clamp(pivotPote, 0, 1);
    } 
  else if ((pivotRotacion <= pivotLimiteInferior ) || (LimitSwitchPivotAbajo) ){
    pivotPote = MathUtil.clamp(pivotPote, -1, 0);
    }

  if(pivotSwitchPID){
    pivotPote = -pivotPID.calculate(pivotRotacion, pivotPIDsetpoint);
    pivotPote = MathUtil.clamp(pivotPote, -pivotPotePID, pivotPotePID);

    } 

    pivotRight.set(pivotPote);
  
    SmartDashboard.putNumber("SP Pivot", pivotPIDsetpoint);
    SmartDashboard.putNumber("Sensor Pivot", (int)pivotRotacion);
    SmartDashboard.putNumber("Out Pivot", pivotPote);


  //Control de Climbers se necesitan mover las 2 palancas.
 
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

  //======================================================Control de LEDS para Eng Game ===========================================
  tiempoMatch = DriverStation.getMatchTime();
    
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

   //Si el robot esta deshabilitado poner chasis en Modo COAST para poderlo mover.
  @Override
  public void disabledInit() {
    rearLeft.setNeutralMode(NeutralMode.Coast);
    rearRight.setNeutralMode(NeutralMode.Coast);
    frontLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
  }

  //Poner leds en arcoiris cuando esta el robot deshabilitado.
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
  public void testInit() {
    

  }


  @Override
  public void testPeriodic() {
   /* 
    shooterPote = SmartDashboard.getNumber("Potencia shooter", 0);
    intakePote = SmartDashboard.getNumber("Potencia intake", 0);

    if (control.getSquareButton()) {

      
      double x = tx.getDouble(0.0);
      double y = ty.getDouble(0.0);
      double area = ta.getDouble(0.0);
      double target = tv.getDouble(0.0);

      if (target == 1) {

        if (x > 1) {
          PIDLimeOutGiro = MathUtil.clamp(PIDLimeLightGiro.calculate(x, 0) - minGiroLimelight, -0.8, 0.8);

        } else if (x < -1) {
          PIDLimeOutGiro = MathUtil.clamp(PIDLimeLightGiro.calculate(x, 0) + minGiroLimelight, -0.8, 0.8);

        }

        PIDLimeOutAvance = MathUtil.clamp(-PIDLimeLightAvance.calculate(area, 0.3), -0.8, 0.8);

        myRobot.driveCartesian(PIDLimeOutAvance, 0, PIDLimeOutGiro);

        if (Math.abs(x) <= 2) {

          shooterTime.start();
          shooterPote = shooterPote;

          if (shooterTime.get() > 0.5) {
            intakeRight.set(intakePote);

          }

          
        }

      } else if (target == 0) {

        if (control.getSquareButton() && control.getL3Button() == false) {
          shooterTime.start();
          shooterPote = shooterPote;
          if (shooterTime.get() > 0.5) {
            intakeRight.set(intakePote);
          }
          
        } else if (control.getSquareButton() && control.getL3Button() == true) {

          shooterPote = -shooterPote;
        }

      }

    } else {
      shooterPote = 0;
      shooterTime.reset();
      intakeRight.stopMotor();
      shooterRight.stopMotor();
      myRobot.stopMotor();
      

    }
    
    
    shooterRight.set(shooterPote);
    
      */
  }

   

  }





