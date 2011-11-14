/*
 GUI understands following command strings:
 M  Multiwii @ arduino send all data to GUI
 W  write to Eeprom @ arduino
 S  acc Sensor calibration request
 E  mag Sensor calibration request
*/
import processing.serial.*; // serial library
import controlP5.*; // controlP5 library
import processing.opengl.*;

Serial g_serial;
ControlP5 controlP5;
Textlabel txtlblWhichcom,version; 
ListBox commListbox;

int frame_size = 124;

cGraph g_graph;
int windowsX    = 800;        int windowsY    = 540;
int xGraph      = 10;         int yGraph      = 325;
int xObj        = 700;        int yObj        = 450;
int xParam      = 120;        int yParam      = 10;
int xRC         = 650;        int yRC         = 15;
int xMot        = 490;        int yMot        = 30;
int xButton     = 485;        int yButton    = 185;
int xBox        = xParam+190; int yBox      = yParam+70;

boolean axGraph =true,ayGraph=true,azGraph=true,gxGraph=true,gyGraph=true,gzGraph=true,baroGraph=true,headGraph=true, magxGraph =true,magyGraph=true,magzGraph=true;
boolean debug1Graph = false,debug2Graph = false,debug3Graph = false,debug4Graph = false;

int multiType;  // 1 for tricopter, 2 for quad+, 3 for quadX, ...

cDataArray accPITCH   = new cDataArray(100), accROLL    = new cDataArray(100), accYAW     = new cDataArray(100);
cDataArray gyroPITCH  = new cDataArray(100), gyroROLL   = new cDataArray(100), gyroYAW    = new cDataArray(100);
cDataArray magxData   = new cDataArray(100), magyData   = new cDataArray(100), magzData   = new cDataArray(100);
cDataArray baroData   = new cDataArray(100);
cDataArray headData    = new cDataArray(100);
cDataArray debug1Data   = new cDataArray(100),debug2Data   = new cDataArray(100),debug3Data   = new cDataArray(100),debug4Data   = new cDataArray(100);

private static final int ROLL = 0, PITCH = 1, YAW = 2, ALT = 3, VEL = 4, LEVEL = 5, MAG = 6;

Numberbox confP[] = new Numberbox[7], confI[] = new Numberbox[7], confD[] = new Numberbox[7];
Numberbox confRC_RATE, confRC_EXPO, rollPitchRate, yawRate, dynamic_THR_PID;

int byteP[] = new int[7], byteI[] = new int[7],byteD[] = new int[7];

int  byteRC_RATE,byteRC_EXPO, byteRollPitchRate,byteYawRate, byteDynThrPID;

Slider rcStickThrottleSlider,rcStickRollSlider,rcStickPitchSlider,rcStickYawSlider,rcStickAUX1Slider,rcStickAUX2Slider,rcStickCAM1Slider,rcStickCAM2Slider;

Slider motSliderV0,motSliderV1,motSliderV2,motSliderV3,motSliderV4,motSliderV5;
Slider servoSliderH1,servoSliderH2,servoSliderH3,servoSliderH4,servoSliderV0,servoSliderV1,servoSliderV2;

Slider axSlider,aySlider,azSlider,gxSlider,gySlider,gzSlider , magxSlider,magySlider,magzSlider , baroSlider,headSlider;
Slider debug1Slider,debug2Slider,debug3Slider,debug4Slider;

Slider scaleSlider;

Button buttonREAD,buttonWRITE,buttonCALIBRATE_ACC,buttonCALIBRATE_MAG,buttonSTART,buttonSTOP;

Button buttonNunchuk,buttonI2cAcc,buttonI2cBaro,buttonI2cMagneto,buttonGPS;
Button buttonI2cAccActive,buttonI2cBaroActive,buttonI2cMagnetoActive,buttonGPSActive;

color yellow_ = color(200, 200, 20), green_ = color(30, 120, 30), red_ = color(120, 30, 30);
boolean graphEnable = false;boolean readEnable = false;boolean writeEnable = false;boolean calibrateEnable = false;

float gx,gy,gz,ax,ay,az,magx,magy,magz,baro,head,angx,angy,debug1,debug2,debug3,debug4;
int GPS_distanceToHome, GPS_directionToHome;
int  GPS_numSat,GPS_fix,GPS_update;
int init_com,graph_on,pMeterSum,intPowerTrigger,bytevbat;

Numberbox confPowerTrigger;

float mot[] = new float[8];

float servo0=1500,servo1=1500,servo2=1500,servo3=1500;
float rcThrottle = 1500,rcRoll = 1500,rcPitch = 1500,rcYaw =1500,
      rcAUX1=1500, rcAUX2=1500, rcCAM1=1500, rcCAM2=1500;
int nunchukPresent,i2cAccPresent,i2cBaroPresent,i2cMagnetoPresent,GPSPresent,levelMode;

float time1,time2;
int cycleTime;

CheckBox checkbox[] = new CheckBox[8];
int activation[] = new int[8];

PFont font8,font12,font15;

// coded by Eberhard Rensch
// Truncates a long port name for better (readable) display in the GUI
String shortifyPortName(String portName, int maxlen)  {
  String shortName = portName;
  if(shortName.startsWith("/dev/")) shortName = shortName.substring(5);  
  if(shortName.startsWith("tty.")) shortName = shortName.substring(4); // get rid off leading tty. part of device name
  if(portName.length()>maxlen) shortName = shortName.substring(0,(maxlen-1)/2) + "~" +shortName.substring(shortName.length()-(maxlen-(maxlen-1)/2));
  if(shortName.startsWith("cu.")) shortName = "";// only collect the corresponding tty. devices
  return shortName;
}

controlP5.Controller hideLabel(controlP5.Controller c) {
  c.setLabel("");
  c.setLabelVisible(false);
  return c;
}

void setup() {
  size(windowsX,windowsY,OPENGL);
  frameRate(20); 
  
  font8 = createFont("Arial bold",8,false);font12 = createFont("Arial bold",12,false);font15 = createFont("Arial bold",15,false);

  controlP5 = new ControlP5(this); // initialize the GUI controls
  controlP5.setControlFont(font12);

  g_graph  = new cGraph(xGraph+110,yGraph, 480, 200);
  commListbox = controlP5.addListBox("portComList",5,65,110,240); // make a listbox and populate it with the available comm ports

  commListbox.captionLabel().set("PORT COM");
  commListbox.setColorBackground(red_);
  for(int i=0;i<Serial.list().length;i++) {
    String pn = shortifyPortName(Serial.list()[i], 13);
    if (pn.length() >0 ) commListbox.addItem(pn,i); // addItem(name,value)
  }

  // text label for which comm port selected
  txtlblWhichcom = controlP5.addTextlabel("txtlblWhichcom","No Port Selected",5,42); // textlabel(name,text,x,y)
    
  buttonSTART = controlP5.addButton("bSTART",1,xGraph+110,yGraph-25,40,19); buttonSTART.setLabel("START"); buttonSTART.setColorBackground(red_);
  buttonSTOP = controlP5.addButton("bSTOP",1,xGraph+160,yGraph-25,40,19); buttonSTOP.setLabel("STOP"); buttonSTOP.setColorBackground(red_);

  buttonNunchuk = controlP5.addButton("bNUNCHUK",1,xButton,yButton,70,15);buttonNunchuk.setColorBackground(red_);buttonNunchuk.setLabel("NUNCHUK");
  buttonI2cAcc = controlP5.addButton("bACC",1,xButton,yButton+17,70,15); buttonI2cAcc.setColorBackground(red_);buttonI2cAcc.setLabel("ACC");
  buttonI2cBaro = controlP5.addButton("bBARO",1,xButton,yButton+34,70,15); buttonI2cBaro.setColorBackground(red_);buttonI2cBaro.setLabel("BARO");
  buttonI2cMagneto = controlP5.addButton("bMAG",1,xButton,yButton+51,70,15); buttonI2cMagneto.setColorBackground(red_);buttonI2cMagneto.setLabel("MAG");
  buttonGPS = controlP5.addButton("bGPS",1,xButton,yButton+68,70,15); buttonGPS.setColorBackground(red_);buttonGPS.setLabel("GPS");

  buttonI2cAccActive = controlP5.addButton("accOFF",1,xButton+75,yButton,70,32);buttonI2cAccActive.setColorBackground(red_);buttonI2cAccActive.setLabel("OFF");
  buttonI2cBaroActive = controlP5.addButton("baroOFF",1,xButton+75,yButton+34,70,15);buttonI2cBaroActive.setColorBackground(red_);buttonI2cBaroActive.setLabel("OFF");
  buttonI2cMagnetoActive = controlP5.addButton("magnetoOFF",1,xButton+75,yButton+51,70,15);buttonI2cMagnetoActive.setColorBackground(red_);buttonI2cMagnetoActive.setLabel("OFF");
  buttonGPSActive = controlP5.addButton("GPSOFF",1,xButton+75,yButton+68,70,15); buttonGPSActive.setColorBackground(red_);buttonGPSActive.setLabel("OFF");

  color c,black;
  black = color(0,0,0);
  int xo = xGraph-7;
  int x = xGraph+40;
  int y1= yGraph+10;  //ACC
  int y2= yGraph+55;  //GYRO
  int y5= yGraph+100; //MAG
  int y3= yGraph+150; //ALT
  int y4= yGraph+165; //HEAD
  int y7= yGraph+185; //GPS
  int y6= yGraph+205; //DEBUG

  Toggle tACC_ROLL =     controlP5.addToggle("ACC_ROLL",true,x,y1+10,20,10);tACC_ROLL.setColorActive(color(255, 0, 0));tACC_ROLL.setColorBackground(black);tACC_ROLL.setLabel(""); 
  Toggle tACC_PITCH =   controlP5.addToggle("ACC_PITCH",true,x,y1+20,20,10);tACC_PITCH.setColorActive(color(0, 255, 0));tACC_PITCH.setColorBackground(black);tACC_PITCH.setLabel(""); 
  Toggle tACC_Z =           controlP5.addToggle("ACC_Z",true,x,y1+30,20,10);tACC_Z.setColorActive(color(0, 0, 255));tACC_Z.setColorBackground(black);tACC_Z.setLabel(""); 
  Toggle tGYRO_ROLL =   controlP5.addToggle("GYRO_ROLL",true,x,y2+10,20,10);tGYRO_ROLL.setColorActive(color(200, 200, 0));tGYRO_ROLL.setColorBackground(black);tGYRO_ROLL.setLabel(""); 
  Toggle tGYRO_PITCH = controlP5.addToggle("GYRO_PITCH",true,x,y2+20,20,10);tGYRO_PITCH.setColorActive(color(0, 255, 255));tGYRO_PITCH.setColorBackground(black);tGYRO_PITCH.setLabel(""); 
  Toggle tGYRO_YAW =     controlP5.addToggle("GYRO_YAW",true,x,y2+30,20,10);tGYRO_YAW.setColorActive(color(255, 0, 255));tGYRO_YAW.setColorBackground(black);tGYRO_YAW.setLabel(""); 
  Toggle tBARO =               controlP5.addToggle("BARO",true,x,y3 ,20,10);tBARO.setColorActive(color(125, 125, 125));tBARO.setColorBackground(black);tBARO.setLabel(""); 
  Toggle tHEAD =               controlP5.addToggle("HEAD",true,x,y4 ,20,10);tHEAD.setColorActive(color(225, 225, 125));tHEAD.setColorBackground(black);tHEAD.setLabel(""); 
  Toggle tMAGX =             controlP5.addToggle("MAGX",true,x,y5+10,20,10);tMAGX.setColorActive(color(50, 100, 150));tMAGX.setColorBackground(black);tMAGX.setLabel(""); 
  Toggle tMAGY =             controlP5.addToggle("MAGY",true,x,y5+20,20,10);tMAGY.setColorActive(color(100, 50, 150));tMAGY.setColorBackground(black);tMAGY.setLabel(""); 
  Toggle tMAGZ =             controlP5.addToggle("MAGZ",true,x,y5+30,20,10);tMAGZ.setColorActive(color(150, 100, 50));tMAGZ.setColorBackground(black);tMAGZ.setLabel(""); 
  Toggle tDEBUG1 =         controlP5.addToggle("DEBUG1",true,x+70,y6,20,10);tDEBUG1.setColorActive(color(150, 100, 50));tDEBUG1.setColorBackground(black);tDEBUG1.setLabel("");tDEBUG1.setValue(0);
  Toggle tDEBUG2 =         controlP5.addToggle("DEBUG2",true,x+190,y6,20,10);tDEBUG2.setColorActive(color(150, 100, 50));tDEBUG2.setColorBackground(black);tDEBUG2.setLabel("");tDEBUG2.setValue(0);
  Toggle tDEBUG3 =         controlP5.addToggle("DEBUG3",true,x+310,y6,20,10);tDEBUG3.setColorActive(color(150, 100, 50));tDEBUG3.setColorBackground(black);tDEBUG3.setLabel("");tDEBUG3.setValue(0);
  Toggle tDEBUG4 =         controlP5.addToggle("DEBUG4",true,x+430,y6,20,10);tDEBUG4.setColorActive(color(150, 100, 50));tDEBUG4.setColorBackground(black);tDEBUG4.setLabel("");tDEBUG4.setValue(0);

  controlP5.addTextlabel("acclabel","ACC",xo,y1);
  controlP5.addTextlabel("accrolllabel","   ROLL",xo,y1+10);
  controlP5.addTextlabel("accpitchlabel","   PITCH",xo,y1+20);
  controlP5.addTextlabel("acczlabel","   Z",xo,y1+30);
  controlP5.addTextlabel("gyrolabel","GYRO",xo,y2);
  controlP5.addTextlabel("gyrorolllabel","   ROLL",xo,y2+10);
  controlP5.addTextlabel("gyropitchlabel","   PITCH",xo,y2+20);
  controlP5.addTextlabel("gyroyawlabel","   YAW",xo,y2+30);
  controlP5.addTextlabel("maglabel","MAG",xo,y5);
  controlP5.addTextlabel("magrolllabel","   ROLL",xo,y5+10);
  controlP5.addTextlabel("magpitchlabel","   PITCH",xo,y5+20);
  controlP5.addTextlabel("magyawlabel","   YAW",xo,y5+30);
  controlP5.addTextlabel("altitudelabel","ALT",xo,y3);
  controlP5.addTextlabel("headlabel","HEAD",xo,y4);
  controlP5.addTextlabel("debug1","debug1",x+90,y6);
  controlP5.addTextlabel("debug2","debug2",x+210,y6);
  controlP5.addTextlabel("debug3","debug3",x+330,y6);
  controlP5.addTextlabel("debug4","debug4",x+450,y6);

  axSlider   =         controlP5.addSlider("axSlider",-1000,+1000,0,x+20,y1+10,50,10);axSlider.setDecimalPrecision(0);axSlider.setLabel("");
  aySlider   =         controlP5.addSlider("aySlider",-1000,+1000,0,x+20,y1+20,50,10);aySlider.setDecimalPrecision(0);aySlider.setLabel("");
  azSlider   =         controlP5.addSlider("azSlider",-1000,+1000,0,x+20,y1+30,50,10);azSlider.setDecimalPrecision(0);azSlider.setLabel("");
  gxSlider   =           controlP5.addSlider("gxSlider",-500,+500,0,x+20,y2+10,50,10);gxSlider.setDecimalPrecision(0);gxSlider.setLabel("");
  gySlider   =           controlP5.addSlider("gySlider",-500,+500,0,x+20,y2+20,50,10);gySlider.setDecimalPrecision(0);gySlider.setLabel("");
  gzSlider   =           controlP5.addSlider("gzSlider",-500,+500,0,x+20,y2+30,50,10);gzSlider.setDecimalPrecision(0);gzSlider.setLabel("");
  baroSlider =        controlP5.addSlider("baroSlider",-30000,+30000,0,x+20,y3 ,50,10);baroSlider.setDecimalPrecision(1);baroSlider.setLabel("");
  headSlider  =          controlP5.addSlider("headSlider",-200,+200,0,x+20,y4  ,50,10);headSlider.setDecimalPrecision(0);headSlider.setLabel("");
  magxSlider  =      controlP5.addSlider("magxSlider",-5000,+5000,0,x+20,y5+10,50,10);magxSlider.setDecimalPrecision(0);magxSlider.setLabel("");
  magySlider  =      controlP5.addSlider("magySlider",-5000,+5000,0,x+20,y5+20,50,10);magySlider.setDecimalPrecision(0);magySlider.setLabel("");
  magzSlider  =      controlP5.addSlider("magzSlider",-5000,+5000,0,x+20,y5+30,50,10);magzSlider.setDecimalPrecision(0);magzSlider.setLabel("");
  debug1Slider  =    controlP5.addSlider("debug1Slider",-32000,+32000,0,x+130,y6,50,10);debug1Slider.setDecimalPrecision(1);debug1Slider.setLabel("");
  debug2Slider  =    controlP5.addSlider("debug2Slider",-32000,+32000,0,x+250,y6,50,10);debug2Slider.setDecimalPrecision(0);debug2Slider.setLabel("");
  debug3Slider  =    controlP5.addSlider("debug3Slider",-32000,+32000,0,x+370,y6,50,10);debug3Slider.setDecimalPrecision(0);debug3Slider.setLabel("");
  debug4Slider  =    controlP5.addSlider("debug4Slider",-32000,+32000,0,x+490,y6,50,10);debug4Slider.setDecimalPrecision(0);debug4Slider.setLabel("");

  for(int i=0;i<7;i++) {
    confP[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confP"+i,0,xParam+40,yParam+20+i*20,30,14));
    confP[i].setColorBackground(red_);confP[i].setMin(0);confP[i].setDirection(Controller.HORIZONTAL);confP[i].setDecimalPrecision(1);confP[i].setMultiplier(0.1);confP[i].setMax(20);}
  for(int i=0;i<6;i++) {
    confI[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confI"+i,0,xParam+75,yParam+20+i*20,40,14));
    confI[i].setColorBackground(red_);confI[i].setMin(0);confI[i].setDirection(Controller.HORIZONTAL);confI[i].setDecimalPrecision(3);confI[i].setMultiplier(0.001);confI[i].setMax(0.250);}
  for(int i=0;i<5;i++) {
    confD[i] = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("confD"+i,0,xParam+120,yParam+20+i*20,30,14));
    confD[i].setColorBackground(red_);confD[i].setMin(0);confD[i].setDirection(Controller.HORIZONTAL);confD[i].setDecimalPrecision(0);confD[i].setMultiplier(1);confD[i].setMax(50);}

  rollPitchRate = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("rollPitchRate",0,xParam+160,yParam+30,30,14));rollPitchRate.setDecimalPrecision(2);rollPitchRate.setMultiplier(0.01);
  rollPitchRate.setDirection(Controller.HORIZONTAL);rollPitchRate.setMin(0);rollPitchRate.setMax(1);rollPitchRate.setColorBackground(red_);
  yawRate = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("yawRate",0,xParam+160,yParam+60,30,14));yawRate.setDecimalPrecision(2);yawRate.setMultiplier(0.01);
  yawRate.setDirection(Controller.HORIZONTAL);yawRate.setMin(0);yawRate.setMax(1);yawRate.setColorBackground(red_); 
  dynamic_THR_PID = (controlP5.Numberbox) hideLabel(controlP5.addNumberbox("dynamic_THR_PID",0,xParam+300,yParam+12,30,14));dynamic_THR_PID.setDecimalPrecision(2);dynamic_THR_PID.setMultiplier(0.01);
  dynamic_THR_PID.setDirection(Controller.HORIZONTAL);dynamic_THR_PID.setMin(0);dynamic_THR_PID.setMax(1);dynamic_THR_PID.setColorBackground(red_);

  confRC_RATE = controlP5.addNumberbox("RC RATE",1,xParam+40,yParam+213,30,14);confRC_RATE.setDecimalPrecision(2);confRC_RATE.setMultiplier(0.02);confRC_RATE.setLabel("");
  confRC_RATE.setDirection(Controller.HORIZONTAL);confRC_RATE.setMin(0);confRC_RATE.setMax(5);confRC_RATE.setColorBackground(red_);
  confRC_EXPO = controlP5.addNumberbox("RC EXPO",0,xParam+40,yParam+240,30,14);confRC_EXPO.setDecimalPrecision(2);confRC_EXPO.setMultiplier(0.01);confRC_EXPO.setLabel("");
  confRC_EXPO.setDirection(Controller.HORIZONTAL);confRC_EXPO.setMin(0);confRC_EXPO.setMax(1);confRC_EXPO.setColorBackground(red_);

  for(int i=0;i<8;i++) {
    checkbox[i] =  controlP5.addCheckBox("cb"+i,xBox+40,yBox+20+13*i);
    checkbox[i].setColorActive(color(255));checkbox[i].setColorBackground(color(120));
    checkbox[i].setItemsPerRow(6);checkbox[i].setSpacingColumn(10);
    checkbox[i].setLabel("");
    hideLabel(checkbox[i].addItem(i + "1",1));hideLabel(checkbox[i].addItem(i + "2",2));hideLabel(checkbox[i].addItem(i + "3",3));
    hideLabel(checkbox[i].addItem(i + "4",4));hideLabel(checkbox[i].addItem(i + "5",5));hideLabel(checkbox[i].addItem(i + "6",6));
  }

  buttonREAD =      controlP5.addButton("READ",1,xParam+5,yParam+260,60,16);buttonREAD.setColorBackground(red_);
  buttonWRITE =     controlP5.addButton("WRITE",1,xParam+290,yParam+260,60,16);buttonWRITE.setColorBackground(red_);
  buttonCALIBRATE_ACC = controlP5.addButton("CALIB_ACC",1,xParam+210,yParam+260,70,16);buttonCALIBRATE_ACC.setColorBackground(red_);
  buttonCALIBRATE_MAG = controlP5.addButton("CALIB_MAG",1,xParam+130,yParam+260,70,16);buttonCALIBRATE_MAG.setColorBackground(red_);

  rcStickThrottleSlider = controlP5.addSlider("Throttle",900,2100,1500,xRC,yRC,10,100);rcStickThrottleSlider.setDecimalPrecision(0);
  rcStickPitchSlider =    controlP5.addSlider("Pitch",900,2100,1500,xRC+80,yRC,10,100);rcStickPitchSlider.setDecimalPrecision(0);
  rcStickRollSlider =     controlP5.addSlider("Roll",900,2100,1500,xRC,yRC+120,100,10);rcStickRollSlider.setDecimalPrecision(0);
  rcStickYawSlider  =     controlP5.addSlider("Yaw",900,2100,1500,xRC,yRC+135,100,10);rcStickYawSlider.setDecimalPrecision(0);
  rcStickAUX1Slider =     controlP5.addSlider("AUX1",900,2100,1500,xRC,yRC+150,100,10);rcStickAUX1Slider.setDecimalPrecision(0);
  rcStickAUX2Slider =     controlP5.addSlider("AUX2",900,2100,1500,xRC,yRC+165,100,10);rcStickAUX2Slider.setDecimalPrecision(0);
  rcStickCAM1Slider =     controlP5.addSlider("CAM1",900,2100,1500,xRC,yRC+180,100,10);rcStickCAM1Slider.setDecimalPrecision(0);
  rcStickCAM2Slider =     controlP5.addSlider("CAM2",900,2100,1500,xRC,yRC+195,100,10);rcStickCAM2Slider.setDecimalPrecision(0);

  motSliderV0  = controlP5.addSlider("motSliderV0",1000,2000,1500,0,0,10,100);motSliderV0.setDecimalPrecision(0);
  motSliderV1  = controlP5.addSlider("motSliderV1",1000,2000,1500,0,0,10,100);motSliderV1.setDecimalPrecision(0);
  motSliderV2  = controlP5.addSlider("motSliderV2",1000,2000,1500,0,0,10,100);motSliderV2.setDecimalPrecision(0);
  motSliderV3  = controlP5.addSlider("motSliderV3",1000,2000,1500,0,0,10,100);motSliderV3.setDecimalPrecision(0);
  motSliderV4  = controlP5.addSlider("motSliderV4",1000,2000,1500,0,0,10,100);motSliderV4.setDecimalPrecision(0);
  motSliderV5  = controlP5.addSlider("motSliderV5",1000,2000,1500,0,0,10,100);motSliderV5.setDecimalPrecision(0);

  servoSliderH1  = controlP5.addSlider("Servo0",1000,2000,1500,0,0,100,10);servoSliderH1.setDecimalPrecision(0);
  servoSliderH2 = controlP5.addSlider("Servo1",1000,2000,1500,0,0,100,10);servoSliderH2.setDecimalPrecision(0);
  servoSliderH3 = controlP5.addSlider("Servo2",1000,2000,1500,0,0,100,10);servoSliderH3.setDecimalPrecision(0);
  servoSliderH4 = controlP5.addSlider("Servo3",1000,2000,1500,0,0,100,10);servoSliderH4.setDecimalPrecision(0);
  servoSliderV0  = controlP5.addSlider("Servov0",1000,2000,1500,0,0,10,100);servoSliderV0.setDecimalPrecision(0);
  servoSliderV1  = controlP5.addSlider("Servov1",1000,2000,1500,0,0,10,100);servoSliderV1.setDecimalPrecision(0);
  servoSliderV2 = controlP5.addSlider("Servov2",1000,2000,1500,0,0,10,100);servoSliderV2.setDecimalPrecision(0);

  scaleSlider = controlP5.addSlider("SCALE",0,10,1,xGraph+400,yGraph-25,150,20);
 
  confPowerTrigger = controlP5.addNumberbox("",0,xGraph+50,yGraph-29,40,14);confPowerTrigger.setDecimalPrecision(0);confPowerTrigger.setMultiplier(10);
  confPowerTrigger.setDirection(Controller.HORIZONTAL);confPowerTrigger.setMin(0);confPowerTrigger.setMax(65535);confPowerTrigger.setColorBackground(red_);
}

void draw() {
  int i;
  float val,inter,a,b,h;
 
  background(80);
  textFont(font15);
  text("multiwii.com",0,16);text("v1.dev", 0, 32);
  text("Cycle Time:",xGraph+220,yGraph-10);text(cycleTime,xGraph+320,yGraph-10);

  textFont(font12);
  text("Power:",xGraph-5,yGraph-30); text(pMeterSum,xGraph+50,yGraph-30);
  text("pAlarm:",xGraph-5,yGraph-15);
  text("Volt:",xGraph-5,yGraph-2);  text(bytevbat/10.0,xGraph+50,yGraph-2);

  text("DIST HOME :",xGraph-8,yGraph+185+8);
  text(GPS_distanceToHome,xGraph+70,yGraph+185+8);
  if(GPS_fix == 0) fill(255,100,100); else fill(0,128,0);
  text("NUM SAT     :",xGraph-8,yGraph+185+23);
  text(GPS_numSat,xGraph+70,yGraph+185+23);
  fill(255,255,255);

  time1=millis();
  if (init_com==1 && (time1-time2)>50 && graph_on==1) {
    g_serial.write('M');
    time2=time1;
  }


  axSlider.setValue(ax);aySlider.setValue(ay);azSlider.setValue(az);gxSlider.setValue(gx);gySlider.setValue(gy);gzSlider.setValue(gz);
  baroSlider.setValue(baro/10);headSlider.setValue(head);magxSlider.setValue(magx);magySlider.setValue(magy);magzSlider.setValue(magz);
  debug1Slider.setValue(debug1/10);debug2Slider.setValue(debug2);debug3Slider.setValue(debug3);debug4Slider.setValue(debug4);

  motSliderV0.setValue(mot[0]);motSliderV1.setValue(mot[1]);motSliderV2.setValue(mot[2]);motSliderV3.setValue(mot[3]);motSliderV4.setValue(mot[4]);motSliderV5.setValue(mot[5]);

  servoSliderH1.setValue(servo0);servoSliderH2.setValue(servo1);servoSliderH3.setValue(servo2);servoSliderH4.setValue(servo3);
  servoSliderV0.setValue(servo0);servoSliderV1.setValue(servo1);servoSliderV2.setValue(servo2);

  rcStickThrottleSlider.setValue(rcThrottle);rcStickRollSlider.setValue(rcRoll);rcStickPitchSlider.setValue(rcPitch);rcStickYawSlider.setValue(rcYaw);
  rcStickAUX1Slider.setValue(rcAUX1);rcStickAUX2Slider.setValue(rcAUX2);rcStickCAM1Slider.setValue(rcCAM1);rcStickCAM2Slider.setValue(rcCAM2);

  stroke(255); 
  a=radians(angx);
  if (angy<-90) b=radians(-180 - angy);
  else if (angy>90) b=radians(+180 - angy);
  else b=radians(angy);
  h=radians(head);

  float size = 30.0;

  pushMatrix();
  camera(xObj,yObj,300/tan(PI*60.0/360.0),xObj/2+30,yObj/2-40,0,0,1,0);
  translate(xObj,yObj);
  directionalLight(200,200,200, 0, 0, -1);
  rotateZ(h);rotateX(b);rotateY(a);
  stroke(150,255,150);
  strokeWeight(0);sphere(size/3);strokeWeight(3);
  line(0,0, 10,0,-size-5,10);line(0,-size-5,10,+size/4,-size/2,10); line(0,-size-5,10,-size/4,-size/2,10);
  stroke(255);
 
  motSliderV0.hide(); motSliderV1.hide(); motSliderV2.hide(); motSliderV3.hide(); motSliderV4.hide(); motSliderV5.hide();
  servoSliderH1.hide(); servoSliderH2.hide(); servoSliderH3.hide(); servoSliderH4.hide(); servoSliderV0.hide(); servoSliderV1.hide(); servoSliderV2.hide();
 
  textFont(font12);
  if (multiType == 1) { //TRI
    ellipse(-size, -size, size, size);ellipse(+size, -size, size, size);ellipse(0,  +size,size, size);
    line(-size,-size, 0,0);line(+size,-size, 0,0);line(0,+size, 0,0);
    noLights();text(" TRICOPTER", -40,-50);camera();popMatrix();
 
    motSliderV0.setPosition(xMot+50,yMot+15);motSliderV0.setHeight(100);motSliderV0.setCaptionLabel("REAR");motSliderV0.show();
    motSliderV1.setPosition(xMot+100,yMot-15);motSliderV1.setHeight(100);motSliderV1.setCaptionLabel("RIGHT");motSliderV1.show();
    motSliderV2.setPosition(xMot,yMot-15);motSliderV2.setHeight(100);motSliderV2.setCaptionLabel("LEFT");motSliderV2.show();
    servoSliderH1.setPosition(xMot,yMot+135);servoSliderH1.setCaptionLabel("SERVO");servoSliderH1.show(); 
  } else if (multiType == 2) { //QUAD+
    ellipse(0,  -size,   size,size);ellipse(0,  +size, size, size);ellipse(+size, 0,  size , size );ellipse(-size, 0,  size , size );
    line(-size,0, +size,0);line(0,-size, 0,+size);
    noLights();text("QUADRICOPTER +", -40,-50);camera();popMatrix();
    
    motSliderV0.setPosition(xMot+50,yMot+75);motSliderV0.setHeight(60);motSliderV0.setCaptionLabel("REAR");motSliderV0.show();
    motSliderV1.setPosition(xMot+100,yMot+35);motSliderV1.setHeight(60);motSliderV1.setCaptionLabel("RIGHT");motSliderV1.show();
    motSliderV2.setPosition(xMot,yMot+35);motSliderV2.setHeight(60);motSliderV2.setCaptionLabel("LEFT");motSliderV2.show();
    motSliderV3.setPosition(xMot+50,yMot-15);motSliderV3.setHeight(60);motSliderV3.setCaptionLabel("FRONT");motSliderV3.show();
  } else if (multiType == 3) { //QUAD X
    ellipse(-size,  -size, size, size);ellipse(+size,  -size, size, size);ellipse(-size,  +size, size, size);ellipse(+size,  +size, size, size);
    line(-size,-size, 0,0);line(+size,-size, 0,0);line(-size,+size, 0,0);line(+size,+size, 0,0);
    noLights();text("QUADRICOPTER X", -40,-50);camera();popMatrix();
    
    motSliderV0.setPosition(xMot+90,yMot+75);motSliderV0.setHeight(60);motSliderV0.setCaptionLabel("REAR_R");motSliderV0.show();
    motSliderV1.setPosition(xMot+90,yMot-15);motSliderV1.setHeight(60);motSliderV1.setCaptionLabel("FRONT_R");motSliderV1.show();
    motSliderV2.setPosition(xMot+10,yMot+75);motSliderV2.setHeight(60);motSliderV2.setCaptionLabel("REAR_L");motSliderV2.show();
    motSliderV3.setPosition(xMot+10,yMot-15);motSliderV3.setHeight(60);motSliderV3.setCaptionLabel("FRONT_L");motSliderV3.show(); 
  } else if (multiType == 4) { //BI
    ellipse(0-size,  0,   size, size);ellipse(0+size,  0,   size, size);
    line(0-size,0, 0,0);  line(0+size,0, 0,0);line(0,size*1.5, 0,0);
    noLights();text("BICOPTER", -30,-20);camera();popMatrix();
   
    motSliderV0.setPosition(xMot,yMot+30);motSliderV0.setHeight(55);motSliderV0.setCaptionLabel("");motSliderV0.show();
    motSliderV1.setPosition(xMot+100,yMot+30);motSliderV1.setHeight(55);motSliderV1.setCaptionLabel("");motSliderV1.show();
    servoSliderH1.setPosition(xMot,yMot+100);servoSliderH1.setWidth(60);servoSliderH1.setCaptionLabel("");servoSliderH1.show();
    servoSliderH2.setPosition(xMot+80,yMot+100);servoSliderH2.setWidth(60);servoSliderH2.setCaptionLabel("");servoSliderH2.show();
  } else if (multiType == 5) { //GIMBAL
    noLights();text("GIMBAL", -20,-10);camera();popMatrix();
    text("GIMBAL", xMot,yMot+25);
 
    servoSliderH3.setPosition(xMot,yMot+75);servoSliderH3.setCaptionLabel("ROLL");servoSliderH3.show();
    servoSliderH2.setPosition(xMot,yMot+35);servoSliderH2.setCaptionLabel("PITCH");servoSliderH2.show();
  } else if (multiType == 6) { //Y6
    ellipse(-size,-size,size,size);ellipse(size,-size,size,size);ellipse(0,-2+size,size,size);
    translate(0,0,7);
    ellipse(-5-size,-5-size,size,size);ellipse(5+size,-5-size,size,size);ellipse(0,3+size,size,size);
    line(-size,-size,0,0);line(+size,-size, 0,0);line(0,+size, 0,0);
    noLights();text("TRICOPTER Y6", -40,-55);camera();popMatrix();

    motSliderV0.setPosition(xMot+50,yMot+23);motSliderV0.setHeight(50);motSliderV0.setCaptionLabel("REAR");motSliderV0.show();
    motSliderV1.setPosition(xMot+100,yMot-18);motSliderV1.setHeight(50);motSliderV1.setCaptionLabel("RIGHT");motSliderV1.show();
    motSliderV2.setPosition(xMot,yMot-18);motSliderV2.setHeight(50);motSliderV2.setCaptionLabel("LEFT");motSliderV2.show();
    motSliderV3.setPosition(xMot+50,yMot+87);motSliderV3.setHeight(50);motSliderV3.setCaptionLabel("U_REAR");motSliderV3.show();
    motSliderV4.setPosition(xMot+100,yMot+48);motSliderV4.setHeight(50);motSliderV4.setCaptionLabel("U_RIGHT");motSliderV4.show();
    motSliderV5.setPosition(xMot,yMot+48);motSliderV5.setHeight(50);motSliderV5.setCaptionLabel("U_LEFT");motSliderV5.show();
  } else if (multiType == 7) { //HEX6
    ellipse(-size,-0.55*size,size,size);ellipse(size,-0.55*size,size,size);ellipse(-size,+0.55*size,size,size);
    ellipse(size,+0.55*size,size,size);ellipse(0,-size,size,size);ellipse(0,+size,size,size);
    line(-size,-0.55*size,0,0);line(size,-0.55*size,0,0);line(-size,+0.55*size,0,0);line(size,+0.55*size,0,0);line(0,+size,0,0);line(0,-size,0,0);
    noLights();text("HEXACOPTER", -40,-50);camera();popMatrix();

    motSliderV0.setPosition(xMot+90,yMot+65);motSliderV0.setHeight(50);motSliderV0.setCaptionLabel("REAR_R");motSliderV0.show();
    motSliderV1.setPosition(xMot+90,yMot-5);motSliderV1.setHeight(50);motSliderV1.setCaptionLabel("FRONT_R");motSliderV1.show();
    motSliderV2.setPosition(xMot+5,yMot+65);motSliderV2.setHeight(50);motSliderV2.setCaptionLabel("REAR_L");motSliderV2.show();
    motSliderV3.setPosition(xMot+5,yMot-5);motSliderV3.setHeight(50);motSliderV3.setCaptionLabel("FRONT_L");motSliderV3.show(); 
    motSliderV4.setPosition(xMot+50,yMot-20);motSliderV4.setHeight(50);motSliderV4.setCaptionLabel("FRONT");motSliderV4.show(); 
    motSliderV5.setPosition(xMot+50,yMot+90);motSliderV5.setHeight(50);motSliderV5.setCaptionLabel("REAR");motSliderV5.show(); 
  } else if (multiType == 8) { //FLYING_WING
    line(0,0, 1.8*size,size);line(1.8*size,size,1.8*size,size-30);  line(1.8*size,size-30,0,-1.5*size);
    line(0,0, -1.8*size,+size);line(-1.8*size,size,-1.8*size,+size-30);    line(-1.8*size,size-30,0,-1.5*size);
    noLights();text("FLYING WING", -40,-50);camera();popMatrix();

    servoSliderV1.setPosition(xMot+5,yMot+10);servoSliderV1.setCaptionLabel("LEFT");servoSliderV1.show(); 
    servoSliderV2.setPosition(xMot+100,yMot+10);servoSliderV2.setCaptionLabel("RIGHT");servoSliderV2.show();
    motSliderV0.setPosition(xMot+50,yMot+30);motSliderV0.setHeight(90);motSliderV0.setCaptionLabel("Mot");motSliderV0.show();
  } else if (multiType == 9) { //Y4
    ellipse(-size,  -size, size, size);ellipse(+size,  -size, size, size);ellipse(0,  +size, size+2, size+2);
    line(-size,-size, 0,0);line(+size,-size, 0,0);line(0,+size, 0,0);
    translate(0,0,7);
    ellipse(0,  +size, size, size);
    noLights();text("Y4", -5,-50);camera();popMatrix();
    
    motSliderV0.setPosition(xMot+80,yMot+75);motSliderV0.setHeight(60);motSliderV0.setCaptionLabel("REAR_1");motSliderV0.show();
    motSliderV1.setPosition(xMot+90,yMot-15);motSliderV1.setHeight(60);motSliderV1.setCaptionLabel("FRONT_R");motSliderV1.show();
    motSliderV2.setPosition(xMot+30,yMot+75);motSliderV2.setHeight(60);motSliderV2.setCaptionLabel("REAR_2");motSliderV2.show();
    motSliderV3.setPosition(xMot+10,yMot-15);motSliderV3.setHeight(60);motSliderV3.setCaptionLabel("FRONT_L");motSliderV3.show(); 
  } else if (multiType == 10) { //HEX6 X
    ellipse(-0.55*size,-size,size,size);ellipse(-0.55*size,size,size,size);ellipse(+0.55*size,-size,size,size);
    ellipse(+0.55*size,size,size,size);ellipse(-size,0,size,size);ellipse(+size,0,size,size);
    line(-0.55*size,-size,0,0);line(-0.55*size,size,0,0);line(+0.55*size,-size,0,0);line(+0.55*size,size,0,0);line(+size,0,0,0);  line(-size,0,0,0);
    noLights();text("HEXACOPTER X", -45,-50);camera();popMatrix();

    motSliderV0.setPosition(xMot+80,yMot+90);motSliderV0.setHeight(45);motSliderV0.setCaptionLabel("REAR_R");motSliderV0.show();
    motSliderV1.setPosition(xMot+80,yMot-20);motSliderV1.setHeight(45);motSliderV1.setCaptionLabel("FRONT_R");motSliderV1.show();
    motSliderV2.setPosition(xMot+25,yMot+90);motSliderV2.setHeight(45);motSliderV2.setCaptionLabel("REAR_L");motSliderV2.show();
    motSliderV3.setPosition(xMot+25,yMot-20);motSliderV3.setHeight(45);motSliderV3.setCaptionLabel("FRONT_L");motSliderV3.show(); 
    motSliderV4.setPosition(xMot+90,yMot+35);motSliderV4.setHeight(45);motSliderV4.setCaptionLabel("RIGHT");motSliderV4.show(); 
    motSliderV5.setPosition(xMot+5,yMot+35);motSliderV5.setHeight(45);motSliderV5.setCaptionLabel("LEFT");motSliderV5.show(); 
  } else if (multiType == 11) { //OCTOX8
    noLights();text("OCTOCOPTER X8", -45,-50);camera();popMatrix();
  } else {
    noLights();camera();popMatrix();
  }
  
  pushMatrix();
  translate(xObj+60,yObj-165);
  rotate(a);
  textFont(font15);text("ROLL", -20, 15);
  line(-30,0,+30,0);line(0,0,0,-10);
  popMatrix();
  
  pushMatrix();
  translate(xObj+60,yObj-100);
  rotate(b);
  textFont(font15);text("PITCH", -30, 15);
  line(-30,0,30,0);line(+30,0,30-size/3 ,size/3);line(+30,0,30-size/3 ,-size/3);  
  popMatrix();
 
  pushMatrix();
  translate(xObj-20,yObj-133);

  size=15;
  strokeWeight(1.5);
  if (GPS_update == 1) {
    fill(125);stroke(125);
  } else {
    fill(160);stroke(160);
  }
  ellipse(0,  0,   4*size+7, 4*size+7);

  rotate(GPS_directionToHome*PI/180);
  strokeWeight(4);stroke(200);line(0,0, 0,-3*size);line(0,-3*size, -5 ,-3*size+10); line(0,-3*size, +5 ,-3*size+10);  
  rotate(-GPS_directionToHome*PI/180);

  strokeWeight(1.5);fill(0);stroke(0);ellipse(0,  0,   2*size+7, 2*size+7);

  stroke(255);

  rotate(head*PI/180);
  line(0,size, 0,-size); line(0,-size, -5 ,-size+10); line(0,-size, +5 ,-size+10);
  popMatrix();
  text("N",xObj-25,yObj-155);text("S",xObj-25,yObj-100);
  text("W",xObj-53,yObj-127);text("E",xObj   ,yObj-127);

  strokeWeight(1);
  fill(255, 255, 255);
  g_graph.drawGraphBox();
  
  strokeWeight(1.5);
  stroke(255, 0, 0); if (axGraph) g_graph.drawLine(accROLL, -1000, +1000);
  stroke(0, 255, 0); if (ayGraph) g_graph.drawLine(accPITCH, -1000, +1000);
  stroke(0, 0, 255);
  if (azGraph) {
    if (scaleSlider.value()<2) g_graph.drawLine(accYAW, -1000, +1000);
    else g_graph.drawLine(accYAW, 200*scaleSlider.value()-1000,200*scaleSlider.value()+500);
  }
  
  float BaroMin = (baroData.getMinVal() + baroData.getRange() / 2) - 10;
  float BaroMax = (baroData.getMaxVal() + baroData.getRange() / 2) + 10;

  stroke(200, 200, 0);  if (gxGraph)   g_graph.drawLine(gyroROLL, -300, +300);
  stroke(0, 255, 255);  if (gyGraph)   g_graph.drawLine(gyroPITCH, -300, +300);
  stroke(255, 0, 255);  if (gzGraph)   g_graph.drawLine(gyroYAW, -300, +300);
  stroke(125, 125, 125);if (baroGraph) g_graph.drawLine(baroData, BaroMin, BaroMax);
  stroke(225, 225, 125);if (headGraph)  g_graph.drawLine(headData, -370, +370);
  stroke(50, 100, 150); if (magxGraph) g_graph.drawLine(magxData, -500, +500);
  stroke(100, 50, 150); if (magyGraph) g_graph.drawLine(magyData, -500, +500);
  stroke(150, 100, 50); if (magzGraph) g_graph.drawLine(magzData, -500, +500);

  stroke(0, 0, 0);
  if (debug1Graph)  g_graph.drawLine(debug1Data, BaroMin, BaroMax);
  if (debug2Graph)  g_graph.drawLine(debug2Data, -5000, +5000);
  if (debug3Graph)  g_graph.drawLine(debug3Data, -5000, +5000);
  if (debug4Graph)  g_graph.drawLine(debug4Data, -5000, +5000);

  fill(0, 0, 0);

  strokeWeight(3);stroke(0);
  rectMode(CORNERS);
  rect(xMot-5,yMot-20, xMot+145, yMot+150);
  rect(xRC-5,yRC-5, xRC+185, yRC+235);
  rect(xParam,yParam, xParam+355, yParam+280);

  int xSens       = xParam + 80;
  int ySens       = yParam + 210;
  stroke(255);
  a=min(confRC_RATE.value(),1);
  b=confRC_EXPO.value();
  strokeWeight(1);
  line(xSens,ySens,xSens,ySens+40);
  line(xSens,ySens+40,xSens+70,ySens+40);
  strokeWeight(3);stroke(30,120,30);
  for(i=0;i<70;i++) {
    inter = 10*i;
    val = a*inter*(1-b+inter*inter*b/490000);
    point(xSens+i,ySens+(70-val/10)*4/7);
  }
  if (confRC_RATE.value()>1) { 
    stroke(220,100,100);
    ellipse(xSens+70, ySens, 7, 7);
  }
  
  fill(255);
  textFont(font15);    
  text("P",xParam+45,yParam+15);text("I",xParam+90,yParam+15);text("D",xParam+130,yParam+15);
  textFont(font12);
  text("    RC",xParam+3,yParam+220);
  text("RATE",xParam+3,yParam+232);
  text("EXPO",xParam+3,yParam+250);
  text("RATE",xParam+160,yParam+15);
  text("ROLL",xParam+3,yParam+32);text("PITCH",xParam+3,yParam+52);text("YAW",xParam+3,yParam+72);
  text("ALT",xParam+3,yParam+92);
  text("VEL",xParam+3,yParam+112);
  text("LEVEL",xParam+1,yParam+132);
  text("MAG",xParam+3,yParam+152); 
  text("Throttle PID",xParam+220,yParam+15);text("attenuation",xParam+220,yParam+30);
  text("AUX1",xBox+55,yBox+5);text("AUX2",xParam+295,yBox+5);
  text("LEVEL",xBox,yBox+30);
  text("BARO",xBox,yBox+43);
  text("MAG",xBox,yBox+56);
  text("ARM",xBox,yBox+95);
  textFont(font8);
  text("CAMSTAB",xBox-5,yBox+69);
  text("CAMTRIG",xBox-5,yBox+82);
  text("GPS HOME",xBox-5,yBox+108);
  text("GPS HOLD",xBox-5,yBox+121);
  text("LOW",xBox+37,yBox+15);text("MID",xBox+57,yBox+15);text("HIGH",xBox+74,yBox+15);
  text("LOW",xBox+100,yBox+15);text("MID",xBox+123,yBox+15);text("HIGH",xBox+140,yBox+15);
}

void ACC_ROLL(boolean theFlag) {axGraph = theFlag;}
void ACC_PITCH(boolean theFlag) {ayGraph = theFlag;}
void ACC_Z(boolean theFlag) {azGraph = theFlag;}
void GYRO_ROLL(boolean theFlag) {gxGraph = theFlag;}
void GYRO_PITCH(boolean theFlag) {gyGraph = theFlag;}
void GYRO_YAW(boolean theFlag) {gzGraph = theFlag;}
void BARO(boolean theFlag) {baroGraph = theFlag;}
void HEAD(boolean theFlag) {headGraph = theFlag;}
void MAGX(boolean theFlag) {magxGraph = theFlag;}
void MAGY(boolean theFlag) {magyGraph = theFlag;}
void MAGZ(boolean theFlag) {magzGraph = theFlag;}
void DEBUG1(boolean theFlag) {debug1Graph = theFlag;}
void DEBUG2(boolean theFlag) {debug2Graph = theFlag;}
void DEBUG3(boolean theFlag) {debug3Graph = theFlag;}
void DEBUG4(boolean theFlag) {debug4Graph = theFlag;}

public void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup()) if (theEvent.name()=="portComList") InitSerial(theEvent.group().value()); // initialize the serial port selected
}

public void bSTART() {
  if(graphEnable == false) {return;}
  graph_on=1;
  readEnable = true;calibrateEnable = true;
  buttonREAD.setColorBackground(green_); buttonCALIBRATE_ACC.setColorBackground(green_); buttonCALIBRATE_MAG.setColorBackground(green_);
  g_serial.clear();
}

public void bSTOP() {
  graph_on=0;
}

public void READ() {
  if(readEnable == false) {return;}
  for(int i=0;i<5;i++) {confP[i].setValue(byteP[i]/10.0);confI[i].setValue(byteI[i]/1000.0);confD[i].setValue(byteD[i]);}
  confP[LEVEL].setValue(byteP[LEVEL]/10.0);confI[LEVEL].setValue(byteI[LEVEL]/1000.0);
  confP[MAG].setValue(byteP[MAG]/10.0);
  confRC_RATE.setValue(byteRC_RATE/50.0);
  confRC_EXPO.setValue(byteRC_EXPO/100.0);
  rollPitchRate.setValue(byteRollPitchRate/100.0);
  yawRate.setValue(byteYawRate/100.0);

  dynamic_THR_PID.setValue(byteDynThrPID/100.0);

  buttonWRITE.setColorBackground(green_);

  for(int i=0;i<7;i++) {confP[i].setColorBackground(green_);}
  for(int i=0;i<6;i++) {confI[i].setColorBackground(green_);}
  for(int i=0;i<5;i++) {confD[i].setColorBackground(green_);}
  
  confRC_RATE.setColorBackground(green_);confRC_EXPO.setColorBackground(green_);rollPitchRate.setColorBackground(green_);yawRate.setColorBackground(green_);dynamic_THR_PID.setColorBackground(green_);

  for(int i=0;i<8;i++) for(int a=0;a<6;a++)
    if ((byte(activation[i])&(1<<a))>0) checkbox[i].activate(a); else checkbox[i].deactivate(a);

  confPowerTrigger.setValue(intPowerTrigger);

  writeEnable = true;  
}

public void WRITE() {
  if(writeEnable == false) {return;}
  for(int i=0;i<7;i++) {byteP[i] = (round(confP[i].value()*10));}
  for(int i=0;i<6;i++) {byteI[i] = (round(confI[i].value()*1000));}
  for(int i=0;i<5;i++) {byteD[i] = (round(confD[i].value()));}

  byteRC_RATE = (round(confRC_RATE.value()*50));
  byteRC_EXPO = (round(confRC_EXPO.value()*100));
  byteRollPitchRate = (round(rollPitchRate.value()*100));
  byteYawRate = (round(yawRate.value()*100));
  byteDynThrPID = (round(dynamic_THR_PID.value()*100));

  for(int i=0;i<8;i++) {
    activation[i] = 0;
    for(int a=0;a<6;a++) activation[i] += (int)(checkbox[i].arrayValue()[a]*(1<<a));
  }
  
  intPowerTrigger = (round(confPowerTrigger.value()));

  int[] s = new int[34];
  int p = 0;
   s[p++] = 'W'; //0 write to Eeprom @ arduino //1
   for(int i=0;i<5;i++) {s[p++] = byteP[i];  s[p++] = byteI[i];  s[p++] =  byteD[i];} //16
   s[p++] = byteP[LEVEL]; s[p++] = byteI[LEVEL]; 
   s[p++] = byteP[MAG]; 
   s[p++] = byteRC_RATE; s[p++] = byteRC_EXPO; 
   s[p++] = byteRollPitchRate; 
   s[p++] = byteYawRate;
   s[p++] = byteDynThrPID; //24
   for(int i=0;i<8;i++) s[p++] = activation[i]; //32
   s[p++] = intPowerTrigger;
   s[p++] = intPowerTrigger >>8 &0xff; //34
   for(int i =0;i<34;i++)    g_serial.write(char(s[i]));
}

public void CALIB_ACC() {
  if(calibrateEnable == false) {return;}
  g_serial.write('S'); // acc Sensor calibration request
}
public void CALIB_MAG() {
  if(calibrateEnable == false) {return;}
  g_serial.write('E'); // mag Sensor calibration request
}

// initialize the serial port selected in the listBox
void InitSerial(float portValue) {
  String portPos = Serial.list()[int(portValue)];
  txtlblWhichcom.setValue("COM = " + shortifyPortName(portPos, 8));
  g_serial = new Serial(this, portPos, 115200);
  init_com=1;
  buttonSTART.setColorBackground(green_);buttonSTOP.setColorBackground(green_);commListbox.setColorBackground(green_);
  graphEnable = true;
  g_serial.buffer(frame_size+1);
}

int p;
byte[] inBuf = new byte[frame_size];

int read16() {return (inBuf[p++]&0xff) + (inBuf[p++]<<8);}
int read8()  {return inBuf[p++]&0xff;}

void serialEvent(Serial p) { 
  processSerialData(); 
}

void processSerialData() {
  int present=0,mode=0;

  if (g_serial.read() == 'M') {
    g_serial.readBytes(inBuf);
    if (inBuf[frame_size-1] == 'M') {  // Multiwii @ arduino send all data to GUI
      p=0;
      read8(); //version                                                              //1
      ax = read16();ay = read16();az = read16();
      gx = read16();gy = read16();gz = read16();                                      //13
      magx = read16();magy = read16();magz = read16();                                //19
      baro = read16();
      head = read16();                                                                 //23
      servo0 = read16();servo1 = read16();servo2 = read16();servo3 = read16();        //31
      for(int i=0;i<8;i++) mot[i] = read16();                                         //47
      rcRoll = read16();rcPitch = read16();rcYaw = read16();rcThrottle = read16();    
      rcAUX1 = read16();rcAUX2 = read16();rcCAM1 = read16();rcCAM2 = read16();        //63
      present = read8(); 
      mode = read8();
      cycleTime = read16();
      angx = read16();angy = read16();
      multiType = read8();                                                            //72
      for(int i=0;i<5;i++) {byteP[i] = read8();byteI[i] = read8();byteD[i] = read8();}//87
      byteP[LEVEL] = read8();byteI[LEVEL] = read8();                                  //89
      byteP[MAG] = read8(); 
      byteRC_RATE = read8();
      byteRC_EXPO = read8();
      byteRollPitchRate = read8();
      byteYawRate = read8();
      byteDynThrPID = read8();                                                        //95
      for(int i=0;i<8;i++) activation[i] = read8();                                   //102
      GPS_distanceToHome = read16();
      GPS_directionToHome = read16();
      GPS_numSat = read8();
      GPS_fix = read8();
      GPS_update = read8();
      pMeterSum = read16();
      intPowerTrigger = read16();
      bytevbat = read8();
      debug1 = read16();debug2 = read16();debug3 = read16();debug4 = read16();
      
      if ((present&1) >0) nunchukPresent = 1;    else  nunchukPresent = 0;
      if ((present&2) >0) i2cAccPresent = 1;     else  i2cAccPresent = 0;
      if ((present&4) >0) i2cBaroPresent = 1;    else  i2cBaroPresent = 0;
      if ((present&8) >0) i2cMagnetoPresent = 1; else  i2cMagnetoPresent = 0;
      if ((present&16)>0) GPSPresent = 1;        else  GPSPresent = 0;
      
      if ((mode&1) >0) {buttonI2cAccActive.setCaptionLabel("ACTIVE");buttonI2cAccActive.setColorBackground(green_);}
      else {buttonI2cAccActive.setCaptionLabel("OFF");buttonI2cAccActive.setColorBackground(red_);}
 
      if ((mode&2) >0) {buttonI2cBaroActive.setCaptionLabel("ACTIVE");buttonI2cBaroActive.setColorBackground(green_);}
      else {buttonI2cBaroActive.setCaptionLabel("OFF");buttonI2cBaroActive.setColorBackground(red_);}

      if ((mode&4) >0) {buttonI2cMagnetoActive.setCaptionLabel("ACTIVE");buttonI2cMagnetoActive.setColorBackground(green_);}
      else {buttonI2cMagnetoActive.setCaptionLabel("OFF");buttonI2cMagnetoActive.setColorBackground(red_);}

      if ((mode&8) >0) {buttonGPSActive.setCaptionLabel("ACTIVE");buttonGPSActive.setColorBackground(green_);}
      else {buttonGPSActive.setCaptionLabel("OFF");buttonGPSActive.setColorBackground(red_);}

      if (nunchukPresent>0) {buttonNunchuk.setColorBackground(green_);} else {buttonNunchuk.setColorBackground(red_);}
      if (i2cAccPresent>0) {buttonI2cAcc.setColorBackground(green_);} else {buttonI2cAcc.setColorBackground(red_);}
      if (i2cBaroPresent>0) {buttonI2cBaro.setColorBackground(green_);} else {buttonI2cBaro.setColorBackground(red_);}
      if (i2cMagnetoPresent>0) {buttonI2cMagneto.setColorBackground(green_);} else {buttonI2cMagneto.setColorBackground(red_);}
      if (GPSPresent>0) {buttonGPS.setColorBackground(green_);} else {buttonGPS.setColorBackground(red_);}

      accROLL.addVal(ax);accPITCH.addVal(ay);accYAW.addVal(az);gyroROLL.addVal(gx);gyroPITCH.addVal(gy);gyroYAW.addVal(gz);
      baroData.addVal(baro);headData.addVal(head);magxData.addVal(magx);magyData.addVal(magy);magzData.addVal(magz);
      debug1Data.addVal(debug1);debug2Data.addVal(debug2);debug3Data.addVal(debug3);debug4Data.addVal(debug4);
    }
  } else g_serial.readStringUntil('M');
}


//********************************************************
//********************************************************
//********************************************************

class cDataArray {
  float[] m_data;
  int m_maxSize, m_startIndex = 0, m_endIndex = 0, m_curSize;
  
  cDataArray(int maxSize){
    m_maxSize = maxSize;
    m_data = new float[maxSize];
  }
  void addVal(float val) {
    m_data[m_endIndex] = val;
    m_endIndex = (m_endIndex+1)%m_maxSize;
    if (m_curSize == m_maxSize) {
      m_startIndex = (m_startIndex+1)%m_maxSize;
    } else {
      m_curSize++;
    }
  }
  float getVal(int index) {return m_data[(m_startIndex+index)%m_maxSize];}
  int getCurSize(){return m_curSize;}
  int getMaxSize() {return m_maxSize;}
  float getMaxVal() {
    float res = 0.0;
    for(int i=0; i<m_curSize-1; i++) if ((m_data[i] > res) || (i==0)) res = m_data[i];
    return res;
  }
  float getMinVal() {
    float res = 0.0;
    for(int i=0; i<m_curSize-1; i++) if ((m_data[i] < res) || (i==0)) res = m_data[i];
    return res;
  }
  float getRange() {return getMaxVal() - getMinVal();}
}

// This class takes the data and helps graph it
class cGraph {
  float m_gWidth, m_gHeight, m_gLeft, m_gBottom, m_gRight, m_gTop;
  
  cGraph(float x, float y, float w, float h) {
    m_gWidth     = w; m_gHeight    = h;
    m_gLeft      = x; m_gBottom    = y;
    m_gRight     = x + w;
    m_gTop       = y + h;
  }
  
  void drawGraphBox() {
    stroke(0, 0, 0);
    rectMode(CORNERS);
    rect(m_gLeft, m_gBottom, m_gRight, m_gTop);
  }
  
  void drawLine(cDataArray data, float minRange, float maxRange) {
    float graphMultX = m_gWidth/data.getMaxSize();
    float graphMultY = m_gHeight/(maxRange-minRange);
    
    for(int i=0; i<data.getCurSize()-1; ++i) {
      float x0 = i*graphMultX+m_gLeft;
      float y0 = m_gTop-(((data.getVal(i)-(maxRange+minRange)/2)*scaleSlider.value()+(maxRange-minRange)/2)*graphMultY);
      float x1 = (i+1)*graphMultX+m_gLeft;
      float y1 = m_gTop-(((data.getVal(i+1)-(maxRange+minRange)/2 )*scaleSlider.value()+(maxRange-minRange)/2)*graphMultY);
      line(x0, y0, x1, y1);
    }
  }
}
