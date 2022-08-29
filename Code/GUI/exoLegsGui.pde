import controlP5.*;  //for graphics
import processing.serial.*;  //for serial comms with teensy via usb

Serial teensyPort;  //serial port object for usb comms with teensy
int teensyPortTimer = 0;  //how long since data recieved. checks port connection
int reconnectAttempts = 0;  //repeated failed attempts throws error
String[] serialValues = new String[22];  //data from serial split into strings
boolean[] serialBoolValues = new boolean[4];  // == {homed, programRunning, isError, allJointsReady}
String[] speedWeightValues = new String[2];  // == walkingSpeed, bodyWeightPercent
String[] errorString = new String[5];  // == {joint name, motorError, encoderError, axisError, index}
String fromTeensy;
char firstChar;
boolean handShake = false;  //contact with teensy
boolean firstContact = false;  //first time made serial connection
boolean homed = false;  //have all joints been homed
boolean moving = false;  //are legs moving
boolean modeSet = false;
boolean readyToAdvance = false;  //are all joints at target position and ready to move to next position
boolean oldErrorVal = false;  //was there previously an error?
boolean firstError = true;  //when recieving errors, is this the first one?
int timer = millis();
int updateTimer = 200;  //interval to update timer in ms

float motorKv = 340.0;  //motor kv

//array of angles for each leg joint
float[] hipAngles = {19.33, 18.92, 18.45, 17.94, 17.3, 16.4, 15.18, 13.67, 11.97, 10.21, 8.48, 6.74, 4.94, 3.13, 1.42,
                     -0.13, -1.54, -2.87, -4.12, -5.3, -6.4, -7.43, -8.39, -9.27, -10.02, -10.61, -10.95, -10.91, -10.31,
                     -9.0, -6.95, -4.25, -1.05, 2.42, 5.93, 9.22, 12.11, 14.55, 16.53, 18.13, 19.45, 20.54, 21.38, 21.84,
                     21.87, 21.5, 20.84, 20.09, 19.5, 19.18, 19.01
                    };
float[] kneeAngles = {3.97, 7.0, 10.52, 14.12, 17.38, 19.84, 21.27, 21.67, 21.22, 20.20, 18.86, 17.35, 15.73, 14.08, 12.5,
                      11.09, 9.91, 8.97, 8.28, 7.86, 7.72, 7.94, 8.6, 9.76, 11.5, 13.86, 16.97, 20.96, 26.0, 32.03, 38.74,
                      45.6, 52.05, 57.54, 61.66, 64.12, 64.86, 63.95, 61.59, 57.97, 53.27, 47.58, 40.94, 33.46, 25.38,
                      17.27, 9.94, 4.31, 1.12, 0.54, 2.21
                     };
float[] ankleAngles = {0.02, -2.06, -3.88, -4.6, -3.98, -2.4, -0.45, 1.45, 3.04, 4.27, 5.13, 5.71, 6.1, 6.43, 6.76, 7.12,
                       7.54, 7.99, 8.44, 8.86, 9.23, 9.51, 9.62, 9.43, 8.7, 7.2, 4.69, 1.15, -3.26, -8.17, -13.05, -17.13,
                       -19.52, -19.77, -18.12, -15.29, -12.04, -8.85, -5.96, -3.51, -1.64, -0.5, -0.07, -0.16, -0.42, -0.52,
                       -0.26, 0.36, 1.0, 1.2, 0.58
                      };
                      
//variables for leg motion graphics
float hipX = 400;
float hipY = 180;
float legSegLength = 100;
float footSegLength = 30;

float convertPotToAngle = 6.7;  //NEW POT. CHECK THIS VALUE

float leftGraphsXPos;
float rightGraphsXPos;
int graphWidth = 250;
int leftLegGaitProgress = 50;
int rightLegGaitProgress = 0;

//variables to display joint info
float rightHipTorque = 0;
float rightKneeTorque = 0;
float rightAnkleTorque = 0;
float leftHipTorque = 0;
float leftKneeTorque = 0;
float leftAnkleTorque = 0;
float rightHipSpeed = 0;
float rightKneeSpeed = 0;
float rightAnkleSpeed = 0;
float leftHipSpeed = 0;
float leftKneeSpeed = 0;
float leftAnkleSpeed = 0;
float[] torque = {0, 0, 0, 0, 0, 0};
float[] speed = {0, 0, 0, 0, 0, 0};

float rightLegTarget;

//start angles for joints
float rightHipAngle = hipAngles[0];
float rightKneeAngle = kneeAngles[0];
float rightAnkleAngle = ankleAngles[0];
float leftHipAngle = hipAngles[24];
float leftKneeAngle = kneeAngles[24];
float leftAnkleAngle = ankleAngles[24];

float walkingSpeed = 0.5;
float walkSpeedLimitLow = 0.3;
float walkSpeedLimitHigh = 1.0;
int bodyWeightPercent = 40;  //percent supported. 10% to 80%?
int bodyWeightLimitLow = 10;
int bodyWeightLimitHigh = 80;

//images for graphic interface
PImage leftArrow;
PImage allJointImg;
PImage currentSymbol;
PImage speedometer;

//objects for user interaction
ControlP5 cp5;  //controlp5 object for graphics
Button startStopBtn;
Button homeBtn;
Textlabel moveStatus;
Toggle modeToggle;
Button advanceBtn;
Button slowerBtn;
Button fasterBtn;
Button decreaseWeightBtn;
Button increaseWeightBtn;
Button clearErrorsBtn;
Textarea pseudoConsole;

void setup() {
  frameRate(60);
  printArray(Serial.list());  //prints list of serial ports
  size(1000, 800);  //set size of canvas
  cp5 = new ControlP5(this);  //new controlP5 instance
  PFont arial = createFont("Arial", 14, true);  //set font
  PFont arialBold = createFont("Arial Bold", 14, true);
  leftGraphsXPos = width * 0.05;  //these two must have values set after size()
  rightGraphsXPos = width * 0.72;  //these two must have values set after size()
  
  startStopBtn = cp5.addButton("startStop")  //object for start/stop button
                    .setPosition(20, 20)
                    .setSize(180, 40)
                    .setCaptionLabel("Press to start")  //initial button text
                    .setValue(0);  //movement stopped
  startStopBtn.getCaptionLabel().setFont(arial).setSize(14);
  homeBtn = cp5.addButton("home")  //object for home button
                    .setPosition(210, 20)
                    .setSize(180, 40)
                    .setCaptionLabel("Press to home")  //initial button text
                    .setValue(0);  //not homed
  homeBtn.getCaptionLabel().setFont(arial).setSize(14);
  moveStatus = cp5.addTextlabel("Status")  //object for text displaying move/stop/load status
                    .setPosition(20, 65)
                    .setSize(300, 80)
                    .setText("Connecting...")  //initial text while waiting for handshake with teensy
                    .setColorValue(color(176, 38, 38));
 moveStatus.getValueLabel().setFont(arial).setSize(28);
 modeToggle = cp5.addToggle("modeToggle")  //object for auto/manual  mode toggle
                    .setPosition(475, 30)
                    .setSize(60, 20)
                    .setValue(true)
                    .setMode(ControlP5.SWITCH)
                    .setColorValue(200)
                    .setCaptionLabel("Set Mode");  //label for toggle switch
  modeToggle.getCaptionLabel().setFont(arial).setSize(12);
  advanceBtn = cp5.addButton("Advance")  //object for manual program advance (step by step)
                    .setPosition(630, 20)
                    .setSize(100, 40)
                    .setCaptionLabel("Advance")  //button text
                    .setValue(0);  //manual mode
  advanceBtn.getCaptionLabel().setFont(arial).setSize(14);
  slowerBtn = cp5.addButton("slower")  //object for button to decrease walking speed
                    .setPosition(width * 0.1, height * 0.8)
                    .setSize(40, 60)
                    .setCaptionLabel("<");
  slowerBtn.getCaptionLabel().setFont(arialBold).setSize(32);
  fasterBtn = cp5.addButton("faster")  //object for button to increase walking speed
                    .setPosition(width * 0.25, height * 0.8)
                    .setSize(40, 60)
                    .setCaptionLabel(">");
  fasterBtn.getCaptionLabel().setFont(arialBold).setSize(32);
  decreaseWeightBtn = cp5.addButton("<")  //object for button to decrease weight bearing
                    .setPosition(width * 0.75 - 40, height * 0.8)
                    .setSize(40, 60);
  decreaseWeightBtn.getCaptionLabel().setFont(arialBold).setSize(32);
  increaseWeightBtn = cp5.addButton(">")  //object for button to increase weight bearing
                    .setPosition(width * 0.9 - 40, height * 0.8)
                    .setSize(40, 60);
  increaseWeightBtn.getCaptionLabel().setFont(arialBold).setSize(32);
  clearErrorsBtn = cp5.addButton("clearErrors")  //object for button to clear errors
                    .setPosition(width / 2 + 40, height * 0.8 - 30)
                    .setSize(100, 30)
                    .setValue(1)
                    .setCaptionLabel("Clear errors")
                    .setColorBackground(color(80, 80, 80))  //change button appearance to discourage clicking
                    .setColorForeground(color(80, 80, 80))
                    .setColorActive(color(80, 80, 80))
                    .setColorLabel(color(30, 30, 30));
  clearErrorsBtn.getCaptionLabel().setFont(arialBold).setSize(10);
  pseudoConsole = cp5.addTextarea("pseudoConsole")
                    .setPosition(width / 2 - 140, height * 0.8)
                    .setSize(280, 100)
                    .setColorBackground(50)
                    .setFont(arialBold)
                    .setText("Connecting...");
  
  //objects for images
  leftArrow = loadImage("leftArrow.png");  //object for left arrow image
  currentSymbol = loadImage("currentSymbol.png");
  allJointImg = loadImage("allJointAngles.jpg");
  speedometer = loadImage("speedometer.png");
  
   
  teensyPort = new Serial(this, Serial.list()[1], 115200);  //select serial port from list and set baudrate
  teensyPortTimer = millis();  //time when port opened
  delay(1000);  //wait 1sec
  teensyPort.bufferUntil('\n');
}

void draw() {  //main control loop
  if(millis() - teensyPortTimer > 10000) {  //every 10sec reset handshake flag to test serial connection
    handShake = false;  //reset flag
    if(millis() - teensyPortTimer > 12000) {  //timer reset on every serial connect. if not,
      if(reconnectAttempts > 4) {  //5 failed attempts
        pseudoConsole.setText("Connection timeout. Try restarting");
      } else {
        pseudoConsole.append("\n" + "Serial connection lost. Reconnecting...");
        moveStatus.setColorValue(color(176, 38, 38))
                  .setText("Connecting...");
        reconnectAttempts++;  //counter
        reconnect();  //force attempted reconnect
      }
    }
  }
  
  background(38, 38, 38);  //set background color
  textSize(18);
  textAlign(LEFT);
  fill(200, 200, 200);  //light gray
  text("Console", width / 2 - 140, height * 0.8 - 10);
  
  if(!firstContact || moving) {  //if serial comms not established or joints are moving
    if(frameCount % 30 == 0) {  //framerate is 60fps. This checks if current frame is multiple of 30, every 0.5sec
      if(moveStatus.isVisible()) {  //if text is visible
        moveStatus.setVisible(false);  //make it not
      } else {
        moveStatus.setVisible(true);  //make text visible
      }
    }
  } else {
    moveStatus.setVisible(true);  //make text visible
  }
  
  if(!serialBoolValues[0]) {  //first value is if joints are homed. if not
    homeBtn.setCaptionLabel("Press to home");  //change homeBtn attributes
    homeBtn.setValue(0);
    homed = false;
            /////no start till homed? /////
  } else {  //if joints are homed
    homeBtn.setColorBackground(color(80, 80, 80));  //change button appearance to discourage clicking
    homeBtn.setColorForeground(color(80, 80, 80));
    homeBtn.setColorActive(color(80, 80, 80));
    homeBtn.setColorLabel(color(30, 30, 30));
    homeBtn.setCaptionLabel("Position homed");
    homeBtn.setValue(1);  //homed == true
    if(homed) {  //if homed == true
      moveStatus.setColorValue(color(0, 200, 0));  //green
      moveStatus.setText("Ready to Move");
    } else {  //if not homed
      homed = true;
    }  //end if else
  }  //end first value if else
  
  if(!serialBoolValues[1]) {  //second value is if program is running. if not
    startStopBtn.setCaptionLabel("Press to start");  //change startStopBtn
    startStopBtn.setValue(0);  //not moving
    moveStatus.setColorValue(color(200, 0, 0));  //red
    moveStatus.setText("Motion stopped");  //change status
  } else {  //if program is running
    startStopBtn.setCaptionLabel("Press to stop");  //change startStopBtn
    startStopBtn.setValue(1);  //moving
    moveStatus.setColorValue(color(0, 200, 0));  //green
    moveStatus.setText("Moving...");  //change status
  }  //end second bool value if else
    
  if(serialBoolValues[2]) {  //if isError == true
    clearErrorsBtn.setColorBackground(color(1, 45, 90));  //set colors of button
    clearErrorsBtn.setColorForeground(controlP5.ControlP5Constants.BLUE);
    clearErrorsBtn.setColorActive(controlP5.ControlP5Constants.AQUA);
    clearErrorsBtn.setColorLabel(controlP5.ControlP5Constants.WHITE);
    clearErrorsBtn.setValue(0);  //unpressed state
  } else if(oldErrorVal && !serialBoolValues[2]) {  //if there was an error but not anymore
    pseudoConsole.append("\n" + "Errors cleared");
    clearErrorsBtn.setColorBackground(color(80, 80, 80))  //change button appearance to discourage clicking
                  .setColorForeground(color(80, 80, 80))
                  .setColorActive(color(80, 80, 80))
                  .setColorLabel(color(30, 30, 30))
                  .setValue(1);
  }  //end isError if else
  
  if(modeToggle.getBooleanValue() == true) {  //if mode is auto
    advanceBtn.setVisible(false);  //hide advance button
  } else {
    advanceBtn.setVisible(true);  //show button to manually advance position
    
    if(readyToAdvance) {  //if joints have reached target position and are ready to move to next position
      advanceBtn.setColorBackground(color(1, 45, 90));  //set colors of button
      advanceBtn.setColorForeground(controlP5.ControlP5Constants.BLUE);
      advanceBtn.setColorActive(controlP5.ControlP5Constants.AQUA);
      advanceBtn.setColorLabel(controlP5.ControlP5Constants.WHITE);
    } else {  //set different colors to discourage clicking
      advanceBtn.setColorBackground(color(80, 80, 80));
      advanceBtn.setColorForeground(color(80, 80, 80));
      advanceBtn.setColorActive(color(80, 80, 80));
      advanceBtn.setColorLabel(color(30, 30, 30));
    }
  }
  
  noStroke();  //no outline
  fill(200);  //set color value, light gray
  textSize(20);
  textAlign(RIGHT);
  text("Auto", modeToggle.getPosition()[0] - 10, modeToggle.getPosition()[1] + 18);  //draw text
  textAlign(LEFT);
  text("Manual", modeToggle.getPosition()[0] + modeToggle.getWidth() + 10, 
                  modeToggle.getPosition()[1] + 18);
  rect(width/2 - 150, 120, 300, 300);  //draw rectangle. background for leg animation
  image(currentSymbol, 390, 450, 30, 30);  //current symbol for left leg
  image(currentSymbol, 520, 450, 30, 30);  //current symbol for right leg
  image(speedometer, 455, 450, 30, 30);  //joint speed symbol for left leg
  image(speedometer, 585, 450, 30, 30);  //joint speed symbol for right leg
  
  drawGraphs();  //draw graph axes, labels and curves
  updateSpeedTorque();  //update speed and current values
  updatePosition();  //update joint position on graphs
  drawLegs();  //draw leg segments for animation
  updateWalkSpeedBodyWeight();  //update walk speed and weight bearing values
}

void reconnect() {
  handShake = false;  //reset flag
  teensyPort.clear();  //empty port buffer
  teensyPort.stop();  //close port connection
  firstContact = false;  //reset flag;
  teensyPort = new Serial(this, Serial.list()[1], 115200);  //select serial port from list and set baudrate
  teensyPortTimer = millis();  //time when port opened
  delay(1000);  //wait 1sec
  teensyPort.bufferUntil('\n');
}

/*
void modeToggle(boolean theFlag) {
  teensyPort.write('z');
  if(theFlag == true) {
    teensyPort.write("a");
    println("Mode set to auto");
  } else {
    teensyPort.write("m");
    println("Mode set to manual");
  }
}*/

void drawLegs() {  //draw legs for animation
  fill(200);  //color for shapes
  strokeWeight(4);  //outline thickness
  stroke(80);  //line and border color
  pushMatrix();  //start left/back leg
  segment(width/2, hipY, 0.9 * legSegLength, 90.0 - leftHipAngle);  //upper leg
  segment(0.9 * legSegLength, 0, 0.9 * legSegLength, leftKneeAngle);  //lower leg
  strokeWeight(2);  //thinner outline for knee joint
  circle(0, 0, 8);
  strokeWeight(4);  //thicker outline for foot
  segment(0.9 * legSegLength, 0, 0.9 * footSegLength, leftAnkleAngle - 90.0);  //foot
  strokeWeight(2);  //thinner outline for ankle joint
  circle(0, 0, 6);
  strokeWeight(4);  //reset to thicker outline
  popMatrix();  //reset coordinate system
  stroke(126);  //darker line and border color for right/front leg
  pushMatrix();  //start right/front leg
  segment(width/2, hipY, legSegLength, 90.0 - rightHipAngle);  //upper leg
  segment(legSegLength, 0, legSegLength, rightKneeAngle);  //lower leg
  strokeWeight(2);
  circle(0, 0, 8);
  strokeWeight(4);
  segment(legSegLength, 0, footSegLength, rightAnkleAngle - 90.0);
  strokeWeight(2);
  circle(0, 0, 6);
  strokeWeight(4);  //reset to default outline
  popMatrix();  //reset coordinate system
}

void segment(float x, float y, float segLength, float angle) {  //helper function to draw leg segments
  translate(x, y);  //move to start of leg segment
  rotate(radians(angle));  //rotate segment
  line(0, 0, segLength, 0);  //draw segment
}

void updateSpeedTorque() {  //update speed and torque values
  speed[0] = leftAnkleSpeed;
  speed[1] = leftKneeSpeed;
  speed[2] = leftHipSpeed;
  speed[3] = rightAnkleSpeed;
  speed[4] = rightKneeSpeed;
  speed[5] = rightHipSpeed;
  
  torque[0] = leftAnkleTorque;
  torque[1] = leftKneeTorque;
  torque[2] = leftHipTorque;
  torque[3] = rightAnkleTorque;
  torque[4] = rightKneeTorque;
  torque[5] = rightHipTorque;
  
  textAlign(CENTER);
  textSize(16);
  text("H:", 365, 510);
  text("K:", 365, 530);
  text("A:", 365, 550);
  for(int i = 0;i < 3;i++) {  //loop through array of current values
    text(torque[i], 405, 20 * i + 510);  //update gui values left leg
  }
  for(int i = 3;i < torque.length;i++) {  //loop through array of current values
    text(torque[i], 535, 20 * i + 450);  //update gui values right leg
  }
  for(int i = 0;i < 3;i++) {  //loop through array of speed values
    text(speed[i], 470, 20 * i + 510);  //update gui left leg
  }
  for(int i = 3;i < speed.length;i++) {  //loop through array of speed values
    text(speed[i], 600, 20 * i + 450);  //update gui right leg
  }
}

void updateWalkSpeedBodyWeight() {  //update values for walking speed and weight bearing
  float slowPos[] = slowerBtn.getPosition();  //get x,y coordinates of slowerBtn
  float speedTextX = (slowPos[0] + slowerBtn.getWidth()) * 1.39;  //center for text x alignment
  float weightPos[] = decreaseWeightBtn.getPosition();  //get x,y coordinates of btn
  float weightTextX = (weightPos[0] + decreaseWeightBtn.getWidth()) * 1.075;  //center for text x alignment
  
  fill(255);  //white
  textAlign(CENTER);
  
  //walk speed
  pushMatrix();  //save coordinafe system
  translate(speedTextX, slowPos[1]);  //move to center of text placement
  textSize(50);  //big text for value
  text(nf(walkingSpeed, 1, 1), 0, 50);  //walking speed
  textSize(20);  //smaller text for label
  text("Walking Speed (m/s)", 0, slowerBtn.getHeight() * 1.5);  //text for walking speed label
  popMatrix();  //reset coordinate system
  //end walk speed
  
  //body weight
  pushMatrix();
  translate(weightTextX, weightPos[1]);  //move to center of text placement
  textSize(50);
  text(bodyWeightPercent, 0, 50);  //% body weight
  textSize(20);
  text("Body Weight (%)", 0, decreaseWeightBtn.getHeight() * 1.5);  //label
  popMatrix();
  //end body weight
}

void updatePosition() {  //animate joint position on graphs
  float leftGraphXShift = leftGraphsXPos + (float)graphWidth/100.0 * leftLegGaitProgress;  //increment for circle x pos
  float righttGraphXShift = rightGraphsXPos + (float)graphWidth/100.0 * rightLegGaitProgress;  //for right leg graphs

  stroke(255, 255, 255);  //white outline
  strokeWeight(1);  //thin outline. default is 4
  fill(5, 175, 242);  //light blue
  
  //left hip
  if(leftHipAngle < -13 || leftHipAngle > 23) {  //if out of bounds, dot == red
    fill(color(255, 0, 0));  //red
    if(leftHipAngle < -13) {
      leftHipAngle = -13;
    } else {
      leftHipAngle = 23;
    }
  }
  circle(leftGraphXShift, height * 0.15 + 80 + -2 * leftHipAngle, 10);  //height of hip graph + pos of x-axis * dataScale
  //end left hip
  //left knee
  if(leftKneeAngle < -1 || leftKneeAngle > 66) {  //if out of bounds, dot == red
    fill(color(255, 0, 0));
    if(leftKneeAngle < -1) {
      leftKneeAngle = -1;
    } else {
      leftKneeAngle = 66;
    }
  }
  circle(leftGraphXShift, height * 0.33 + 128 + -2 * leftKneeAngle, 10);
  //end left knee
  //left ankle
  if(leftAnkleAngle < -12 || leftAnkleAngle > 21) {
    fill(color(255, 0, 0));
    if(leftAnkleAngle < -12) {
      leftAnkleAngle = -12;
    } else {
      leftAnkleAngle = 21;
    }
  }
  circle(leftGraphXShift, height * 0.56 + 30 + -2 * leftAnkleAngle, 10);
  //end left ankle
  ////////// right leg /////////
  //right hip
  if(rightHipAngle < -13 || rightHipAngle > 23) {
    fill(color(255, 0, 0));
    if(rightHipAngle < -13) {
      rightHipAngle = -13;
    } else {
      rightHipAngle = 23;
    }
  }
  circle(righttGraphXShift, height * 0.15 + 80 + -2 * rightHipAngle, 10);
  //end right hip
  //right knee
  if(rightKneeAngle < -1 || rightKneeAngle > 66) {
    fill(color(255, 0, 0));
    if(rightKneeAngle < -1) {
      rightKneeAngle = -1;
    } else {
      rightKneeAngle = 66;
    }
  }
  circle(righttGraphXShift, height * 0.33 + 128 + -2 * rightKneeAngle, 10);
  //end right knee
  //right ankle
  if(rightAnkleAngle < -21 || rightAnkleAngle > 12) {
    fill(color(255, 0, 0));
    if(rightAnkleAngle < -21) {
      rightAnkleAngle = -21;
    } else {
      rightAnkleAngle = 12;
    }
  }
  circle(righttGraphXShift, height * 0.56 + 30 + -2 * rightAnkleAngle, 10);
  //end right ankle
}
