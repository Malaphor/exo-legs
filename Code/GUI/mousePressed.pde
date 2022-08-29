void mousePressed() {  //capture mouse click event
  if(startStopBtn.isPressed()) {  //if mouse click is on startStop button
    if(startStopBtn.getValue() == 0) {  //if stopped, start
      teensyPort.write("b");  //send begin motion command to teensy
      println("begin");  //print to console
      startStopBtn.setValue(1);
      startStopBtn.setCaptionLabel("Press to stop");
      moveStatus.setColorValue(color(0, 200, 0));  //green
      moveStatus.setText("Moving...");
    } else {  //if moving
      teensyPort.write("s");  //send stop motion command to teensy
      println("stop");  //print to console
      startStopBtn.setValue(0);
      startStopBtn.setCaptionLabel("Press to start");
      moveStatus.setColorValue(color(200, 0, 0));  //red
      moveStatus.setText("Motion stopped");
      if(homeBtn.getValue() == 1 && !serialBoolValues[0]) {  //if homeBtn is in its pressed state && not homed
        homeBtn.setValue(0);
        homeBtn.setColorBackground(color(1, 45, 90));  //set colors of button
        homeBtn.setColorForeground(controlP5.ControlP5Constants.BLUE);
        homeBtn.setColorActive(controlP5.ControlP5Constants.AQUA);
        homeBtn.setColorLabel(controlP5.ControlP5Constants.WHITE);
        homeBtn.setCaptionLabel("Press to Home");
      }
    }
  }  //end if startStop pressed
  
  if(homeBtn.isPressed()) {  //if mouse click is on home button
    if(homeBtn.getValue() == 0) {  //if not homed
      teensyPort.write("h");  //send home sequence command to teensy
      println("home");
      homeBtn.setValue(1);  //dont think this should be set outside serialEvent
      homeBtn.setColorBackground(color(80, 80, 80));  //change button appearance to discourage clicking
      homeBtn.setColorForeground(color(80, 80, 80));
      homeBtn.setColorActive(color(80, 80, 80));
      homeBtn.setColorLabel(color(30, 30, 30));
      homeBtn.setCaptionLabel("Homing");  //change button label
      startStopBtn.setValue(1);  //joints are moving so change startStop val
      startStopBtn.setCaptionLabel("Press to stop");  //and change label
      moveStatus.setColorValue(color(0, 200, 0));
      moveStatus.setText("Moving...");
    } else {  //if homeBtn.getValue() == 1
      //do nothing. button unclickable
    }
  }
  
  if(advanceBtn.isPressed() && readyToAdvance) {  //manual control only. if joints at target ready to move
    readyToAdvance = false;
    teensyPort.write("y");  //send move to next position command to teensy
    println("Advance");
  }  //end if advanceBtn
  
  if(clearErrorsBtn.isPressed()) {
    if(clearErrorsBtn.getValue() == 0) {  //if it hasnt already been pressed
      teensyPort.write("k");  //send clearErrors() cmd
      println("attempting to clear errors");  //print to console
      pseudoConsole.append("\n" + "Attempting to clear errors");  //alert user
      clearErrorsBtn.setColorBackground(color(80, 80, 80));  //change button appearance to discourage clicking
      clearErrorsBtn.setColorForeground(color(80, 80, 80));
      clearErrorsBtn.setColorActive(color(80, 80, 80));
      clearErrorsBtn.setColorLabel(color(30, 30, 30));
      clearErrorsBtn.setValue(1);
    }
  }  //end if clearErrorsBtn
  
  if(slowerBtn.isPressed()) {
    if(walkingSpeed > walkSpeedLimitLow) {
      teensyPort.write("q");
    } else {
      pseudoConsole.append("\n" + "Cannot go any slower");
    }
  }  //end if slowerBtn pressed
  
  if(fasterBtn.isPressed()) {
    if(walkingSpeed < walkSpeedLimitHigh) {
      teensyPort.write("f");
    } else {
      pseudoConsole.append("\n" + "Cannot go any faster");
    }
  }  //end if fasterBtn pressed
  
  if(decreaseWeightBtn.isPressed()) {
    if(bodyWeightPercent > bodyWeightLimitLow) {
      teensyPort.write("k");
    } else {
      pseudoConsole.append("\n" + "Cannot decrease body weight support");
    }
  }  //end if decreaseWeightBtn pressed
  
  if(increaseWeightBtn.isPressed()) {
    if(bodyWeightPercent < bodyWeightLimitHigh) {
      teensyPort.write("a");
    } else {
      pseudoConsole.append("\n" + "Cannot increase body weight support");
    }
  }  //end if increaseWeightBtn pressed
}  //end mousePressed
