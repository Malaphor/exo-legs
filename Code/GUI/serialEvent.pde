void serialEvent(Serial teensyPort) {  //serial event called when data available in teensyPort
  try {
    fromTeensy = teensyPort.readStringUntil('\n');  //read data available from serial port from teensy
    if(fromTeensy != null) {  //if string read is valid, not empty
      fromTeensy = trim(fromTeensy);  //remove whitespace from front and back of string
      firstChar = fromTeensy.charAt(0);  //char at beginning of string
    
      if(handShake == false) {  //if handshake not completed
        if(fromTeensy.equals("A")) {  //look for handshake char
          teensyPort.clear();  //clear data in port
          handShake = true;  //flag handshake complete
          teensyPort.write("A");  //send handshake char back to teensy
          println("Contact");
          teensyPortTimer = millis();
          if(!firstContact) {  //if have not connected at least once
            firstContact = true;  //this is the first time serial comms established or by reconnect()
            reconnectAttempts = 0;  //reset counter
            pseudoConsole.append("\n" + "Serial connection successful");
            moveStatus.setColorValue(color(212, 85, 0));  //change color of status text, orange
            moveStatus.setText("Waiting for input");  //change status text to reflect completed handshake
          }
        }  //end if
      } else {  //if handshake complete, look for start char
        if(firstChar == '<') {  //if error start char seen
          if(!oldErrorVal && serialBoolValues[2]) {  //if previous error value is false and theres currently an error
            errorString = split(fromTeensy.substring(1, fromTeensy.length() - 1), ",");
            printArray(errorString);
            if(firstError) {  //if this is the first error that came in
              clearErrorsBtn.setValue(0);
              firstError = false;  //set flag
              pseudoConsole.setColor(color(200, 0, 0));  //red
              pseudoConsole.clear();  //empty textarea
            }  //end if firstError
            
            for(int i = 0;i < 4;i++) {  //loop through errorString array and print errors
              pseudoConsole.append(errorString[i]);
              //pseudoConsole.append("\n");
            }
            
            if(int(errorString[4]) == 6) {  //if error index is 6, all joints errors have been read
              oldErrorVal = true;  //set flag so dont continue to append repeat errors
              firstError = true;  //set flag for next time errors come
            }
          }
        } /*else if(firstChar == '$') {  //toggle mode stuff?
            int toggleVal = int(modeToggle.getValue());
            println(fromTeensy.substring(1, fromTeensy.length() - 1));
            println(toggleVal);
            if(toggleVal == 0) {
              teensyPort.write("m");
            } else {
              teensyPort.write("a");
            }
        }*/ else if(firstChar == '#') {  //if data start char seen
          println(fromTeensy.substring(1, fromTeensy.length() - 1));
          serialValues = split(fromTeensy.substring(1, fromTeensy.length() - 2), ",");  //split data into string array at each ,
          //printArray(serialValues);
          for(int i = 0;i < serialBoolValues.length;i++) {
            serialBoolValues[i] = boolean(serialValues[i]);  //cast String values into bool values
          }
          leftAnkleAngle = float(serialValues[4]);
          leftKneeAngle = float(serialValues[5]);
          leftHipAngle = float(serialValues[6]);
          rightAnkleAngle = float(serialValues[7]);  // / convertPotToAngle;  //CHECK MATH
          rightKneeAngle = float(serialValues[8]);  // / convertPotToAngle;  //CHECK MATH
          rightHipAngle = float(serialValues[9]);  // / convertPotToAngle;  //CHECK MATH
          leftAnkleTorque = float(serialValues[10]) * 8.3 / motorKv;  //convert measured current to torque
          leftKneeTorque = float(serialValues[11]) * 8.3 / motorKv;  //convert measured current to torque
          leftHipTorque = float(serialValues[12]) * 8.3 / motorKv;  //convert measured current to torque
          rightAnkleTorque = float(serialValues[13]);
          rightKneeTorque = float(serialValues[14]);
          rightHipTorque = float(serialValues[15]);
          leftAnkleSpeed = float(serialValues[16]);
          leftAnkleSpeed = float(serialValues[17]);
          leftAnkleSpeed = float(serialValues[18]);
          rightAnkleSpeed = float(serialValues[19]);
          rightKneeSpeed = float(serialValues[20]);
          rightHipSpeed = float(serialValues[21]);
        } else if(firstChar == '!') {  //if walkSpeed or bodyWeight start char seen
          println(fromTeensy.substring(1, fromTeensy.length() - 1));
          speedWeightValues = split(fromTeensy.substring(1, fromTeensy.length() - 2), ",");  //split data into string array at each ,
          walkingSpeed = float(speedWeightValues[0]);
          bodyWeightPercent = int(speedWeightValues[1]);
          
          if(walkingSpeed <= walkSpeedLimitLow) {
            slowerBtn.setColorBackground(color(80, 80, 80));  //change button appearance to discourage clicking
            slowerBtn.setColorForeground(color(80, 80, 80));
            slowerBtn.setColorActive(color(80, 80, 80));
            slowerBtn.setColorLabel(color(30, 30, 30));
          } else {
            slowerBtn.setColorBackground(color(1, 45, 90));  //set colors of button
            slowerBtn.setColorForeground(controlP5.ControlP5Constants.BLUE);
            slowerBtn.setColorActive(controlP5.ControlP5Constants.AQUA);
            slowerBtn.setColorLabel(controlP5.ControlP5Constants.WHITE);
          }  //end walkingSpeed < limit if else
          
          if(walkingSpeed >= walkSpeedLimitHigh) {
            fasterBtn.setColorBackground(color(80, 80, 80));  //change button appearance to discourage clicking
            fasterBtn.setColorForeground(color(80, 80, 80));
            fasterBtn.setColorActive(color(80, 80, 80));
            fasterBtn.setColorLabel(color(30, 30, 30));
          } else {
            fasterBtn.setColorBackground(color(1, 45, 90));  //set colors of button
            fasterBtn.setColorForeground(controlP5.ControlP5Constants.BLUE);
            fasterBtn.setColorActive(controlP5.ControlP5Constants.AQUA);
            fasterBtn.setColorLabel(controlP5.ControlP5Constants.WHITE);
          }  //end walkingSpeed > limit if else
          
          if(bodyWeightPercent <= bodyWeightLimitLow) {
            decreaseWeightBtn.setColorBackground(color(80, 80, 80));  //change button appearance to discourage clicking
            decreaseWeightBtn.setColorForeground(color(80, 80, 80));
            decreaseWeightBtn.setColorActive(color(80, 80, 80));
            decreaseWeightBtn.setColorLabel(color(30, 30, 30));
          } else {
            decreaseWeightBtn.setColorBackground(color(1, 45, 90));  //set colors of button
            decreaseWeightBtn.setColorForeground(controlP5.ControlP5Constants.BLUE);
            decreaseWeightBtn.setColorActive(controlP5.ControlP5Constants.AQUA);
            decreaseWeightBtn.setColorLabel(controlP5.ControlP5Constants.WHITE);
          }  //end bodyWeightPercent < limit if else
          
          if(bodyWeightPercent >= bodyWeightLimitHigh) {
            increaseWeightBtn.setColorBackground(color(80, 80, 80));  //change button appearance to discourage clicking
            increaseWeightBtn.setColorForeground(color(80, 80, 80));
            increaseWeightBtn.setColorActive(color(80, 80, 80));
            increaseWeightBtn.setColorLabel(color(30, 30, 30));
          } else {
            increaseWeightBtn.setColorBackground(color(1, 45, 90));  //set colors of button
            increaseWeightBtn.setColorForeground(controlP5.ControlP5Constants.BLUE);
            increaseWeightBtn.setColorActive(controlP5.ControlP5Constants.AQUA);
            increaseWeightBtn.setColorLabel(controlP5.ControlP5Constants.WHITE);
          }  //end bodyWeightPercent > limit if else
        }  //end first char if else
      }  //end handShake
    }  //end if(!null)
    fromTeensy = "";
  } catch(RuntimeException e) {
    e.printStackTrace();
  }  //end try catch
}  //end serialEvent
