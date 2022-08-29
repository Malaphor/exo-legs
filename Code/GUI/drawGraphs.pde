void drawGraphs() {  //draw graph axes, labels and curves
  PFont arialBold = createFont("Arial Bold", 14);  //select font and size
  int yAxisLabelXPos = -5;  //y-axis label offset from graph axis line
  
  //properties for drawing
  noFill();
  stroke(241, 241, 241);
  strokeWeight(2);
  strokeCap(ROUND);
  
  //left hip graph
  pushMatrix();  //save current coordinate system
  translate(leftGraphsXPos, height * 0.15);  //move to draw hip graph
  textFont(arialBold);
  textSize(20);
  text("Hip", 100, 30);
  //properties for graph y-axis labels
  textAlign(RIGHT);
  textSize(14);
  text("40", yAxisLabelXPos, 7);  //desired position + 1/2 textSize
  text("20", yAxisLabelXPos, 47);
  text("0", yAxisLabelXPos, 87);
  text("-20", yAxisLabelXPos, 127);
  line(0, 0, 0, 120);  //y axis
  line(0, 80, graphWidth, 80);  //x axis
  for(int i = 0;i < 4;i++) {  //draw tick marks on y-axis
    line(0, i * 40, 5, i * 40);
  }
  beginShape();  //start drawing graph curve
  curveVertex(0, hipAngles[0] * -2 + 80);  //start point of graph curve
  for(int i = 0;i < hipAngles.length;i++) {  //draw the rest
    curveVertex(i * 5, hipAngles[i] * -2 + 80);  //add x-axis pos to y values
  }
  endShape();  //stop drawing graph curve
  popMatrix();  //restore coordinate system
  //end left hip graph
  
  //left knee graph
  pushMatrix();
  translate(leftGraphsXPos, height * 0.33);
  textFont(arialBold);
  textSize(20);
  text("Knee", 110, 40);
  textAlign(RIGHT);
  textSize(14);
  text("80", yAxisLabelXPos, 7);
  text("60", yAxisLabelXPos, 39);
  text("40", yAxisLabelXPos, 71);
  text("20", yAxisLabelXPos, 103);
  text("0", yAxisLabelXPos, 135);
  text("-20", yAxisLabelXPos, 167);
  line(0, 0, 0, 160);  //y axis
  line(0, 128, graphWidth, 128);  //x axis
  for(int i = 0;i < 6;i++) {  //tick marks
    line(0, i * 32, 5, i * 32);
  }
  beginShape();
  curveVertex(0, kneeAngles[0] * -2 + 120);
  for(int i = 0;i < kneeAngles.length;i++) {
    curveVertex(i * 5, kneeAngles[i] * -2 + 128);
  }
  endShape();
  popMatrix();
  //end left knee graph
  
  //left ankle graph
  pushMatrix();
  translate(leftGraphsXPos, height * 0.56);  //position upper left corner of graph
  textFont(arialBold);
  textSize(20);
  text("Ankle", 110, -10);
  textAlign(RIGHT);
  textSize(14);
  text("20", yAxisLabelXPos, 7);
  text("0", yAxisLabelXPos, 37);
  text("-20", yAxisLabelXPos, 67);
  line(0, 0, 0, 60);  //y axis
  line(0, 30, graphWidth, 30);  //x axis, y values are at height y == 0
  for(int i = 0;i < 3;i++) {  //tick marks
    line(0, i * 30, 5, i * 30);
  }
  beginShape();
  curveVertex(0, ankleAngles[0] * -2 + 30);
  for(int i = 0;i < ankleAngles.length;i++) {  //loop through data, create curve
    curveVertex(i * 5, ankleAngles[i] * -2 + 30);
  }
  endShape();
  popMatrix();
  //end left ankle graph
  
  //left graphs bottom axis (0 - 100)
  pushMatrix();
  translate(width * 0.03, height * 0.67);
  line(0, 0, 270, 0);  //bottom axis
  for(int i = 0;i <11;i++) {  //tick marks
    line(i * 24.5 + 20, 0, i * 24.5 + 20, -5);
  }
  rotate(PI * -0.5);
  text("0", -10, 25);
  text("20", -10, 74);
  text("40", -10, 123);
  text("60", -10, 172);
  text("80", -10, 221);
  text("100", -10, 270);
  popMatrix();
  //end left graphs bottom axis
  //////////////////////////////////////////////////
  //right hip graph
  pushMatrix();
  translate(rightGraphsXPos, height * 0.15);
  textFont(arialBold);
  textSize(20);
  text("Hip", 100, 30);
  textAlign(RIGHT);
  textSize(14);
  text("40", yAxisLabelXPos, 7);  //desired position + 1/2 textSize
  text("20", yAxisLabelXPos, 47);
  text("0", yAxisLabelXPos, 87);
  text("-20", yAxisLabelXPos, 127);
  line(0, 0, 0, 120);  //y axis
  line(0, 80, graphWidth, 80);  //x axis
  for(int i = 0;i < 4;i++) {  //tick marks
    line(0, i * 40, 5, i * 40);
  }
  beginShape();
  curveVertex(0, hipAngles[0] * -2 + 80);
  for(int i = 0;i < hipAngles.length;i++) {
    curveVertex(i * 5, hipAngles[i] * -2 + 80);  //add x-axis pos to y values
  }
  endShape();
  popMatrix();
  //end right hip graph
  
  //right knee graph
  pushMatrix();
  translate(rightGraphsXPos, height * 0.33);
  textFont(arialBold);
  textSize(20);
  text("Knee", 110, 40);
  textAlign(RIGHT);
  textSize(14);
  text("80", yAxisLabelXPos, 7);
  text("60", yAxisLabelXPos, 39);
  text("40", yAxisLabelXPos, 71);
  text("20", yAxisLabelXPos, 103);
  text("0", yAxisLabelXPos, 135);
  text("-20", yAxisLabelXPos, 167);
  line(0, 0, 0, 160);  //y axis
  line(0, 128, graphWidth, 128);  //x axis
  for(int i = 0;i < 6;i++) {  //tick marks
    line(0, i * 32, 5, i * 32);
  }
  beginShape();
  curveVertex(0, kneeAngles[0] * -2 + 120);
  for(int i = 0;i < kneeAngles.length;i++) {
    curveVertex(i * 5, kneeAngles[i] * -2 + 128);
  }
  endShape();
  popMatrix();
  //end right knee graph
  
  //right ankle graph
  pushMatrix();
  translate(rightGraphsXPos, height * 0.56);  //position upper left corner of graph
  textFont(arialBold);
  textSize(20);
  text("Ankle", 110, -10);
  textAlign(RIGHT);
  textSize(14);
  text("20", yAxisLabelXPos, 7);
  text("0", yAxisLabelXPos, 37);
  text("-20", yAxisLabelXPos, 67);
  line(0, 0, 0, 60);  //y axis
  line(0, 30, graphWidth, 30);  //x axis, y values are at height y == 0
  for(int i = 0;i < 3;i++) {  //tick marks
    line(0, i * 30, 5, i * 30);
  }
  beginShape();
  curveVertex(0, ankleAngles[0] * -2 + 30);
  for(int i = 0;i < ankleAngles.length;i++) {  //loop through data, create curve
    curveVertex(i * 5, ankleAngles[i] * -2 + 30);
  }
  endShape();
  popMatrix();
  //end right ankle graph
  
  //right graphs bottom axis (0 - 100)
  pushMatrix();
  translate(width * 0.70, height * 0.67);
  line(0, 0, 270, 0);  //bottom axis
  for(int i = 0;i <11;i++) {  //tick marks
    line(i * 24.5 + 20, 0, i * 24.5 + 20, -5);
  }
  rotate(PI * -0.5);
  text("0", -10, 25);
  text("20", -10, 74);
  text("40", -10, 123);
  text("60", -10, 172);
  text("80", -10, 221);
  text("100", -10, 270);
  popMatrix();
  //end right graphs bottom axis
}
