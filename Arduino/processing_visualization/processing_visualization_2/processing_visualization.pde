/*
    Arduino and MPU6050 IMU - 3D Visualization Example 
     by Dejan, https://howtomechatronics.com
*/
import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;
Serial myPort;
String data="";
float roll, pitch,yaw;

void setup() {
  size (1600, 800, P3D);
  //myPort = new Serial(this, "COM12", 19200); // starts the serial communication
  //myPort.bufferUntil('\n');
}

void draw() {
  translate(width/2, height/2, 0);
  background(150,230,230);
  textSize(22);
  fill(0, 0, 200); // Blue 
  text("Roll: " + int(roll) + "     Pitch: " + int(pitch) + "     Yaw: " + int(yaw), -100, 265);
  // Rotate the object
  rotateX(radians(-pitch));
  rotateZ(radians(roll));
  rotateY(radians(yaw));
  
  // 3D 0bject
  drawRocket();
  
  //delay(10);
  //println("ypr:\t" + angleX + "\t" + angleY); // Print the values to check whether we are getting proper values
}

void keyPressed() {
  if (key == 'w') pitch -= 5;
  if (key == 's') pitch += 5;
  if (key == 'a') roll -= 5;
  if (key == 'd') roll += 5;
  if (key == 'q') yaw -= 5;
  if (key == 'e') yaw += 5;
}


// Read data from the Serial Port
void serialEvent (Serial myPort) { 
  // reads the data from the Serial Port up to the character '.' and puts it into the String variable "data".
  data = myPort.readStringUntil('\n');
  // if you got any bytes other than the linefeed:
  if (data != null) {
    data = trim(data);
    // split the string at "/"
    String items[] = split(data, '/');
    if (items.length > 1) {
      //--- Roll,Pitch in degrees
      roll = float(items[0]);
      pitch = float(items[1]);
      yaw = float(items[2]);
    }
  }
}


// DRAW ROCKET --------------------------------------------------------------------------------------------
void drawRocket() {
  pushMatrix();
  
  // Corpo del razzo
  fill(200, 0, 0);
  cylinder(40, 150);
  
  // Punta del razzo
  translate(0, -105, 0);  // 105 = cylinder_height/2 + cone_height/2 = 150/2 + 60/2 = 75 + 30 = 105
  fill(255, 255, 255);
  cone(40, 60);
  
  // Torniamo alla base per disegnare le alette
  translate(0, 75, 0);
  fill(0, 0, 200);
  drawFins();
  
  popMatrix();
}

void cylinder(float r, float h) {
  int sides = 30;
  float angleStep = TWO_PI / sides;
  beginShape(QUAD_STRIP);
  for (int i = 0; i <= sides; i++) {
    float angle = i * angleStep;
    float x = cos(angle) * r;
    float z = sin(angle) * r;
    vertex(x, -h / 2, z);
    vertex(x, h / 2, z);
  }
  endShape();
}

void cone(float r, float h) {
  int sides = 30;
  float angleStep = TWO_PI / sides;
  beginShape(TRIANGLE_FAN);
  vertex(0, -h / 2, 0); // Punta del cono
  for (int i = 0; i <= sides; i++) {
    float angle = i * angleStep;
    float x = cos(angle) * r;
    float z = sin(angle) * r;
    vertex(x, h / 2, z);
  }
  endShape();
}

void drawFins() {
  // two fins green, two blue
  for (int i = 0; i < 4; i++) {
    pushMatrix();
    rotateY(HALF_PI * i);
    translate(40, 30, 0);
    
     // Alternating colors for fins
    if (i % 2 == 0) {
      fill(0, 0, 200); // Blue 
    } else {
      fill(0, 255, 0); // Green
    }
    
    beginShape();
    vertex(0, 0, 0);
    vertex(-20, 40, 0);
    vertex(20, 40, 0);
    endShape(CLOSE);
    popMatrix();
  }
}
//------------------------------------------------------------------------
