import processing.serial.*;
Serial myPort;        // The serial port
int xPos = 1;         // horizontal position of the graph


int graph = 1;
void setup () {
 // set the window size:
 size(1000, 600);        
 // List all the available serial ports
 println(Serial.list());
 // Open whatever port is the one you're using.
 myPort = new Serial(this, Serial.list()[0], 9600);
 // don't generate a serialEvent() unless you get a newline character:
 myPort.bufferUntil('\n');
 // set inital background:
 background(0);
}
void draw () {
}
void mouseClicked() {
  //myPort.write('s');
}
void keyPressed() {
  myPort.write(key);
}
void serialEvent (Serial myPort) {
 // get the ASCII string:
  String inString = myPort.readStringUntil('\n');
  println(inString+inString.length());
  if (inString != null && inString.length() > 60 ) {
// 0         1         2         3         4         5         6
// 0123456789012345678901234567890123456789012345678901234567890123456789   
// X  -3 Y   2 Z   0 x -23 y  16 z   2 R2657 r2960 L2623 l3420
    float xRoll = float(trim(inString.substring(3,7)));
    float yRoll = float(trim(inString.substring(9,13)));
    float zRoll = float(trim(inString.substring(15,19)));
    println("x-"+xRoll+" y-"+yRoll+" z-"+zRoll);
    
    float xAccel = float(trim(inString.substring(21,25)));
    float yAccel = float(trim(inString.substring(27,31)));
    float zAccel = float(trim(inString.substring(33,37)));
    println("x-"+xAccel+" y-"+yAccel+" z-"+zAccel);
    
    float rMotor = float(trim(inString.substring(39,43)));
    float lMotor = float(trim(inString.substring(45,49)));
    float rServo = float(trim(inString.substring(51,55)));
    float lServo = float(trim(inString.substring(57,61)));
    println("start graphing");
    if(graph == 0){
      println("graphing Roll");
      graph(xRoll, -20, 20, 0, height/3, xPos);
      graph(yRoll, -20, 20, height/3, 2*height/3, xPos);
      graph(zRoll, -20, 20, 2*height/3, height, xPos);
    }else if(graph == 1){
      
      println("graphing Accel");
      graph(xAccel, -100, 100, 0, height/3, xPos);
      graph(yAccel, -100, 100, height/3, 2*height/3, xPos);
      graph(zAccel, -100, 100, 2*height/3, height, xPos);
    }else if(graph == 2){
      println("graphing Motors");
      graph(rMotor, -5000, 5000, 0, height/4, xPos);
      graph(lMotor, -5000, 5000, height/4, height/2, xPos);
      graph(rServo, -5000, 5000, height/2, 3*height/4, xPos);
      graph(lServo, -5000, 5000, 3*height/4, height, xPos);
    }
    if (xPos >= width) {
      xPos = 0;
       background(0); 
    } else {
 // increment the horizontal position:
      xPos++;
    }
  }
}
void graph(float inByte, int bInRange, int tInRange, int bGrRange, int tGrRange, int xPos){
  inByte = map(inByte, bInRange, tInRange, bGrRange, tGrRange);
  stroke(127,34,255);
  println(   ((tGrRange+bGrRange)/2)+" "+ inByte);
  line(xPos,  (tGrRange+bGrRange)/2, xPos, inByte);
}
//0        1       2
//012 5678 1234 7890 3456   
// X%4d Y%4d Z%4d x%4d y%4d z%4d R%4d r%4d L%4d l%4d 
 //capital XYZ = Roll lower case xyz = acceleration RL = motors rl = servos
// 0         1         2         3         4         5         6
// 0123456789012345678901234567890123456789012345678901234567890123456789   
// X  -3 Y   2 Z   0 x -23 y  16 z   2 R2657 r2960 L2623 l3420
