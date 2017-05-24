/*
ChickenRun 3DOF IK by ShanGlor
upload "StandardFirmata" sketch to Arduino
tested on Processing 2.0.1
tested on Processing 3.3 needs to install "Arduino" libs
Select from the serial port list, change [1] to your assigned COM port 
Servos on D9, D10 and D11 of Arduino
*/

import processing.serial.*;    
import cc.arduino.*;
Arduino arduino;
int xpos;
int ypos;
int A2, A3, D2, D3, F2;

PVector origin;
int femur_length, tibia_length, pastern_length, toe_length;
int hip_angle,knee_angle,knee_angle2,pastern_angle,toe_angle;
Segment femur,tibia,pastern,toe;
PVector drag_delta = new PVector(0,0);
int hock1_radius = 35;
boolean hock1_drag = false;

void setup()
{  
  size(500,500,P2D);
    
  origin = new PVector(width/2, height/4);  
  femur_length = 100;
  tibia_length = 120;
  pastern_length = 50;
  toe_length = 1;
  hip_angle = 90;
  knee_angle = 90;
  pastern_angle = 180; //0 avian gait or 180 for man gait, edit also "float A3a"
  toe_angle = 90;

  femur = new Segment(femur_length);
  femur.setOrigin(this.origin);
  femur.setRotation(hip_angle);

  tibia = new Segment(tibia_length);
  tibia.setOrigin(femur.P2);
  tibia.setRotation(knee_angle);

  pastern = new Segment(pastern_length);
  pastern.setOrigin(tibia.P2);
  pastern.setRotation(pastern_angle);
  
  toe = new Segment(toe_length);
  toe.setOrigin(pastern.P2);
  toe.setRotation(toe_angle);
      
  println(Arduino.list());
  arduino = new Arduino(this, Arduino.list()[1], 57600); //Select from the list your assigned COM port 
  
  arduino.pinMode(9, Arduino.SERVO);
  arduino.pinMode(10, Arduino.SERVO);
  arduino.pinMode(11, Arduino.SERVO);    
}

void draw()
{
    background(128); 
    stroke(#333333);
    strokeWeight(1);

    femur.draw();
    tibia.draw();
    pastern.draw();
    toe.draw();
    
    stroke(#333333);
    fill(#333333);
    text("Hip angle: "+ int(femur.rotation), 10, 20); //uncomment this line for man gait
    //text("Hip angle: "+ int(femur.rotation-90), 10, 20); //uncomment this line for quad/hexapod gait
    //text("Hip angle: "+ int(femur.rotation-45), 10, 20); //uncomment this line for avian gait
    text("Knee angle: "+ int(A2), 10, 40);
    text("Ankle angle: "+ int(A3), 10, 60);
    text("by ShanGlor", 10, 480);
    
    if(mouse_over_hock1()) draw_hock1();
      
}

boolean mouse_over_hock1()
{
  return ( dist(tibia.P2.x, tibia.P2.y, mouseX, mouseY) <= hock1_radius);    
}

void draw_hock1()
{
   noStroke();
   fill(#CC3300,100);
   ellipse(tibia.P2.x,tibia.P2.y,hock1_radius,hock1_radius);
}

void mousePressed() 
{
   if(!mouse_over_hock1()) return;
   hock1_drag = true;
   drag_delta.x = tibia.P2.x - mouseX;
   drag_delta.y = tibia.P2.y - mouseY;
}

void mouseDragged() 
{
   if(!hock1_drag) return;

   PVector target = new PVector( drag_delta.x + mouseX, drag_delta.y + mouseY);
   PVector rots =  IK(femur_length,tibia_length,target);

   femur.setRotation(rots.x);
   tibia.setOrigin(femur.P2);
   tibia.setRotation(rots.y);
   pastern.setOrigin(tibia.P2);
   pastern.setRotation(pastern_angle);
   toe.setOrigin(pastern.P2);
   toe.setRotation(toe_angle);    

}

void mouseReleased() 
{
    if(hock1_drag) hock1_drag = false;
}

/**********************************************************
 * Inverse Kinematic function for a two link planar system.
 * Given the size of the two links an a desired position, 
 * it returns the angles for both links
 **********************************************************/
 PVector IK(int a,int b,PVector d)
 {
     PVector rotations = new PVector(0,0);
     
     float dx = d.x - origin.x;
     float dy = d.y - origin.y;
     
     //calculates the distance beetween the first link and the endpoint
     float distance = sqrt(dx*dx+dy*dy);
     float c = min(distance, a + b);
       
     //calculates ankle angle relative to origin
     float D = atan2(dy,dx);
     float D1 = degrees(D);
     D2 = Math.round(D1);
            
     float D4 = 180-D2;
     D3 = Math.round(D4);
        
     //calculates the angle between the distance segment and the first link
     float B = acos((b * b - a * a - c * c)/(-2 * a * c));

     //calculates the angle between the first and second link
     float C = acos((c * c - a * a - b * b)/(-2 * a * b));
     //float C = acos((a * a + b * b - c * c)/(2 * a * b));
     
     //calculate knee angle relative to femur 
     float E = degrees(C);
     A2 = Math.round(E);
     
     //calculate ankle angle relative to tibia 
     float F = acos((tibia_length * tibia_length + c * c - femur_length * femur_length)/(2 * tibia_length * c));
     float F1 = degrees(F);
     F2 = Math.round(F1);
     
     float A3a = 180 -(F2+D3);  //for man run
     //float A3a = F2+D3;  //for chicken run
     A3 = Math.round(A3a);
   
     float hip_angle = degrees(D + B); 
     float knee_angle = degrees(D + B + PI + C);
     if(hip_angle > 360) hip_angle -= 360;
     if(knee_angle > 360) knee_angle -= 360;
    
     
     rotations.x = hip_angle;
     //xpos = Math.round(hip_angle);
     //println(xpos); //
      
     rotations.y = knee_angle;
     //ypos = Math.round(knee_angle);
    // println(ypos); //
    // println("=========================");        
    
     return rotations;
 }

class Segment
{
    int size;
    PVector P1,P2;
    float rotation;

    Segment(int s)
    {
        size = s;
        P1 = new PVector(0,0);
        P2 = new PVector(0,0);
    }
    
    void setOrigin(PVector orig)
    {
      P1.x = orig.x;
      P1.y = orig.y;
    }
    
    void setRotation(float rotation)
    {
      this.rotation = rotation;
      P2.x = P1.x + this.size * cos(radians(this.rotation));
      P2.y = P1.y + this.size * sin(radians(this.rotation));
    }

    void draw()
    {
      
      stroke(0);
      strokeWeight(2);
      line(P1.x,P1.y,P2.x,P2.y);
      
      stroke(255,0,0,100);
      fill(240,0,0,200);
      ellipse(P1.x,P1.y,4,4);
      
  //arduino.servoWrite(9, xpos);  //hip angle
  arduino.servoWrite(9, constrain(int(femur.rotation), 0, 180));
  arduino.servoWrite(10, constrain(A2, 0, 180)); //knee angle
  arduino.servoWrite(11, constrain(A3, 0, 180));  //ankle angle
  }
}
