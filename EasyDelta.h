const float XMAX = 2000;
const float YMAX = 2000;
const float ZMAX = 2000; // maximum dimensions (measured in mm)

const float d = YMAX * tan( 15 * ( PI / 180 )); // standard offset of motors

class Motor {

  public:

    int stepPin;
    int dirPin;

    int radius;

    float x_coord;
    float y_coord;

    float oldDistance;
    float newDistance;
    float difference;

    int stepsToTake;
    int rotations;

    Motor(int dir, int stp, float x, float y) {

      dirPin = dir; // direction
      stepPin = stp; // steps

      radius = 10; // radius of spool (mm)
      
      x_coord = x;
      y_coord = y; // coords of motor
      
      oldDistance = distance(oldDistInit[0],oldDistInit[1],oldDistInit[2]);
    }

    void Update(int x, int y, int z) {
      
      newDistance = distance(x,y,z);
      difference = oldDistance - newDistance;

      stepsToTake = difference/(2 * PI * stepsFull * radius); // how many steps to reach targetCoord
//      rotations = 1000* abs(difference / radius);
      rotations = 3000;
//      Serial.println("steps to take");
//      Serial.println(rotations);
//      Serial.println();

      oldDistance = newDistance;
      Serial.println(oldDistance);
      
      return;
    }

//    void iterate() {
//      
//      digitalWrite(stepPin, HIGH);
//      delay(1);        
//      digitalWrite(stepPin, LOW); 
//      delay(1);
//      
//      stepCounter = stepCounter + 1;
//
//      return;
//    }

  private:

    const float oldDistInit[3] = {(XMAX / 2), (YMAX / 2), 0};
    const int stepsFull = 3600;

    int distance(int x, int y, int z) {
      
      int z_comp = sq( ZMAX - z );
      return sqrt( sq( x_coord - x ) + sq( y_coord - y ) + z_comp );
    }
};
