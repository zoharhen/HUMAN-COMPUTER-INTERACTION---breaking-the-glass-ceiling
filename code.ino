#include<Wire.h>
#define NOTHING 0
#define KNOCK 1
#define SLIDE 2
#define MAX_KNOCK 3
#define RESET_MAX_KNOCK 4

const int MIN_GLASS = 2;
const int MAX_GLASS = 100;
const int VERY_LONG_DELAY = 100;
const int LONG_DELAY = 50;
const int SHORT_DELAY = 25;
int knockCounter;

const int filterSize = 31;
float hpf[filterSize] = {0.0561, -0.149, 0.1263, 0.0152, -0.069, -0.026, 0.0447, 0.0433, -0.023, -0.062, -0.010, 0.0766, 0.0716, -0.087, -0.306, 0.5904, -0.306, -0.087, 0.0716, 0.0766, -0.010, -0.062, -0.023, 0.0433, 0.0447, -0.026, -0.069, 0.0152, 0.1263, -0.149, 0.0561};
float lpf[filterSize] = {-0.025, -0.000, 0.0014, 0.0041, 0.0078, 0.0125, 0.0179, 0.0238, 0.0301, 0.0364, 0.0425, 0.0479, 0.0525, 0.0559, 0.0581, 0.0588, 0.0581, 0.0559, 0.0525, 0.0479, 0.0425, 0.0364, 0.0301, 0.0238, 0.0179, 0.0125, 0.0078, 0.0041, 0.0014, -0.000, -0.025};
float cyclicBuffer[filterSize];
float lpfConvolution = 0.0;
float hpfConvolution = 0.0;
float lpfValue = 0.0;
float hpfValue = 0.0;
float alpha = 0.5;
float lpfThreshold = 12000.0;
float hpfThreshold = 2000.0;
const int MPU6050_addr=0x68;
int16_t AccX;
int16_t AccY;
int16_t AccZ;
int curIndex = 0;
int lastGesture = NOTHING;

void setup() {
  for (int i = 0; i < filterSize; i++)
  {
    cyclicBuffer[i] = 1; 
  }
  initMpu();
  empty_loop(50);
}

/*
 * a method to init MPU
 */
void initMpu()
{
  Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
  knockCounter = 0;
}

void empty_loop(int num_of_times)
{
  for (int i = 0; i < num_of_times; i++)
  {
    readMPU();
    cyclicBuffer[curIndex] = (float)AccZ;
  
    // convolution:
    lpfConvolution = 0;
    hpfConvolution = 0;
    for (int j = 0; j < filterSize; j++ )
    {
      lpfConvolution += cyclicBuffer[(curIndex - j + filterSize) % filterSize] * lpf[j];
      hpfConvolution += cyclicBuffer[(curIndex - j + filterSize) % filterSize] * hpf[j];
    }
    // running average:
    lpfValue = alpha * abs(lpfConvolution) + (1.0 - alpha) * lpfValue;
    hpfValue = alpha * abs(hpfConvolution) + (1.0 - alpha) * hpfValue;

    curIndex = (curIndex + 1) % filterSize;
    delay(25);
  }

}

void loop() {
  readMPU();
  cyclicBuffer[curIndex] = (float)AccZ;

  // convolution:
  lpfConvolution = 0;
  hpfConvolution = 0;
  for (int j = 0; j < filterSize; j++ )
  {
    lpfConvolution += cyclicBuffer[(curIndex - j + filterSize) % filterSize] * lpf[j];
    hpfConvolution += cyclicBuffer[(curIndex - j + filterSize) % filterSize] * hpf[j];
  }
  // running average:
  lpfValue = alpha * abs(lpfConvolution) + (1.0 - alpha) * lpfValue;
  hpfValue = alpha * abs(hpfConvolution) + (1.0 - alpha) * hpfValue;
  reactToGestures();
  curIndex = (curIndex + 1) % filterSize;
  delay(30);
}


void reactToGestures()
{  
  if (lpfValue < lpfThreshold) 
  {
    int fraction = knockCounter*10;
    int modulo = knockCounter < 8 ? 10 : 4;
    for (int i = 0; i <= 360; i++)
    {
      if (i%modulo == 0) {
        fraction--; 
      }
      printHelper(SLIDE, knockCounter*10, i, fraction);
    }
    knockCounter = 0;
    empty_loop(LONG_DELAY);
    printHelper(NOTHING, MIN_GLASS, 0, MIN_GLASS);
  }
  else if (hpfValue > hpfThreshold)
  {
    handleKnock();
  }
  else if (lastGesture != NOTHING) {
      printHelper(NOTHING, knockCounter*10, 0, knockCounter*10);
  }
}

void handleKnock(){
    // note: first we need to increase counter, THEN print the updated counter
    if(knockCounter == 4) {
      knockCounter = 8;
    } else {
      knockCounter+=2;
    }
    
    if(knockCounter > 10){ 
      printHelper(MAX_KNOCK, MAX_GLASS, 0, MAX_GLASS);
      delay(20000);
      printHelper(RESET_MAX_KNOCK, MAX_GLASS, 0, MAX_GLASS);
      empty_loop(SHORT_DELAY);
      knockCounter = 0; // note: we must initialize this AFTER getting to MAX
      printHelper(NOTHING, MIN_GLASS, 0, MIN_GLASS);
      empty_loop(SHORT_DELAY);
    } else {
      printHelper(KNOCK,knockCounter*10, 0, knockCounter*10);
      empty_loop(LONG_DELAY);
      printHelper(NOTHING, knockCounter*10, 0, knockCounter*10);
    }  
}

void printHelper(int gesture, int counter, int rotate , int fraction) {
  lastGesture = gesture;
  Serial.print(gesture);
  Serial.print(",");
  Serial.print(counter);
  Serial.print(",");
  Serial.print(rotate);
  Serial.print(",");
  Serial.println(fraction);
}

/*
 * a method to read values from MPU
 */
void readMPU()
{
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr,14,true);
  AccX = Wire.read()<<8|Wire.read();
  AccY = Wire.read()<<8|Wire.read();
  AccZ = Wire.read()<<8|Wire.read();
}
