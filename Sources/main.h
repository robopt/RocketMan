// wall PID values
double Kp = 1;
int lastError = 0;
   
int sum, senSum;
int error, deltaError;
int err_right, err_left;
int lastFront = 0;
int moveRight = 0;
int moveLeft = 0;

///////////////////////
// Motor constraints //
///////////////////////
//base speed
int BASE = 45;


//limitations
const int MAX = 60;
const int MIN = 30; 

//motor variables 
int routput,loutput;

int isSetup = 0;


//sensor variables
const int sensorCount = 4;
const int sensorStart = 4;
int s0,s1,s2,s3,s4,s5,s6,s7;
int s4avg = 0, s5avg = 0, s6avg = 0, s7avg = 0;
int sn = 10;
int mid = 0;

//function defs
void portInit(void);
void averageSensors(void);
void forward(int speed);
void forwardP();
void right(int lspeed);
void left(int rspeed);
void leftManual(int lspeed, int rspeed);
void back(int speed);
void backSplit(int lspeed, int rspeed);
void backLeft(int rspeed, int lspeed);
void stop(void);
void setup(void);
