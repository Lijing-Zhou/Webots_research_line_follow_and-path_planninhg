/*********************************************
 Library Includes
   You can use other C libraries
**********************************************/
//#include <easyx.h>
#include <stdio.h>                   
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h> 
#include <webots/position_sensor.h>
#include <math.h>
#include <stdlib.h>
#include <conio.h>

//#include <iostream>
//#include <vector>
//#include <algorithm>
/*********************************************
 Global variables.  
   Declare any global variables here to keep
   your code tidy.
**********************************************/

// simulation time step is 32ms
#define TIME_STEP 32  
// 3 IR ground sensors
#define NB_GROUND_SENS 3
#define GS_LEFT 0
#define GS_CENTER 1
#define GS_RIGHT 2
#define T 0.7
#define STATE_INITIAL        0
#define STATE_RUN            1
#define STATE_END            1
#define WHEEL_RADIUM        20.5
#define WhEEL_DISTANCE      52
// safe distance
#define OBS1                75
#define OBS2                75
#define MAP_XLIM            400
#define MAP_YLIM            400
#define null                 0
float map[ MAP_XLIM ][ MAP_YLIM ];



static int k=0;
//float map[MAP_X][MAP_X];
WbDeviceTag gs[NB_GROUND_SENS]; /* ground sensors */
unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0};

// Motors
WbDeviceTag left_motor, right_motor;

// LEDs
#define NB_LEDS 8
WbDeviceTag led[NB_LEDS];

// Proximity Sensors
#define NB_PS 8
WbDeviceTag distance_sensor[NB_PS];
double ps_value[NB_PS] = {0, 0, 0, 0, 0, 0, 0,0};
// Position Sensors (encoders)
WbDeviceTag left_position_sensor, right_position_sensor;
int state;
// struct : odometry : posotion of robot


struct Kinematics_s {

  // Left wheel
  float l_last_angle;  // angular position of wheel in last loop()
  float l_delta_angle; // calucated difference in wheel angle.

  // Right wheel
  float r_last_angle;  // angular position of wheel in last loop()
  float r_delta_angle; // calucated difference in wheel angle.

  float x;             // Global x position of robot
  float y;             // Global y position of robot
  float th;            // Global theta rotation of robot.
  float total_diatance; // path distance
  float path_line_x[100000];
  float path_line_y[100000];
}pose;

// record the current distance and the calculated velocity
struct Kinematics_s2 {

   float current_dis; 
   float vright;
   float vleft;
   float vright_obs;
   float vleft_obs;
   
}velocity_dis;



void setup( );
void loop( );
void delay_ms( float ms );
void stop_moving( );
void print_current_time();
void Kinematics();
void moving_backwards();
void run(float destination_x, float destination_y);
void obscale_avoiding();
void find_small();
void runw(float vright,  float vleft,   float vright_obs,   float vleft_obs, float current_dis);

void writeMap();

int main(void) {

  // Initialise Webots - must be done!
  wb_robot_init();

  // Code called here happens before the 
  // simulator begins running properly.
  setup();

  // Run the simulator forever.
  while( wb_robot_step(TIME_STEP) != -1 )
  {
    
     loop();
     Kinematics();
     run(400, 400);
     obscale_avoiding();
     //printf("current distancehhhhh  = %.2f\n",velocity_dis.current_dis);

     runw(velocity_dis.vright,  velocity_dis.vleft,  velocity_dis.vright_obs,  velocity_dis.vleft_obs,  velocity_dis.current_dis); 
    // print();
   // plot_path( pose.path_line_x, pose.path_line_y)
  }
  
   wb_robot_cleanup();

  return 0;
}








void setup() {


  char name[20];
  int i;
  
  // Setup LEDs
  for (i = 0; i < NB_LEDS; i++) {
    sprintf(name, "led%d", i);
    led[i] = wb_robot_get_device(name); /* get a handler to the sensor */
  }
  
  // Setup Ground Sensors
  for (i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name); /* ground sensors */
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }
  
  
  // Setup motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  // get a handler to the position sensors and enable them.
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, TIME_STEP);
  wb_position_sensor_enable(right_position_sensor, TIME_STEP);
  
  // Setup proximity sensors
  for (i = 0; i < 8; i++) {
    
    // get distance sensors 
    sprintf(name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(distance_sensor[i], TIME_STEP);
  }

  pose.l_last_angle = 2.84484;
  pose.l_delta_angle = 0; 
  pose.r_last_angle = 2.85112;  
  pose.r_delta_angle = 0; 
  pose.x = 0;             
  pose.y = 0;            
  pose.th = M_PI/2;
  
}

void loop() {
  
  // Report current time.
  printf("Loop at %.2f (secs)\n", wb_robot_get_time() );
  
  // Get latest ground sensor readings
  gs_value[0] = wb_distance_sensor_get_value(gs[0]);
  gs_value[1] = wb_distance_sensor_get_value(gs[1]);  
  gs_value[2] = wb_distance_sensor_get_value(gs[2]);
   
  // Report ground sensor values
  //printf("Ground sensor values: \n");
  // printf(" 0: %d\n", gs_value[0] );
  // printf(" 0: %d\n", gs_value[1] );
  // printf(" 0: %d\n\n", gs_value[2] );
  
  
  int i;
  for( i = 0; i < NB_PS; i++ ) {

    // read value from sensor
    ps_value[i] =  wb_distance_sensor_get_value(distance_sensor[i]);

    // Print this value to the console to inspect.
    //printf("the distance is %f,   ", ps_value[i]);
    
    
  }
  printf("\n");
  
  
  
  // Call a delay function
   //delay_ms( 500 );
}

void delay_ms( float ms ) {
  float millis_now;
  float millis_future;
  
  millis_now = wb_robot_get_time() * 1000.0;
  millis_future = millis_now + ms;
  
  // Wait for the elapsed time to occur
  // Note, steps simulation, so blocks
  // any further updates the rest of the code.
  while( millis_now < millis_future ) {
    millis_now = wb_robot_get_time() * 1000.0;
    wb_robot_step( TIME_STEP );
  } 
  
  return;
}

void stop_moving() {
    
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
  }

void print_current_time() {

    double  current_time=wb_robot_get_time();
       printf(" current_time: %f\n\n", current_time);
}

void Kinematics() 
{
  // Get current wheel angular positions
  float Current_left = wb_position_sensor_get_value(left_position_sensor);
  float Current_right = wb_position_sensor_get_value(right_position_sensor);
  //printf("Left = %f, Right = %f\n",Current_left,Current_right);


  // Use current and last wheel angular positions
  // to determine l_delta_angle and r_delta_angle
  pose.l_delta_angle = Current_left - pose.l_last_angle;
  pose.r_delta_angle = Current_right - pose.r_last_angle;

  // Since current value has been used, save the current
  // left and right angles into  l_last_angle and 
  // r_last_angle for the next iteration.
  pose.l_last_angle = Current_left;
  pose.r_last_angle = Current_right;

  // Apply delta_angle within the kinematic equations provided.
  // 1) Work out forward contribution of motion
  // 2) Work out theta contribution of rotation
  //float w1=(WHEEL_RADIUM * pose.r_delta_angle)/(2*WhEEL_DISTANCE);
  //float w2=(WHEEL_RADIUM * pose.l_delta_angle)/(2*WhEEL_DISTANCE);
  // v
  float forward_contribution = ((WHEEL_RADIUM * pose.l_delta_angle)/2) + ((WHEEL_RADIUM * pose.r_delta_angle)/2);
  //float v=(w1+w2)*WhEEL_DISTANCE;
  //w
  float theta_contribution = (pose.r_delta_angle-pose.l_delta_angle)* (0.5*(WHEEL_RADIUM)/WhEEL_DISTANCE);
  //float w=w1-w2;

  // Update the X, Y and th values.
  // You can reference the struct in global scope
  //v costh w x sinth (position) th
  float xo=pose.x;
  float yo=pose.y;
  pose.path_line_x[k]=pose.x;
  pose.path_line_y[k]=pose.y;
  pose.path_line_x[k]=pose.path_line_x[k]/10;
  pose.path_line_y[k]=pose.path_line_y[k]/10;
  pose.path_line_x[k]= floor(pose.path_line_x[k]);
  pose.path_line_y[k]= floor(pose.path_line_y[k]);
  k++;
  
  //update the pose
  pose.x  = pose.x + (forward_contribution * cos(pose.th));
  pose.y  = pose.y + (forward_contribution * sin(pose.th));
  
  pose.total_diatance=pose.total_diatance+sqrt((pose.x-xo)*(pose.x-xo) + (pose.y-yo)*(pose.y-yo));
  //angle transfer
  pose.th = pose.th + theta_contribution;
  if(pose.th > (2*M_PI))
  {
    pose.th = pose.th - (2*M_PI);
  }
  else if (pose.th < 0)
  {
    pose.th = pose.th + (2*M_PI);
  }
  printf("x is %.2lf，y is  %.2lf，theta is %.2lf", pose.x, pose.y, pose.th);
  printf("  Total Distance is %.2lf\n", pose.total_diatance);
  
}

void run(float destination_x, float destination_y)
{
  //pose.x, pose.y, pose.th
  // current distance
  velocity_dis.current_dis = sqrt((destination_x-pose.x)*(destination_x-pose.x) + (destination_y-pose.y)*(destination_y-pose.y));
  printf("current distance  = %.2f\n",velocity_dis.current_dis);
  float tan_angle = atan(  (destination_y-pose.y)/(destination_x-pose.x)  );
  // angle transfer (0-2pi)
  float theta=0;
  if ((destination_y-pose.y)>0 && (destination_x-pose.x)>0)
     theta=tan_angle;
  if ((destination_y-pose.y)>0 && (destination_x-pose.x)<0)
     theta=tan_angle+M_PI;
  if ((destination_y-pose.y)<0 && (destination_x-pose.x)<0)
     theta=tan_angle+M_PI;
  if ((destination_y-pose.y)<0 && (destination_x-pose.x)>0)
     theta=tan_angle+2*M_PI;
  if ((destination_y-pose.y)>0 && (destination_x-pose.x)==0)
     theta=M_PI/2;
  if ((destination_y-pose.y)<0 && (destination_x-pose.x)==0)
     theta=3*M_PI/2;
  if ((destination_y-pose.y)==0 && (destination_x-pose.x)>0)
     theta=0;
  if ((destination_y-pose.y)==0 && (destination_x-pose.x)<0)
     theta=M_PI;


  printf("tan_angle = %.2f  ",tan_angle);
  printf("theta = %.2f  ",theta);

  //total distance
  float distance_start_position_to_destination = sqrt(destination_x*destination_x + destination_y*destination_y);
  //initial theta
  float angle_start_position_to_destination= atan(destination_y/destination_x);
  float e_theta = pose.th - theta;
  printf("pose.th = %.2lf  ",  pose.th);
  printf("e_theta = %.2f\n",e_theta);
  float speed_factor = velocity_dis.current_dis/1;

  float turn_velocity_base;
  turn_velocity_base = 0.8 ;
  //velocity( no obstacle only navigation)
  velocity_dis.vright=0.1-turn_velocity_base/(M_PI+0.1)*e_theta;
  velocity_dis.vleft=0.1+turn_velocity_base/(M_PI+0.1)*e_theta;
  printf("right velocity = %.2f  ", velocity_dis.vright);
  printf("  left velocity = %.2f\n", velocity_dis.vleft);
   //moving_backwards();
  // Set motor values.
  /*
  if (current_dis>=2)
  {
  
  wb_motor_set_velocity(right_motor, vright);
  wb_motor_set_velocity(left_motor, vleft);
  }
  else
  {
   wb_motor_set_velocity(right_motor, 0.0);
   wb_motor_set_velocity(left_motor, 0.0);
  }
  */


 
}

void moving_backwards() {
    
    wb_motor_set_velocity(left_motor, 0.5);
    wb_motor_set_velocity(right_motor, 0.5);
  }

void obscale_avoiding()
{
 int i;
 float e_theta;
 //ps_valve ( analysis)
  for( i = 0; i < NB_PS; i++ ) {
    printf("PS%d: %f,   ", i ,ps_value[i]); 
  }
  printf("\n");
 
 // the two arraies are for analysis
  float ps_angle_valve_base[8]={1.27, 0.77, 0, 5.21, 4.21, M_PI, 2.37, 1.87};
  float ps_angle_valve[8]={0.30, 0.80, 1.57, 2.64, -2.64, -1.57, -0.80, -0.30};
      
  
  for( i = 0; i < NB_PS; i++ ) {
   printf("%.2f ", ps_angle_valve[i]);
  }
  printf("\n");
  
  int a[8]={0,0,0,0,0,0,0,0};
  int num_obs=0;

  //VFH+ select suitable histogram according to the safe ditance
  for( i = 0; i < NB_PS; i++ ) 
  {
      if ( ps_value[i]<OBS2)
      {
      a[i]=1; 
      num_obs++;
      }
      if ( ps_value[i]>OBS2)
      {
      a[i]=2;
      }
  }
  

  for( i = 0; i < NB_PS; i++ ) 
   {
      printf("%d ", a[i]);
   }
      printf("\n");
      printf("num_obs: %d",num_obs);
      printf("\n");
  
  //e_theta

  float e_theta_obs[8]={0,0,0,0,0,0,0,0};
  float e_theta_obs_abs[8]={0,0,0,0,0,0,0,0};

  // use the histogram to select the most suitable section and calculate the angle( velocity)
  for( i = 0; i < NB_PS; i++ ) 
   {
      if (a[i]==1)
      {
       e_theta_obs[i]=ps_angle_valve[i]-e_theta;
       e_theta_obs_abs[i]=abs(e_theta_obs[i]*100);              
      }
   }
    printf( "e_theta_obs:      ") ;
    
     for( i = 0; i < NB_PS; i++ ) 
   {
      printf( "%.2f    ", e_theta_obs[i]);   
   }
   printf("\n");
   
   printf( "e_theta_obs_abs:  ") ;
     for( i = 0; i < NB_PS; i++ ) 
   {
      printf( "%.2f    ", e_theta_obs_abs[i]);   
   }
   printf("\n");
   // small choose
   find_small( e_theta_obs_abs);
   int arr_min_abs;
   printf( "arr_min_abs: %d", arr_min_abs); 
   printf("\n");
   float w_etheta_obs;
   w_etheta_obs=e_theta_obs[i];
   
  float turn_velocity_base;
  turn_velocity_base = 0.8 ;
  // velocity calculation
  //velocity_dis.vright_obs=0.1-turn_velocity_base/(M_PI+0.1)* w_etheta_obs/100;
  //velocity_dis.vleft_obs=0.1+turn_velocity_base/(M_PI+0.1)* w_etheta_obs/100;
  if (w_etheta_obs >= 0.05)
  {
  velocity_dis.vright_obs=0.01;
  velocity_dis.vleft_obs=0.01+0.5;
  }
  
  if (w_etheta_obs <=-0.05)
  {
  velocity_dis.vright_obs=0.01+0.5;
  velocity_dis.vleft_obs=0.1;
  }
  
  if (w_etheta_obs <= 0.05 && w_etheta_obs >= -0.05) 
  {
  velocity_dis.vright_obs=0.05;
  velocity_dis.vleft_obs=0.05;
  }
  
  
  
  
  
  printf("right velocity_obs = %.2f  ", velocity_dis.vright_obs);
  printf("  left velocity_obs = %.2f\n", velocity_dis.vleft_obs);
}

void find_small( float e_theta_obs_abs[8])
{
 float min_abs=1000;
 int arr_min_abs=0;
 
  for(int  i = 0; i < NB_PS; i++ ) 
   {
      if (e_theta_obs_abs[i]< min_abs )
      {
      arr_min_abs=i;
      }    
   }
   //printf( "arr_min_abs: %d", arr_min_abs); 
}







void runw(float vright,  float vleft,   float vright_obs,   float vleft_obs, float current_dis)
{ 
    int state_e=0;
    if (current_dis>=2) // not reach the end point
      {
        //no obstacle
         if( ps_value[0]<OBS1 && ps_value[1]<OBS1 && ps_value[2]<OBS1 && ps_value[3]<OBS1 && ps_value[4]<OBS1 && ps_value[5]<OBS1 && ps_value[6]<OBS1 && ps_value[7]<OBS1 )
      {
       wb_motor_set_velocity(right_motor, vright);
       wb_motor_set_velocity(left_motor, vleft);
       state_e=1;
      }
       else 
       {//meet obstacle
        wb_motor_set_velocity(right_motor, vright_obs);
        wb_motor_set_velocity(left_motor, vleft_obs);
        state_e=2;
     }
  
  }
  else
     {
       //stop (meet the end point)
       wb_motor_set_velocity(right_motor, 0.0);
       wb_motor_set_velocity(left_motor, 0.0);
        state_e=3;
     }
    printf( "STATE: %d",state_e ); 
}

/*void plot_path(float path_line_x,float path_line_y)
/int main()
{
   FILE *fp = NULL;
 
   fp = fopen("/C:\ff\aa.txt", "w+");
   fprintf(fp, "This is testing for fprintf...\n");
   fputs("This is testing for fputs...\n", fp);
   fclose(fp);
}



}
*/


/*
void writeMap( ) {
  int x;
  int y;
float obs_x;
float obs_y;

// To temporarily store x,y index of map
int map_x;
int map_y;

   for( y = 0; y < MAP_YLIM; y++ ) {    // start at row 0
        for( x = 0; x < MAP_XLIM; x++ ) {  // for all columns
      // Sets location x,y in array to 0.0
       map[x][y] = 0.0;
      // Reads and prints value at location x,y
      // to 2 decimal places.
        //printf("%.2f, ", map[x][y] );

  }
   }

  map_x = (int)pose.path_line_x;
  map_y = (int)pose.path_line_y; 
  map[map_x][map_y] = map[map_x][map_y] + 1;
 

for ( int o=0; o++; o <10000)
{
     FILE *cor_x = NULL;
     cor_x = fopen("C:\Users\31950\Desktop\pathx.txt","a");
     fprintf(cor_x,"%d", map_x(o); 
     fclose(fp);
}
}
*/

