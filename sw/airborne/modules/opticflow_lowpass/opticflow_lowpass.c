/**
 * @file "modules/opticflow_lowpass/opticflow_lowpass.c"
 * @author Group 5
 * Avoid Obstacles with optic flow. The model gets the divegence size from the cv_opticflow module and based of a certain threshold
 *  it makes the drone perform certain manouvers. The main obstacle mavouver is a turn of 30 deg while is the drone goes out of bounds 
 * the mavounver will be a 180 deg turn. This module can be expanded with any parameter from the optic flow module. 
 * 
 * The documentation for this module follows the report written by group 5
 */

#include "modules/opticflow_lowpass/opticflow_lowpass.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "modules/datalink/telemetry.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C 
#include "generated/flight_plan.h"

#define OPTICFLOW_LOWPASS_VERBOSE TRUE

#ifndef OFL_OPTICAL_FLOW_ID
#define OFL_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OFL_OPTICAL_FLOW_ID)

#define PRINT(string,...) fprintf(stderr, "[opticflow_lowpass->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OPTICFLOW_LOWPASS_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif





static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t moveWaypointBackward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t calculateBackwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE1,
  SAFE2,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  TURN180,
  OUT_OF_BOUNDS
};

// define settings
float div_thr = 0.1f;
float divergence = 0;
float divergence_vision;
float vision_time, vision_time_prev;


// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
float heading_increment = 30.f;          // heading angle increment [deg]
float maxDistance = 1.25;               // max waypoint displacement [m]
float Total_Distance_before_turn = 0.f;
float Total_Distance_back = 0.f;
float next_heading = 0.f;

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
static abi_event optical_flow_ev;

// Reading from sensors:

void vertical_ctrl_optical_flow_cb(uint8_t sender_id UNUSED, uint32_t stamp,
                                   int32_t flow_x UNUSED, int32_t flow_y UNUSED,
                                   int32_t flow_der_x UNUSED, int32_t flow_der_y UNUSED, float quality UNUSED, float size_divergence)
{
  divergence_vision = size_divergence;
  vision_time = ((float)stamp) / 1e6;
}

static void send_divergence(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DIVERGENCE(trans, dev, AC_ID,
                           &divergence, &divergence_vision, &div_thr,
                           &vision_time, &vision_time_prev, &maxDistance, &next_heading);
}


/*
 * Initialisation function, setting divergence vision
 */
void opticflow_lowpass_init(void)
{
  // Initialise random values
  divergence_vision = 0.;

  
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  //Binding Messages
  AbiBindMsgOPTICAL_FLOW(OFL_OPTICAL_FLOW_ID, &optical_flow_ev, vertical_ctrl_optical_flow_cb);

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DIVERGENCE, send_divergence);

}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void opticflow_lowpass_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }
  //Time for divergence
  float dt = vision_time - vision_time_prev;
 
  VERBOSE_PRINT(" state: %d div : %f div_thr: %f \n", navigation_state, div_thr);
  // Divergence Caclulations
  float moveDistance = maxDistance;
  float div_factor = 1.28;
  float new_divergence = (divergence_vision*div_factor)/dt;
  //lowpass
  divergence += (new_divergence - divergence)*0.7;

  switch (navigation_state){
    case SAFE1:
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);

      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else if (divergence > div_thr){
        VERBOSE_PRINT("found /n");
        navigation_state = OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
        Total_Distance_before_turn+=moveDistance;
        VERBOSE_PRINT("Moved %f before turn.\n",Total_Distance_before_turn);
      }
      

      break;
    case SAFE2:
      // Move waypoint backrward
      moveWaypointBackward(WP_TRAJECTORY, 0.5f * Total_Distance_before_turn);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
          waypoint_move_here_2d(WP_GOAL);
          waypoint_move_here_2d(WP_TRAJECTORY);
          navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      else {
          moveWaypointBackward(WP_GOAL, 0.5f * Total_Distance_before_turn);
      }
      Total_Distance_before_turn=0;
      navigation_state = SEARCH_FOR_SAFE_HEADING;
      VERBOSE_PRINT("Current state: %d", navigation_state);
      break;
    case OBSTACLE_FOUND:
      // stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      //reset total moving distance
      VERBOSE_PRINT("*********Turing after %f due to OB and reset total distance.\n",Total_Distance_before_turn);


      // randomly select new search direction

      navigation_state = SAFE2;
      VERBOSE_PRINT("From OB, Enter SAFE2 state");


      break;
    case SEARCH_FOR_SAFE_HEADING:
      increase_nav_heading(heading_increment);

      // make sure we have a couple of good readings before declaring the way safe
      
      navigation_state = SAFE1;
      
      VERBOSE_PRINT("From SH, Enter SAFE1 state");
      break;
    case TURN180:
      increase_nav_heading(90.f);
      increase_nav_heading(90.f);
      VERBOSE_PRINT("Finish 180 turn");
      navigation_state = SAFE1;
      VERBOSE_PRINT("Enter SAFE1 state");
      break;
    case OUT_OF_BOUNDS:
      // stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      VERBOSE_PRINT("Out of BOund\n");
      
      navigation_state = TURN180;
      VERBOSE_PRINT("From OUT OF BOUND, Enter SAFE2 state");
      

      break;
    default:
      break;
  }
    vision_time_prev = vision_time;
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  // for performance reasons the navigation variables are stored and processed in Binary Fixed-Point format
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of distance backward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointBackward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateBackwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateBackwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x - POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y - POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}
/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = 10.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = 10.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}

