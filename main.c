#include <kipr/wombat.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
// PID controller state
typedef struct pid_controller_struct {
    double Kp;           // PID tuning value
    double Ki;           // PID tuning value
    double Kd;           // PID tuning value
    double integral_max; // anti-windup term (see pid_set_integral_max())
    double last_time;    // time of last call to pid_update()
    double last_error;   // last error value
    double integral;     // integral term accumulator
    bool debug_log;      // enable/disable PID tuning output to stdout
    int integral_reset;  // integral reset value
    int integral_reset_count; // integral reset counter
} pid_controller;

typedef struct robot_pos_struct {
    double x;
    double y;
    double angle;        // Radians
} Robot_pos;

/* Set PID parameters here.  Also initialized PID controller stuct to a
 * valid starting state.  PID debug logging to stdout is disabled by
 * default.*/
void pid_init(pid_controller *pid, double Kp, double Ki, double Kd);
void pid_enable_log(pid_controller *pid);
void pid_disable_log(pid_controller *pid);
void pid_set_integral_max(pid_controller *pid, const double max);
double pid_update(pid_controller *pid, const double setpoint, const double actual);

/* Sends command to external PC to attack other robots.*/
void killswitch();

//CH0 - green
//CH1 - Yellow
//CH2 - Purple
//CH3 - QR
const uint8_t QR_CHANNEL = 3;

// --- ODOMETRY GLOBALS & CONSTANTS ---
volatile Robot_pos robot_pos;
pthread_t odom_thread;
void * odom_main(void *);

const double WHEELBASE = 6.65; 
const double WHEEL_DIAMETER = 2.75;
const double PI = 3.14159265358979;
const double TICKS_PER_REV = 1833.0; 
const int LEFT_MOTOR = 3;           
const int RIGHT_MOTOR = 0;           
const int GATE_MOTOR = 2;
char qr_color = 'g';
void turn_rad(float rads);
void go_straight(float dist_in);
void go_home();
void read_qr();
void aquire_block();
bool attempt_grab();
int map_qr_to_channel(char qr_color);
void close_gate();
void open_gate();
int main()
{
    if(camera_load_config("mock_comp.conf")){
        printf("ERROR: could not load config, aborting.\n");
        exit(1);
    }
    if(!camera_open()){
        printf("ERROR: could not open camera, aborting\n");
        exit(1);
    }
    printf("Press A when robot is placed in starting position.\n");
    while(!a_button_clicked());
    printf("Initialization sequence started.\n");
    cmpc(GATE_MOTOR);
    // Initialize global position
    robot_pos.x = 0.0;
    robot_pos.y = 0.0;
    robot_pos.angle = 0.0;

    // Start Odometry Tracking in the background
    if(pthread_create(&odom_thread, NULL, odom_main, NULL)) {
        printf("Error creating odometry thread\n");
        return 1;
    }
    //  printf("Press A again to start.\n");
    //  while(!a_button_clicked());
    printf("Wait for \"ODOMETERY OK\" message,\n then press B to start.\n");
    while(!b_button_clicked());
    printf("PROGRAM STARTED\n");
    killswitch();

    shut_down_in(119);
    open_gate();
    //   msleep(5000);
    //  close_gate();
    // while(1);;
    go_straight(7.5);
    turn_rad(PI);

    read_qr();
    // while(1);;
    turn_rad(PI);
    go_straight(6);
    aquire_block();
    go_home();

    while(1);;


    printf("Hello World\n");

    while(1){
        camera_update();
        if(get_object_count(QR_CHANNEL)){
            printf("%c\n",'d');
        }

        // Optional: Print odometry for debugging (convert radians to degrees for readability)
        // printf("X: %.2f Y: %.2f Angle(deg): %.2f\n", robot_pos.x, robot_pos.y, robot_pos.angle * (180.0/PI));
    }

    return 0;
}

// ==========================================
// Odometry Background Thread
// ==========================================
void * odom_main(void * args){
    bool one_shot = true;
    // 1. SET HIGH PRIORITY
    struct sched_param param;
    param.sched_priority = 80; // High priority (1-99)

    // SCHED_FIFO is a real-time policy. It will pre-empt the main loop 
    // whenever the odom math needs to run.
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        printf("Warning: Failed to set real-time priority.\n");
    }
    // printf("Odometry thread start successful\n");
    cmpc(LEFT_MOTOR);
    cmpc(RIGHT_MOTOR);
    double log_time = 0;

    log_time = seconds();
    const bool enable_log = false;
    int last_ticks_l = 0;
    int last_ticks_r = 0;
    double dist_per_tick = (PI * WHEEL_DIAMETER) / TICKS_PER_REV;

    while(1){
        // 1. Read current encoders
        int current_ticks_l = gmpc(LEFT_MOTOR);
        int current_ticks_r = gmpc(RIGHT_MOTOR);

        // 2. Calculate change in ticks
        int delta_ticks_l = current_ticks_l - last_ticks_l;
        int delta_ticks_r = current_ticks_r - last_ticks_r;

        // Update last ticks for the next loop
        last_ticks_l = current_ticks_l;
        last_ticks_r = current_ticks_r;

        // 3. Convert ticks to physical distance (inches)
        double delta_d_l = delta_ticks_l * dist_per_tick;
        double delta_d_r = delta_ticks_r * dist_per_tick;

        // 4. Calculate center distance and angle change
        double delta_d = (delta_d_r + delta_d_l) / 2.0;
        double delta_theta = (delta_d_r - delta_d_l) / WHEELBASE;

        // 5. Update global position (volatile struct)
        robot_pos.x -= delta_d * cos(robot_pos.angle + (delta_theta / 2.0));
        robot_pos.y -= delta_d * sin(robot_pos.angle + (delta_theta / 2.0));
        robot_pos.angle += delta_theta;
        // robot_pos.angle=fmod(robot_pos.angle,2*PI);
        if(one_shot){
            //printf("angle: %.1f (%.1f,%.1f)\n",robot_pos.angle*180 *(1/PI),robot_pos.x,robot_pos.y);
            printf("ODOMETERY OK\n");
            one_shot = false;
        }
        if(log_time+0.5<seconds() && enable_log){

            printf("angle: %.1f (%.1f,%.1f)\n",robot_pos.angle*180 *(1/PI),robot_pos.x,robot_pos.y);
            log_time=seconds();

        }
        usleep(5000);
    }
    return NULL;
}


// ==========================================
// PID & Utilities
// ==========================================

void pid_init(pid_controller *pid, double Kp, double Ki, double Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0;
    pid->last_error = 0;
    pid->last_time = 0;
    pid->integral_reset = 500;
    pid->integral_reset_count = 0;
    pid->debug_log = false;
    pid->integral_max = 0;
}

void pid_set_integral_max(pid_controller *pid, const double max) {
    pid->integral_max = max;
}

void pid_enable_log(pid_controller *pid) { pid->debug_log = true; }
void pid_disable_log(pid_controller *pid) { pid->debug_log = false; } // Fixed to false

double pid_update(pid_controller *pid, const double setpoint, const double actual) {
    const double error = setpoint - actual;
    const double curr_time = (double)seconds();
    const double delta_time = curr_time - pid->last_time;
    const double delta_error = error - pid->last_error;
    // Prevent divide by zero on first loop
    double error_ddt = 0;
    if (delta_time > 0) {
        error_ddt = delta_error / delta_time;
    }

    pid->last_error = error;
    pid->last_time = curr_time;

    const double proportional = error * pid->Kp;
    const double deriv = error_ddt * pid->Kd;

    pid->integral += pid->Ki * delta_time * error;
    pid->integral_reset_count++;
    if(pid->integral_reset_count == pid->integral_reset){
        pid->integral = 0;
        pid->integral_reset_count = 0;
    }

    if (pid->integral_max != 0) {
        if (pid->integral >= pid->integral_max) {
            pid->integral = pid->integral_max;
        }
        // FIXED: Changed to clamp against negative limit instead of positive
        else if (pid->integral <= -pid->integral_max) { 
            pid->integral = -pid->integral_max;
        }
    }

    if (pid->debug_log) {
        printf("prop: %.4f, deriv %.4f, integral %.4f\n", proportional, deriv, pid->integral);
    }
    return proportional + deriv + pid->integral;
}

void killswitch(){
    system("echo \"START\" | nc -q 0 192.168.125.100 9000 & ");
}
void motor_filt(int motor_port,int power){
    const int threshold = 40;
    if(power > threshold){
        power = threshold;
    }
    if(-power>threshold){
        power = -threshold;
    }
    motor(motor_port,power);

}

//turns a relative number of degrees from the current heading.  Turns in shortest direction
void turn_rad(float rads){
    rads+=robot_pos.angle;
    const float final_error_rads = PI/200.0;
    const float timeout_sec = 8; //seconds
    pid_controller turn_pid;
    pid_init(&turn_pid,2,0,0.015);
    // pid_enable_log(&turn_pid);
    const double start = seconds();
    while(fabs(rads-robot_pos.angle)>=final_error_rads && seconds()-start < timeout_sec){
        //  printf("debug %.2f, (%.3f)\n",rads,robot_pos.angle-rads); 
        const double rotation = 100*pid_update(&turn_pid,rads,robot_pos.angle);
        motor_filt(LEFT_MOTOR,-rotation);
        motor_filt(RIGHT_MOTOR,rotation);
    }
    ao();
}
double get_distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    // Using hypot() is actually faster and more precise than sqrt(dx*dx + dy*dy)
    return hypot(dx, dy); 
}
//moves a distance in inches, negative is backwards
void go_straight(float dist_in){
    const float final_error_in = (2.0/12)*dist_in;
    const float timeout_sec = 3; //seconds
    const int sign = (dist_in >=0)?1:-1;
    pid_controller straight_pid;
    pid_init(&straight_pid,2,0,0.015);
    float end_x = robot_pos.x + dist_in*cos(robot_pos.angle);
    float end_y = robot_pos.y + dist_in*sin(robot_pos.angle);
    const double start = seconds();
    float error = get_distance(robot_pos.x,robot_pos.y,end_x,end_y);
    while(error>=final_error_in && seconds()-start < timeout_sec){
        const double power = 100*pid_update(&straight_pid,0,error);
        //   printf("debug %.2f, (%.3f,%.3f)\n",error,end_x,end_y); 
        motor_filt(LEFT_MOTOR,sign*power);
        motor_filt(RIGHT_MOTOR,sign*power);
        error = get_distance(robot_pos.x,robot_pos.y,end_x,end_y);
    }
    ao();
}

void go_home(){
    //we want to be facing +PI/2 rads
    turn_rad(PI/2-robot_pos.angle);
    while(robot_pos.y <0){
        motor_filt(LEFT_MOTOR,-100);
        motor_filt(RIGHT_MOTOR,-100);
    }
    ao();

    turn_rad(PI-robot_pos.angle);
    while(true){
        motor_filt(LEFT_MOTOR,-100);
        motor_filt(RIGHT_MOTOR,-100);
    }
    ao();


}
void read_qr(){
    bool qr_found = false;
    int i =0;
    while(!qr_found && i <30){
        msleep(1000);

        camera_update();
        i++;
        if(get_object_count(3)){
            qr_color = get_object_data(3,0)[0];
            printf("QR read success (read as \"%c\")\n",qr_color);
            qr_found = true;
        }
        else{
            printf("QR read fail, retrying...\n");
            turn_rad(PI-robot_pos.angle);

        }
    }
    if(qr_found == false){
        printf("Could not read QR code. \n Aborting...\n");
        exit(1);
        return;
    }
}
int map_qr_to_channel(char qr_color){
    if(qr_color == 'p'){
        return 2;
    }
    if(qr_color == 'y'){
        return 1;
    }
    if(qr_color == 'g'){
        return 0;
    }
    printf("ERROR: QR DOES NOT MAP TO CHANNEL\n");
    exit(1);
    return 0;
}
void aquire_block(){
    const int area_threshold = 20;
    int channel = map_qr_to_channel(qr_color);
    printf("QR channel is %i\n",channel);
    bool block_aquired = false;
    bool target_lock = false;
    //    int camera_open_device_model_at_res(int number, enum Model model, enum Resolution res);
    //get_camera_width() get_camera_height()
    //get_object_area() 

    //point2 get_object_center	(	int 	channel,int obj);
    pid_controller rotation_pid;
    pid_init(&rotation_pid,0.5,0,0.015);
    // pid_enable_log(&rotation_pid);
    //pid_enable_log(&distance_pid);
    // pid_enable_log(&rotation_pid);
    const double x_midpoint = get_camera_width()/2;
    printf("Midpoint is %i of total width %i\n",(int)x_midpoint,get_camera_width());
    while(!block_aquired){
        camera_update();
        //if object is detected
        if(get_object_count(channel)){
            if(get_object_area(channel,0) < area_threshold){
                continue;
            }
            //  printf("OBJ\n");
            //(channel,object)
            const point2 object_pos= get_object_center(channel,0);
            const double rotation = pid_update(&rotation_pid,x_midpoint,object_pos.x);
            ///  printf("Error:  %.1f\n",x_midpoint-object_pos.x);
            motor_filt(LEFT_MOTOR,-15+rotation);
            motor_filt(RIGHT_MOTOR,-15-rotation);
            target_lock = true;
        }
        else{
            if(!target_lock){
                motor_filt(LEFT_MOTOR,20);
                motor_filt(RIGHT_MOTOR,-20);
            }
            else{
                block_aquired = attempt_grab();
            }
            target_lock = false;
        }


    }
    ao();
}
void close_gate(){
    mrp(GATE_MOTOR,-500,900);
}
void open_gate(){
    mtp(GATE_MOTOR,500,-900);
}
bool attempt_grab(){
    static int grab_attempts = 0;
    grab_attempts++;
    if(grab_attempts>=7){
        printf("Grab limit reached\n");
        return true;}
    const int threshold = 2000;
    printf("Attempting grab\n");
    go_straight(6);
    if(analog(5)>threshold){
        printf("Grab success\n");
        close_gate();
        return true;
    }else{
        return false;
    }
}