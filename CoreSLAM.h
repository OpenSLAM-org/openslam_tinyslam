#ifndef _TINYSLAM_H_
#define _TINYSLAM_H_

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TS_SCAN_SIZE 8192
#define TS_MAP_SIZE 2048
#define TS_MAP_SCALE 0.1
#define TS_NO_OBSTACLE 65500
#define TS_OBSTACLE 0

typedef unsigned short ts_map_pixel_t;

typedef struct {
    ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE];    
} ts_map_t;

typedef struct {
    double x[TS_SCAN_SIZE], y[TS_SCAN_SIZE];
    int value[TS_SCAN_SIZE];
    int nb_points;
} ts_scan_t;

typedef struct {
    double x, y;    // in mm
    double theta;   // in degrees
} ts_position_t;

typedef struct {
    unsigned int timestamp;
    int q1, q2;                // Odometry information
    double v, psidot;          // Used to correct the scans according to the speed of the robot
    ts_position_t position[3]; // 0 : forward - 1 : backward - 2 : final / closed loop
    int d[TS_SCAN_SIZE];
} ts_sensor_data_t;

void ts_map_init(ts_map_t *map);
int ts_distance_scan_to_map(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos);
void ts_map_update(ts_scan_t *scan, ts_map_t *map, ts_position_t *position, int quality, int hole_width);

// Stochastic part
typedef struct {
    unsigned long jz;
    unsigned long jsr;
    long hz;
    unsigned long iz;
    unsigned long kn[128];
    double wnt[128];
    double wn[128];
    double fn[128];
} ts_randomizer_t;

double ts_random_normal_fix(ts_randomizer_t *d);
double ts_random_normal(ts_randomizer_t *d, double m, double s);
void ts_random_init(ts_randomizer_t *d, unsigned long jsrseed);
double ts_random(ts_randomizer_t *d);
long ts_random_int(ts_randomizer_t *d, long min, long max);

ts_position_t ts_monte_carlo_search(ts_randomizer_t *randomizer, ts_scan_t *scan, ts_map_t *map, ts_position_t *start_pos, double sigma_xy, double sigma_theta, int stop, int *bestdist);

// Extensions
double ts_distance(ts_position_t *pos1, ts_position_t *pos2);
void ts_save_map_pgm(ts_map_t *map, ts_map_t *overlay, char *filename, int width, int height);
void ts_draw_scan(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos);
void ts_draw_scan_RGB(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos, unsigned char *pixmap, int scale, int reversey);

typedef struct {
    double r;	    // length wheels' radius
    double R;	    // half the wheels' axis length
    int inc;	    // wheels' counters increments per turn
    double ratio;   // ratio between left and right wheel
} ts_robot_parameters_t;

typedef struct {
    double offset;  // position of the laser wrt center of rotation
    int scan_size;  // number of points per scan
    int angle_min;  // start angle for scan
    int angle_max;  // end angle for scan
    int detection_margin; // first scan element to consider
    double distance_no_detection; // default value when the laser returns 0
} ts_laser_parameters_t;

typedef struct {
    ts_randomizer_t randomizer;
    ts_map_t *map;
    ts_robot_parameters_t params;
    ts_laser_parameters_t laser_params;
    ts_position_t position;
    int q1, q2;
    unsigned int timestamp;
    double psidot, v;
    double distance;
    int hole_width;
    int direction;
    int done, draw_hole_map;
    ts_scan_t scan;
    double sigma_xy;
    double sigma_theta;
} ts_state_t;

#define TS_DIRECTION_FORWARD   0
#define TS_DIRECTION_BACKWARD  1
#define TS_FINAL_MAP 2

void ts_state_init(ts_state_t *state, ts_map_t *map, ts_robot_parameters_t *params, ts_laser_parameters_t *laser_params, ts_position_t *position, double sigma_xy, double sigma_theta, int hole_width, int direction);
void ts_build_scan(ts_sensor_data_t *sd, ts_scan_t *scan, ts_state_t *state, int span);
void ts_iterative_map_building(ts_sensor_data_t *sd, ts_state_t *state);

// Loop closing
ts_position_t ts_close_loop_position(ts_state_t *state, ts_sensor_data_t *sensor_data, ts_map_t *loop_close_map, ts_position_t *start_position, int *q);
void ts_close_loop_trajectory(ts_sensor_data_t *sensor_data, int maxscans, 
        ts_position_t *startpos, ts_position_t *close_loop_position);


#endif // _TINYSLAM_H_
