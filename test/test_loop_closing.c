#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifdef USE_SDL
#ifdef WIN32
#include "SDL.h"
#include "SDL_thread.h"
#include "SDL_rotozoom.h"
#else
#include "SDL/SDL.h"
#include "SDL/SDL_thread.h"
#include "SDL/SDL_rotozoom.h"
#endif
#endif
#include "CoreSLAM.h"

#define TEST_FILENAME "test_lab2"

#define TEST_MAX_SCANS 5000
#define TEST_SCAN_SIZE 682

void ts_map_empty(ts_map_t *map)
{
    int x, y;
    ts_map_pixel_t *ptr;
    for (ptr = map->map, y = 0; y < TS_MAP_SIZE; y++) {
	for (x = 0; x < TS_MAP_SIZE; x++, ptr++) {
	    *ptr = 65535;
	}
    }
}

ts_sensor_data_t sensor_data[TEST_MAX_SCANS];

int ts_read_scans(char *filename, ts_sensor_data_t *sensor_data)
{
    FILE *input;
    char *str, line[4000];
    int i, nbscans = 0;
    ts_sensor_data_t *sd;

    input = fopen(filename, "rt");
    do {    
        sd = &sensor_data[nbscans];
        // Read the scan
        str = fgets(line, 4000, input);
        if (str == NULL) break;
        str = strtok(str, " ");
        sscanf(str, "%u", &sd->timestamp);
        str = strtok(NULL, " ");
        str = strtok(NULL, " ");
        sscanf(str, "%d", &sd->q1);
        str = strtok(NULL, " ");
        sscanf(str, "%d", &sd->q2);
        for (i = 0; i < 21; i++)
            str = strtok(NULL, " ");
        for (i = 0; i < TEST_SCAN_SIZE; i++) {
            if (str) {
                sscanf(str, "%d", &sd->d[i]);
                str = strtok(NULL, " ");
            } else sd->d[i] = 0;
        }
        nbscans++;
    } while (1);
    fclose(input);
    return nbscans;
}

void ts_draw_map(ts_map_t *map, ts_map_t *map_scans, ts_sensor_data_t *sensor_data, ts_state_t *state, int maxscans, int direction)
{
    int i;
    ts_position_t pos;

    ts_scan_t scan2map;
    ts_map_init(map);
    if (map_scans) ts_map_empty(map_scans);
    for (i = 0; i != maxscans; i++) {
        ts_build_scan(&sensor_data[i], &scan2map, state, 3);
        pos = sensor_data[i].position[direction];
        pos.x += state->laser_params.offset * cos(pos.theta * M_PI / 180);
        pos.y += state->laser_params.offset * sin(pos.theta * M_PI / 180);
        ts_map_update(&scan2map, map, &pos, 50, state->hole_width);
        if (map_scans)
            ts_draw_scan(&scan2map, map_scans, &pos);
    }
}

ts_map_t trajectory, map, loop_map, map_scans;
    
#ifdef USE_SDL
unsigned char pixmap[TS_MAP_SIZE * TS_MAP_SIZE * 3];
SDL_Surface *screen, *pixmap_surface, *arrow_up;
SDL_cond *draw_it;
SDL_mutex *drawing_mutex;

const int DISPLAY_SIZE = 768;

int drawing_thread(void *s)
{
    ts_map_pixel_t *ptr;
    int w, x, y;
    double xr, xrp, yr, yrp, theta;
    SDL_Rect rect;
    SDL_Surface *image, *rotate;
    ts_state_t *state = (ts_state_t *)s;
    ts_position_t position;

    do {
        SDL_LockMutex(drawing_mutex);
        SDL_CondWait(draw_it, drawing_mutex);

#define ZOOM 1 
        xr = state->position.x - 0.5 * TS_MAP_SIZE / TS_MAP_SCALE;
        yr = state->position.y - 0.5 * TS_MAP_SIZE / TS_MAP_SCALE;
        theta = (-state->position.theta + 90) * M_PI / 180;
        xrp = xr * cos(theta) - yr * sin(theta);
        yrp = xr * sin(theta) + yr * cos(theta);
        xrp *= 0.5 * TS_MAP_SCALE * ZOOM; yrp *= 0.5 * TS_MAP_SCALE * ZOOM; // Scale to rotate surface size

        // Create a map twice smaller than the original (TS_MAPS_SIZE / 2)
        if (state->draw_hole_map) {
            for (ptr = state->map->map, y = 0; y < TS_MAP_SIZE / 2; y++) {
                ptr = &state->map->map[(TS_MAP_SIZE -1 - y * 2) * TS_MAP_SIZE];
                for (x = 0; x < TS_MAP_SIZE / 2; x++, ptr += 2) {
                    pixmap[(y * TS_MAP_SIZE / 2 + x) * 3] = (*ptr) >> 8;
                    pixmap[(y * TS_MAP_SIZE / 2 + x) * 3 + 1] = (*ptr) >> 8;
                    pixmap[(y * TS_MAP_SIZE / 2 + x) * 3 + 2] = (*ptr) >> 8;
                }
            }
        }
        position = state->position;
        position.x += state->laser_params.offset * cos(position.theta * M_PI / 180);
        position.y += state->laser_params.offset * sin(position.theta * M_PI / 180);
        ts_draw_scan_RGB(&state->scan, state->map, &position, pixmap, 2, 1);

        // SDL code :
        image = SDL_DisplayFormat(pixmap_surface);
        rotate = rotozoomSurface(image, -state->position.theta + 90, ZOOM, 0); 
        w = -(rotate->w / 2 - DISPLAY_SIZE / 2);
        rect.x = (int)floor(w - xrp + 0.5);
        rect.y = (int)floor(w + yrp + 0.5);
        SDL_BlitSurface(rotate, NULL, screen, &rect);
        rect.x = DISPLAY_SIZE / 2 - arrow_up->w / 2;
        rect.y = DISPLAY_SIZE / 2 - arrow_up->h / 2;
        SDL_BlitSurface(arrow_up, NULL, screen, &rect);
        SDL_FreeSurface(rotate);

        SDL_Flip(screen);
        SDL_FreeSurface(image);

        SDL_UnlockMutex(drawing_mutex);
    } while (!state->done);
    return 0;
}
#endif

int main(int argc, char *argv[])
{
    ts_position_t startpos, loop_startpos, loop_endpos, position;
    char filename[256], test_filename[256];
    int i, x, y;
    int quality;
    int nbscans, maxscans;
    ts_robot_parameters_t params;
    ts_laser_parameters_t laser_params;
    ts_state_t state;
    int loop, loop_start, nb_loops, loop_end[2];
#ifdef USE_SDL
    SDL_Surface *temp;
    SDL_Thread *th;
    SDL_Event event;
#endif

    if (argc < 2) 
        strcpy(test_filename, TEST_FILENAME);
    else
        strcpy(test_filename, argv[1]);

    // Read the scans from data file
    sprintf(filename, "%s.dat", test_filename);
    printf("Loading data from file %s...\n", test_filename);
    maxscans = ts_read_scans(filename, sensor_data);

    if (argc == 3) {
        nb_loops = 2;
        loop_end[0] = atoi(argv[2]);
    } else
        nb_loops = 1;
    loop_end[nb_loops - 1] = maxscans;

    params.r = 0.077;
    params.R = 0.165;
    params.inc = 2000;
    params.ratio = 1.0;
    
    laser_params.offset = 145;
    laser_params.scan_size = TEST_SCAN_SIZE;
    laser_params.angle_min = -120;
    laser_params.angle_max = +120;
    laser_params.detection_margin = 70;
    laser_params.distance_no_detection = 4000;

    startpos.x = 0.5 * TS_MAP_SIZE / TS_MAP_SCALE;
    startpos.y = 0.6 * TS_MAP_SIZE / TS_MAP_SCALE; 
    startpos.theta = 0;

    ts_map_init(&map);
    ts_map_init(&trajectory);
    ts_map_init(&loop_map);

#ifdef USE_SDL
    // SDL initialization code
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        printf("Unable to initialize SDL: %s\n", SDL_GetError());
	return 1;
    }
    atexit(SDL_Quit);
    drawing_mutex = SDL_CreateMutex();
    draw_it = SDL_CreateCond();
    th = SDL_CreateThread(drawing_thread, &state);

    screen = SDL_SetVideoMode(DISPLAY_SIZE, DISPLAY_SIZE, 24, SDL_DOUBLEBUF);
    if (screen == NULL) {
        printf("Unable to set video mode: %s\n", SDL_GetError());
        return 1;
    } 
    pixmap_surface = SDL_CreateRGBSurfaceFrom(pixmap, TS_MAP_SIZE / 2, TS_MAP_SIZE / 2, 24, TS_MAP_SIZE / 2 * 3, 0, 0, 0, 0); 
    for (i = 0; i != TS_MAP_SIZE / 2 * TS_MAP_SIZE / 2 * 3; i++) pixmap[i] = 0;
    temp = SDL_LoadBMP("arrow_up.bmp");
    if (temp == NULL) {
        printf("Unable to load bitmap: %s\n", SDL_GetError());
        return 1;
    }

    arrow_up = SDL_DisplayFormat(temp);
    SDL_FreeSurface(temp);
    SDL_SetColorKey(arrow_up, SDL_SRCCOLORKEY, 0);
#endif

    loop_start = 0;
    loop_startpos = startpos;
    for (loop = 0; loop != nb_loops; loop++) { 

        ts_state_init(&state, &map, &params, &laser_params, &loop_startpos, 100, 20, 600, TS_DIRECTION_FORWARD);
        for (nbscans = loop_start; nbscans != loop_end[loop]; nbscans++) {
            ts_iterative_map_building(&sensor_data[nbscans], &state);

#ifdef USE_SDL
            SDL_CondSignal(draw_it);
            while (SDL_PollEvent(&event));
#endif
            printf("#%d : %lg %lg %lg\n", nbscans, state.position.x, state.position.y, state.position.theta);

            if (nbscans == 50) 
                loop_map = map;

            x = (int)floor(state.position.x * TS_MAP_SCALE + 0.5);
            y = ((int)floor(state.position.y * TS_MAP_SCALE + 0.5));
            if (x >= 0 && y >= 0 && x < TS_MAP_SIZE && y < TS_MAP_SIZE)
                trajectory.map[y * TS_MAP_SIZE + x] = 0;
        }
        printf("Forward distance: %lg\n", state.distance);
        printf("Starting point : %lg %lg %lg\n", startpos.x, startpos.y, startpos.theta);
        printf("End point : %lg %lg %lg\n", state.position.x, state.position.y, state.position.theta);

        // Record the map
        sprintf(filename, "%s_loop%d_forward.pgm", test_filename, loop);
        ts_save_map_pgm(&map, &trajectory, filename, TS_MAP_SIZE, TS_MAP_SIZE);

        printf("Try to close the loop...\n");
        
        position = state.position;
        position.x += state.laser_params.offset * cos(position.theta * M_PI / 180);
        position.y += state.laser_params.offset * sin(position.theta * M_PI / 180);
        loop_endpos = ts_close_loop_position(&state, &sensor_data[loop_end[loop] - 1], &loop_map, &position, &quality);
        loop_endpos.x -= state.laser_params.offset * cos(loop_endpos.theta * M_PI / 180);
        loop_endpos.y -= state.laser_params.offset * sin(loop_endpos.theta * M_PI / 180);

        printf("Loop close point : %lg %lg %lg (%d)\n", loop_endpos.x, loop_endpos.y, loop_endpos.theta, quality);
        printf("Deviation is : %lg mm, %lg degrees\n", ts_distance(&state.position, &loop_endpos), fabs(state.position.theta - loop_endpos.theta)); 

        map = loop_map;
        ts_map_init(&trajectory);
#ifdef USE_SDL
        for (i = 0; i != TS_MAP_SIZE / 2 * TS_MAP_SIZE / 2 * 3; i++) pixmap[i] = 0;
#endif
        ts_state_init(&state, &map, &params, &laser_params, &loop_endpos, 100, 20, 600, TS_DIRECTION_BACKWARD);
        for (nbscans = loop_end[loop] - 1; nbscans >= loop_start; nbscans--) {
            ts_iterative_map_building(&sensor_data[nbscans], &state);        
#ifdef USE_SDL
            SDL_CondSignal(draw_it);
#ifdef WIN32
            while (SDL_PollEvent(&event));
#endif
#endif
            printf("#%d : %lg %lg %lg\n", nbscans, state.position.x, state.position.y, state.position.theta);

            x = (int)floor(state.position.x * TS_MAP_SCALE + 0.5);
            y = ((int)floor(state.position.y * TS_MAP_SCALE + 0.5));
            if (x >= 0 && y >= 0 && x < TS_MAP_SIZE && y < TS_MAP_SIZE)
                trajectory.map[y * TS_MAP_SIZE + x] = 0;
        }
        printf("Backward distance: %lg\n", state.distance);
        printf("Deviation is : %lg mm, %lg degrees\n", ts_distance(&state.position, &startpos), fabs(state.position.theta - startpos.theta)); 
        // Record the map
        sprintf(filename, "%s_loop%d_backward.pgm", test_filename, loop);
        ts_save_map_pgm(&map, &trajectory, filename, TS_MAP_SIZE, TS_MAP_SIZE);

        printf("Closing the loop...\n"); 
        ts_close_loop_trajectory(&sensor_data[loop_start], loop_end[loop] - loop_start, &loop_startpos, &loop_endpos);
        ts_draw_map(&map, &map_scans, sensor_data, &state, loop_end[loop], TS_FINAL_MAP);
        loop_map = map;
        sprintf(filename, "%s_loop%d_final.pgm", test_filename, loop);
        ts_save_map_pgm(&map, &trajectory, filename, TS_MAP_SIZE, TS_MAP_SIZE);
        sprintf(filename, "%s_loop%d_scans.pgm", test_filename, loop);
        ts_save_map_pgm(&map_scans, &trajectory, filename, TS_MAP_SIZE, TS_MAP_SIZE);
    
        loop_startpos = loop_endpos;
        loop_start = loop_end[loop];
    }

    state.done = 1;
#ifdef USE_SDL
    SDL_CondSignal(draw_it);
    SDL_WaitThread(th, NULL);
    SDL_FreeSurface(pixmap_surface);
    SDL_FreeSurface(arrow_up);
#endif
    printf("Done\n"); 
    return 1;
}
