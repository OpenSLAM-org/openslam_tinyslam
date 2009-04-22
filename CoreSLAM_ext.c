#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "CoreSLAM.h"

double
ts_distance(ts_position_t *pos1, ts_position_t *pos2)
{
    return sqrt((pos1->x - pos2->x) * (pos1->x - pos2->x) + (pos1->y - pos2->y) * (pos1->y - pos2->y));
}

void
ts_save_map_pgm(ts_map_t *map, ts_map_t *overlay, char *filename, int width, int height)
{
    int x, y, xp, yp;
    FILE *output;
    output = fopen(filename, "wt");
    fprintf(output, "P2\n%d %d 255\n", width, height);
    y = (TS_MAP_SIZE - height) / 2;
    for (yp = 0; yp < height; y++, yp++) {
        x = (TS_MAP_SIZE - width) / 2; 
	for (xp = 0; xp < width; x++, xp++) {
	    if (overlay->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x] == 0) 
                fprintf(output, "0 ");
            else 
                fprintf(output, "%d ", (int)(map->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 8);
	}
	fprintf(output, "\n");
    }
    fclose(output);
}

void
ts_draw_scan(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos)
{
    double c, s;
    double x2p, y2p;
    int i, x1, y1, x2, y2;

    c = cos(pos->theta * M_PI / 180);
    s = sin(pos->theta * M_PI / 180);
    x1 = (int)floor(pos->x * TS_MAP_SCALE + 0.5);
    y1 = (int)floor(pos->y * TS_MAP_SCALE + 0.5);
    // Translate and rotate scan to robot position
    for (i = 0; i != scan->nb_points; i++) {
        if (scan->value[i] != TS_NO_OBSTACLE) {
            x2p = c * scan->x[i] - s * scan->y[i];
            y2p = s * scan->x[i] + c * scan->y[i];
            x2p *= TS_MAP_SCALE;
            y2p *= TS_MAP_SCALE; 
            x2 = (int)floor(pos->x * TS_MAP_SCALE + x2p + 0.5);
            y2 = (int)floor(pos->y * TS_MAP_SCALE + y2p + 0.5);
            if (x2 >= 0 && y2 >= 0 && x2 < TS_MAP_SIZE && y2 < TS_MAP_SIZE)
                map->map[y2 * TS_MAP_SIZE + x2] = 0;
        }
    }
}

void
ts_draw_scan_RGB(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos, unsigned char *pixmap, int scale, int reversey)
{
    double c, s;
    double x2p, y2p;
    int i, x1, y1, x2, y2;
    int color;
    unsigned char *ptr;

    c = cos(pos->theta * M_PI / 180);
    s = sin(pos->theta * M_PI / 180);
    x1 = (int)floor(pos->x * TS_MAP_SCALE + 0.5);
    y1 = (int)floor(pos->y * TS_MAP_SCALE + 0.5);
    // Translate and rotate scan to robot position
    for (i = 0; i < scan->nb_points; i++) {
        if (scan->value[i] != TS_NO_OBSTACLE) {
            x2p = c * scan->x[i] - s * scan->y[i];
            y2p = s * scan->x[i] + c * scan->y[i];
            x2p *= TS_MAP_SCALE;
            y2p *= TS_MAP_SCALE; 
            x2 = (int)floor(pos->x * TS_MAP_SCALE + x2p + 0.5);
            y2 = (int)floor(pos->y * TS_MAP_SCALE + y2p + 0.5);
            if (x2 >= 0 && y2 >= 0 && x2 < TS_MAP_SIZE && y2 < TS_MAP_SIZE) {
                color = map->map[y2 * TS_MAP_SIZE + x2] >> 8;
                if (reversey)
                    ptr = &pixmap[((TS_MAP_SIZE - 1 - y2) / scale * TS_MAP_SIZE / scale + x2 / scale) * 3];
                else
                    ptr = &pixmap[(y2 / scale * TS_MAP_SIZE / scale  + x2 / scale) * 3];
                ptr[0] = 255 - color;
                ptr[1] = 0;
                ptr[2] = color;
            }
        }
    }
}


