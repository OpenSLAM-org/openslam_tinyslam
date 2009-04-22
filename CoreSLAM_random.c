#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "CoreSLAM.h"

static unsigned long SHR3(ts_randomizer_t *d)
{ d->jz=d->jsr; d->jsr^=(d->jsr<<13); d->jsr^=(d->jsr>>17); d->jsr^=(d->jsr<<5); return d->jz+d->jsr;}

static double UNI(ts_randomizer_t *d) 
{ return .5 + (signed)SHR3(d) * .2328306e-9;}

double ts_random_normal_fix(ts_randomizer_t *d) 
{	
    const double r = 3.442620; 	// The starting of the right tail 	
    static double x, y;
    for(;;)
    {
        x=d->hz*d->wn[d->iz];
        if(d->iz==0)
        { // iz==0, handle the base strip
            do
            {	
                x=-log(UNI(d))*0.2904764;   
                // .2904764 is 1/r				
                y=-log(UNI(d));			
            } while(y+y<x*x);
            return (d->hz>0)? r+x : -r-x;	
        }
        
        // iz>0, handle the wedges of other strips		
        if( d->fn[d->iz]+UNI(d)*(d->fn[d->iz-1]-d->fn[d->iz]) < exp(-.5*x*x) ) 
            return x;
        // Start all over		
        d->hz=SHR3(d);		
        d->iz=d->hz&127;		
        if((unsigned long)abs(d->hz)<d->kn[d->iz]) 
            return (d->hz*d->wn[d->iz]);	
    }   
}

double ts_random_normal(ts_randomizer_t *d, double m, double s) 
{ 
    double x;
    d->hz = SHR3(d); 
    d->iz = d->hz & 127;
    x= ((unsigned long)abs(d->hz) < d->kn[d->iz])? d->hz * d->wn[d->iz] : ts_random_normal_fix(d); // Generic version
    return x * s + m ;
};

void ts_random_init(ts_randomizer_t *d, unsigned long jsrseed) 
{	  
    const double m1 = 2147483648.0;
    
    double dn=3.442619855899, tn=dn, vn=9.91256303526217e-3, q;      
    int i;		  
    d->jsr=jsrseed;
    
    // Set up tables for Normal	  
    q=vn/exp(-.5*dn*dn);  
    d->kn[0]=(int)((dn/q)*m1);	  
    d->kn[1]=0;		  
    d->wn[0]=q/m1; d->wnt[0]=q;
    d->wn[127]=dn/m1; d->wnt[127]=dn;
    d->fn[0]=1.;	  
    d->fn[127]=exp(-.5*dn*dn);		
    for(i=126;i>=1;i--) {   
        dn=sqrt(-2.*log(vn/dn+exp(-.5*dn*dn)));          
        d->kn[i+1]=(int)((dn/tn)*m1);		  tn=dn;          
        d->fn[i]=exp(-.5*dn*dn);          
        d->wn[i]=dn/m1;     d->wnt[i]=dn; 
    }
}

double ts_random(ts_randomizer_t *d)
{
    return UNI(d);
}

long ts_random_int(ts_randomizer_t *d, long min, long max)
{
    // Output random integer in the interval min <= x <= max
    long r;
    r = (long)((max - min + 1) * ts_random(d)) + min; // Multiply interval with random and truncate
    if (r > max) r = max;
    if (max < min) return 0x80000000;
    return r;
}

ts_position_t ts_monte_carlo_search(ts_randomizer_t *randomizer, ts_scan_t *scan, ts_map_t *map, ts_position_t *start_pos, double sigma_xy, double sigma_theta, int stop, int *bd)
{
    ts_position_t currentpos, bestpos, lastbestpos;
    int currentdist;
    int bestdist, lastbestdist;
    int counter = 0, debug = 0;

    if (stop < 0) {
        debug = 1;
        stop = -stop;
    }
    currentpos = bestpos = lastbestpos = *start_pos;
    currentdist = ts_distance_scan_to_map(scan, map, &currentpos);
    bestdist = lastbestdist = currentdist;

    do {
	currentpos = lastbestpos;
	currentpos.x = ts_random_normal(randomizer, currentpos.x, sigma_xy);
	currentpos.y = ts_random_normal(randomizer, currentpos.y, sigma_xy);
	currentpos.theta = ts_random_normal(randomizer, currentpos.theta, sigma_theta);

	currentdist = ts_distance_scan_to_map(scan, map, &currentpos);
	
	if (currentdist < bestdist) {
	    bestdist = currentdist;
	    bestpos = currentpos;
            if (debug) printf("Monte carlo ! %lg %lg %lg %d (count = %d)\n", bestpos.x, bestpos.y, bestpos.theta, bestdist, counter);
	} else {
	    counter++;
	}
        if (counter > stop / 3) {
            if (bestdist < lastbestdist) {
                lastbestpos = bestpos;
                lastbestdist = bestdist;
                counter = 0;
                sigma_xy *= 0.5;
                sigma_theta *= 0.5;
            }
        }
    } while (counter < stop);
    if (bd)
        *bd = bestdist;
    return bestpos;
}


