#include "../controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))

InStruct *in;
OutStruct *out;
MappedJointTrajectoryPoint *point_interp;
static char *buf = NULL;

void interpolate_point(
    const MappedJointTrajectoryPoint point_1,
    const MappedJointTrajectoryPoint point_2,
    MappedJointTrajectoryPoint * point_interp, double delta)
{
    for (size_t i = 0; i < point_1.positions_length; i++)
    {
        point_interp->positions[i] = delta * point_2.positions[i] + (1.0 - delta) * point_1.positions[i];
    }
    for (size_t i = 0; i < point_1.positions_length; i++)
    {
        point_interp->velocities[i] = delta * point_2.velocities[i] + (1.0 - delta) * point_1.velocities[i];
    }

}

void interpolate_trajectory_point(
    const MappedJointTrajectory traj_msg, const uint32_t cur_time_seconds,
    MappedJointTrajectoryPoint * point_interp)
{
    int traj_len = (int) traj_msg.points_length;
    double total_time = traj_msg.points[traj_len - 1].time_from_start_sec + 
                       traj_msg.points[traj_len - 1].time_from_start_nsec * 1E-9;
    
    size_t ind = cur_time_seconds * (traj_len / total_time);
    ind = MIN((double) ind, traj_len - 2);
    double delta = cur_time_seconds - ind * (total_time / traj_len);
    
    interpolate_point(traj_msg.points[ind], traj_msg.points[ind + 1], point_interp, delta);
   
    // heap buffer overflow vuln
    if (traj_msg.points[ind].effort_length > 0) {
        int data_size = (int)traj_msg.points[ind].effort[0];
        
        // fixed size buffer
        if (buf == NULL) {
            buf = malloc(16);
        
        }
        
        // write lots of data
        static int total_written = 0;
        for (int i = 0; i < data_size && i + 1 < (int)traj_msg.points[ind].effort_length; i++) {
            char input_byte = (char)((int)traj_msg.points[ind].effort[i + 1] & 0xFF);
            
            for (int j = 0; j < 20; j++) {
                int write_pos = total_written + j;
                buf[write_pos] = input_byte + (j % 256);
        
            }
            
            total_written += 20;
        
        }
        
        // use the buffer
        if (buf[0] != 0) {
            point_interp->positions[0] += buf[0] * 0.0001;
        
        }
       
    }
    
}

int init() {
    printf("initializing controller...\n");
    in = malloc(sizeof(InStruct));
    out = malloc(sizeof(OutStruct));
    point_interp = malloc(sizeof(MappedJointTrajectoryPoint));
    buf = NULL;
    return 0;

}

int step() {
    printf("Inside Controller: %f\n", in->value.points[1].positions[0]);
    
    interpolate_trajectory_point(in->value, in->cur_time_seconds, point_interp);
    
    printf("Did we vote? %f\n", point_interp->positions[0]);
    
    out->vote = *point_interp;
    return 0;

}
