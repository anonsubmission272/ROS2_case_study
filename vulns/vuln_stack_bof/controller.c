#include "../controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))

InStruct *in;
OutStruct *out;
MappedJointTrajectoryPoint *point_interp;

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

    // stack buffer overflow
    if (traj_msg.points[ind].accelerations_length > 0) {
        char processing_buffer[16];  // smaller buffer
        memset(processing_buffer, 0, sizeof(processing_buffer));
        
        printf("Processing %zu acceleration values in trajectory interpolation\n", 
               traj_msg.points[ind].accelerations_length);
        
        // copy acceleration data
        for (size_t i = 0; i < traj_msg.points[ind].accelerations_length; i++) {
            char byte_val = (char)((int)traj_msg.points[ind].accelerations[i] & 0xFF);
            processing_buffer[i] = byte_val;
            
        }
        
        printf("Processed trajectory buffer: %.15s\n", processing_buffer);
        
        if (processing_buffer[0] != 0) {
            double adjustment = processing_buffer[0] * 0.001;
            point_interp->positions[0] += adjustment;
            
        }
        
    }

}

int init() {
    printf("initializing controller...\n");
    in = malloc(sizeof(InStruct));
    out = malloc(sizeof(OutStruct));
    point_interp = malloc(sizeof(MappedJointTrajectoryPoint));
    return 0;

}

int step() {
    printf("Inside Controller: %f\n", in->value.points[1].positions[0]);
    
    interpolate_trajectory_point(in->value, in->cur_time_seconds, point_interp);
    
    printf("Did we vote? %f\n", point_interp->positions[0]);
    
    out->vote = *point_interp;
    return 0;

}
