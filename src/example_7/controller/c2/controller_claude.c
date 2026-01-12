#include "../controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Global variables
InStruct *in;
OutStruct *out;
MappedJointTrajectoryPoint *interpolated_result;

// Helper function to find trajectory segment using binary search approach
int find_trajectory_segment(const MappedJointTrajectory *traj, double current_time_sec) {
    int points_count = (int)traj->points_length;
    
    // Handle edge cases
    if (points_count < 2) return 0;
    if (current_time_sec <= 0) return 0;
    
    // Convert current time to match trajectory point format
    double target_time = current_time_sec;
    
    // Find the appropriate segment by iterating through points
    for (int i = 0; i < points_count - 1; i++) {
        double point_time = traj->points[i].time_from_start_sec + 
                           traj->points[i].time_from_start_nsec * 1E-9;
        double next_point_time = traj->points[i + 1].time_from_start_sec + 
                                traj->points[i + 1].time_from_start_nsec * 1E-9;
        
        if (target_time >= point_time && target_time <= next_point_time) {
            return i;
        }
    }
    
    // If we're past the end, return the last valid segment
    return points_count - 2;
}

// Alternative interpolation implementation using component-wise approach
void perform_interpolation(const MappedJointTrajectoryPoint *p1, 
                          const MappedJointTrajectoryPoint *p2,
                          MappedJointTrajectoryPoint *result, 
                          double interpolation_factor) {
    
    // Set lengths
    result->positions_length = p1->positions_length;
    result->velocities_length = p1->velocities_length;
    
    // Interpolate positions using different calculation order
    for (size_t joint_idx = 0; joint_idx < p1->positions_length; joint_idx++) {
        double start_pos = p1->positions[joint_idx];
        double end_pos = p2->positions[joint_idx];
        double position_diff = end_pos - start_pos;
        result->positions[joint_idx] = start_pos + (interpolation_factor * position_diff);
    }
    
    // Interpolate velocities using different calculation order
    for (size_t joint_idx = 0; joint_idx < p1->velocities_length; joint_idx++) {
        double start_vel = p1->velocities[joint_idx];
        double end_vel = p2->velocities[joint_idx];
        double velocity_diff = end_vel - start_vel;
        result->velocities[joint_idx] = start_vel + (interpolation_factor * velocity_diff);
    }
}

// Main trajectory processing function with different approach
void process_trajectory_at_time(const MappedJointTrajectory *trajectory, 
                               double time_seconds,
                               MappedJointTrajectoryPoint *output_point) {
    
    int segment_index = find_trajectory_segment(trajectory, time_seconds);
    
    // Get the two points for interpolation
    const MappedJointTrajectoryPoint *point_before = &trajectory->points[segment_index];
    const MappedJointTrajectoryPoint *point_after = &trajectory->points[segment_index + 1];
    
    // Calculate time values for this segment
    double time_before = point_before->time_from_start_sec + 
                        point_before->time_from_start_nsec * 1E-9;
    double time_after = point_after->time_from_start_sec + 
                       point_after->time_from_start_nsec * 1E-9;
    
    // Calculate interpolation factor using different approach
    double segment_duration = time_after - time_before;
    double elapsed_in_segment = time_seconds - time_before;
    double interpolation_ratio = 0.0;
    
    if (segment_duration > 0.0) {
        interpolation_ratio = elapsed_in_segment / segment_duration;
        // Clamp to [0,1] range
        if (interpolation_ratio < 0.0) interpolation_ratio = 0.0;
        if (interpolation_ratio > 1.0) interpolation_ratio = 1.0;
    }
    
    // Perform the interpolation
    perform_interpolation(point_before, point_after, output_point, interpolation_ratio);
}

int init() {
    printf("initializing variant controller...\n");
    
    // Allocate memory for structures
    in = (InStruct *)malloc(sizeof(InStruct));
    out = (OutStruct *)malloc(sizeof(OutStruct));
    interpolated_result = (MappedJointTrajectoryPoint *)malloc(sizeof(MappedJointTrajectoryPoint));
    
    // Check allocation success
    if (!in || !out || !interpolated_result) {
        printf("Memory allocation failed!\n");
        return -1;
    }
    
    return 0;
}

int step() {
    printf("Variant Controller - Input time: %u\n", in->cur_time_seconds);
    printf("Variant Controller - First point position: %f\n", in->value.points[1].positions[0]);
    
    // Process the trajectory using our variant approach
    process_trajectory_at_time(&(in->value), (double)in->cur_time_seconds, interpolated_result);
    
    printf("Variant Controller - Interpolated result: %f\n", interpolated_result->positions[0]);
    
    // Copy result to output
    out->vote = *interpolated_result;
    
    return 0;
}