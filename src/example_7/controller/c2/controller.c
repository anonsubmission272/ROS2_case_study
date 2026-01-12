#include "../controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Completely different data organization - use global arrays instead of structs
static double cached_positions[100];
static double cached_velocities[100];
static double workspace_buffer_a[100];
static double workspace_buffer_b[100];
static size_t cached_array_sizes[4]; // positions_len, velocities_len, accel_len, effort_len

InStruct controller_input_data;
OutStruct controller_output_data;

InStruct *in;
OutStruct *out;

// Different approach: iterative blending instead of direct formula
void blend_trajectory_data_iteratively(
    double* source_data_1, double* source_data_2, double* target_data,
    size_t data_count, double blend_weight)
{
    // Use iterative accumulation instead of direct multiplication
    for (size_t idx = 0; idx < data_count; idx++) {
        double accumulated_value = 0.0;
        
        // Accumulate first source with (1-weight)
        double remaining_weight = 1.0 - blend_weight;
        double temp_val = source_data_1[idx];
        accumulated_value += temp_val * remaining_weight;
        
        // Accumulate second source with weight  
        temp_val = source_data_2[idx];
        accumulated_value += temp_val * blend_weight;
        
        target_data[idx] = accumulated_value;
    }
}

// Different approach: matrix-style point processing
void process_trajectory_points_matrix_style(
    const MappedJointTrajectory* trajectory_reference, 
    double current_timestamp,
    double* output_positions, double* output_velocities, size_t* output_sizes)
{
    // Create local working matrices
    double point_matrix_a[100];
    double point_matrix_b[100];
    double velocity_matrix_a[100]; 
    double velocity_matrix_b[100];
    
    int trajectory_point_count = (int)trajectory_reference->points_length;
    
    // Calculate end timestamp using different variable naming
    const MappedJointTrajectoryPoint* final_point = &trajectory_reference->points[trajectory_point_count - 1];
    double mission_duration = final_point->time_from_start_sec + final_point->time_from_start_nsec * 1E-9;
    
    // Use different index calculation approach but same result
    double time_ratio = current_timestamp / mission_duration;
    double index_float = time_ratio * trajectory_point_count;
    size_t base_index = (size_t)index_float;
    
    // Apply same bounds logic but with different variable names
    size_t clamped_index = base_index;
    if (clamped_index >= trajectory_point_count - 1) {
        clamped_index = trajectory_point_count - 2;
    }
    
    // Calculate interpolation weight using different method but same result
    double segment_duration = mission_duration / trajectory_point_count;
    double segment_start_time = clamped_index * segment_duration;
    double interpolation_weight = current_timestamp - segment_start_time;
    
    // Copy data to working matrices
    const MappedJointTrajectoryPoint* point_A = &trajectory_reference->points[clamped_index];
    const MappedJointTrajectoryPoint* point_B = &trajectory_reference->points[clamped_index + 1];
    
    memcpy(point_matrix_a, point_A->positions, point_A->positions_length * sizeof(double));
    memcpy(point_matrix_b, point_B->positions, point_B->positions_length * sizeof(double));
    memcpy(velocity_matrix_a, point_A->velocities, point_A->velocities_length * sizeof(double));  
    memcpy(velocity_matrix_b, point_B->velocities, point_B->velocities_length * sizeof(double));
    
    // Store sizes
    output_sizes[0] = point_A->positions_length;
    output_sizes[1] = point_A->velocities_length;
    output_sizes[2] = point_A->accelerations_length;
    output_sizes[3] = point_A->effort_length;
    
    // Perform blending using iterative method
    blend_trajectory_data_iteratively(point_matrix_a, point_matrix_b, output_positions, 
                                     point_A->positions_length, interpolation_weight);
    blend_trajectory_data_iteratively(velocity_matrix_a, velocity_matrix_b, output_velocities,
                                     point_A->velocities_length, interpolation_weight);
}

int init() {
    printf("initializing maximally diversified variant controller...\n");
    
    // Use stack allocation with different organization
    in = &controller_input_data;
    out = &controller_output_data;
    
    // Initialize working arrays
    memset(cached_positions, 0, sizeof(cached_positions));
    memset(cached_velocities, 0, sizeof(cached_velocities));
    memset(workspace_buffer_a, 0, sizeof(workspace_buffer_a));
    memset(workspace_buffer_b, 0, sizeof(workspace_buffer_b));
    memset(cached_array_sizes, 0, sizeof(cached_array_sizes));
    
    return 0;
}

int step() {
    printf("Inside Maximally Diversified Controller: %f\n", in->value.points[1].positions[0]);
    
    // Use completely different function interface but same core logic
    process_trajectory_points_matrix_style(&in->value, (double)in->cur_time_seconds, 
                                          cached_positions, cached_velocities, cached_array_sizes);
    
    printf("Diversified controller vote: %f\n", cached_positions[0]);
    
    // Transfer data from arrays back to struct format
    out->vote.positions_length = cached_array_sizes[0];
    out->vote.velocities_length = cached_array_sizes[1]; 
    out->vote.accelerations_length = cached_array_sizes[2];
    out->vote.effort_length = cached_array_sizes[3];
    
    memcpy(out->vote.positions, cached_positions, cached_array_sizes[0] * sizeof(double));
    memcpy(out->vote.velocities, cached_velocities, cached_array_sizes[1] * sizeof(double));
    
    return 0;
}