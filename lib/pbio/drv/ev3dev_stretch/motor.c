
#include <stdbool.h>
#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <pbdrv/motor.h>


#define MAX_PATH_LENGTH 50

#define PORT_TO_IDX(p) ((p) - PBDRV_CONFIG_FIRST_MOTOR_PORT)

// Motor file structure for each motor
typedef struct _motor_file_t {
    bool connected;
    bool coasting;
    int dir_number;
    FILE *f_encoder_count;
    FILE *f_encoder_rate;
    FILE *f_duty;
} motor_file_t;

motor_file_t motor_files[] = {
    [PORT_TO_IDX(PBDRV_CONFIG_FIRST_MOTOR_PORT) ... PORT_TO_IDX(PBDRV_CONFIG_LAST_MOTOR_PORT)]{
        .connected=false
    }
};


// Open file, write contents, and close it
pbio_error_t slow_write(pbio_port_t port, const char* filename, const char* content) {
    if (port < PBDRV_CONFIG_FIRST_MOTOR_PORT || port > PBDRV_CONFIG_LAST_MOTOR_PORT) {
        return PBIO_ERROR_INVALID_PORT;
    }
    if (!motor_files[PORT_TO_IDX(port)].connected) {
        return PBIO_ERROR_NO_DEV;
    }
    // Open the file in the directory corresponding to the specified port
    char filepath[MAX_PATH_LENGTH];
    snprintf(filepath, MAX_PATH_LENGTH, "/sys/class/tacho-motor/motor%d/%s", motor_files[PORT_TO_IDX(port)].dir_number, filename);
    FILE* file = fopen(filepath, "w"); 
    if (file != NULL) {
        // Write the contents to the file
        fprintf(file, "%s", content);  
        // Close the file
        fclose(file); 
        return PBIO_SUCCESS;
    }
    else{
        return PBIO_ERROR_IO;
    }
          
}

void _pbdrv_motor_init(void) {
    // Open tacho-motor directory
    DIR *dp;
    struct dirent *ep;
    dp = opendir ("/sys/class/tacho-motor");
    // Loop through the motorXs
    while ((ep = readdir(dp))) {
        // Ignore the . and .. folders
        if (strlen(ep->d_name) > 5) {
            // Obtain the numeric device ID from the path
            int dir_number, port_index;
            sscanf(ep->d_name, "%*5c%d",&dir_number);
            // Open the address file in this folder
            char ppath[MAX_PATH_LENGTH];
            snprintf(ppath, MAX_PATH_LENGTH, "/sys/class/tacho-motor/motor%d/address", dir_number);
            FILE* addr = fopen(ppath, "r");
            // Extract the port (14th) character, convert to numeric, and close address file
            fseek(addr, 13, SEEK_SET);
            port_index = fgetc(addr) - 'A';
            fclose(addr);
            // Store that this motor is present and save its location
            motor_files[port_index].connected = true;
            motor_files[port_index].dir_number = dir_number;
        }
    }
    // Close tacho-motor directory when done
    closedir (dp);

    // Now that we know which motors are present, open the relevant files for reading and writing
    for(pbio_port_t port = PBDRV_CONFIG_FIRST_MOTOR_PORT; port <= PBDRV_CONFIG_LAST_MOTOR_PORT; port++) {
        int port_index = PORT_TO_IDX(port);
        if (motor_files[port_index].connected) {
            //Debug message. Should replace with debug print
            printf("Detected motor%d on port %c.\n", motor_files[port_index].dir_number, port_index+65);
            // Reset the motor and let stop default to coast
            slow_write(port, "command", "reset");
            slow_write(port, "stop_action", "coast");
            pbdrv_motor_coast(port);
            // File path character array to the relevant speed, position files, etc.
            char filepath[MAX_PATH_LENGTH];
            // Open the position file
            snprintf(filepath, MAX_PATH_LENGTH, "/sys/class/tacho-motor/motor%d/position", motor_files[port_index].dir_number);
            motor_files[port_index].f_encoder_count = fopen(filepath, "r");
            // Open the speed file
            snprintf(filepath, MAX_PATH_LENGTH, "/sys/class/tacho-motor/motor%d/speed", motor_files[port_index].dir_number);
            motor_files[port_index].f_encoder_rate = fopen(filepath, "r");            
            // Open the duty file
            snprintf(filepath, MAX_PATH_LENGTH, "/sys/class/tacho-motor/motor%d/duty_cycle_sp", motor_files[port_index].dir_number);
            motor_files[port_index].f_duty = fopen(filepath, "w");
        }
    }    
}

#ifdef PBIO_CONFIG_ENABLE_DEINIT
void _pbdrv_motor_deinit(void) {
    // Close the relevant files
    for(pbio_port_t port = PBDRV_CONFIG_FIRST_MOTOR_PORT; port <= PBDRV_CONFIG_LAST_MOTOR_PORT; port++) {
        int port_index = PORT_TO_IDX(port);
        if (motor_files[port_index].connected) {
            // Only close files for motors that are attached
            fclose(motor_files[port_index].f_encoder_count);
            fclose(motor_files[port_index].f_encoder_rate);
            fclose(motor_files[port_index].f_duty);
            // Reset the motor
            slow_write(port, "command", "reset");
        }
    }
}
#endif

pbio_error_t pbdrv_motor_coast(pbio_port_t port) {
    if (port < PBDRV_CONFIG_FIRST_MOTOR_PORT || port > PBDRV_CONFIG_LAST_MOTOR_PORT) {
        return PBIO_ERROR_INVALID_PORT;
    }
    if (!motor_files[PORT_TO_IDX(port)].connected) {
        return PBIO_ERROR_NO_DEV;
    }
    motor_files[PORT_TO_IDX(port)].coasting = true;
    return slow_write(port, "command", "stop");
}

pbio_error_t pbdrv_motor_set_duty_cycle(pbio_port_t port, int16_t duty_cycle) {
    if (port < PBDRV_CONFIG_FIRST_MOTOR_PORT || port > PBDRV_CONFIG_LAST_MOTOR_PORT) {
        return PBIO_ERROR_INVALID_PORT;
    }
    if (!motor_files[PORT_TO_IDX(port)].connected) {
        return PBIO_ERROR_NO_DEV;
    }
    // If the motor is currently in coast mode, set it back to run-direct mode
    if (motor_files[PORT_TO_IDX(port)].coasting) {
        slow_write(port, "command", "run-direct");
        motor_files[PORT_TO_IDX(port)].coasting = false;
    }
    fseek(motor_files[PORT_TO_IDX(port)].f_duty, 0, SEEK_SET);
    fprintf(motor_files[PORT_TO_IDX(port)].f_duty, "%d", duty_cycle/100);
    int err = fflush(motor_files[PORT_TO_IDX(port)].f_duty); 
    if (err == EOF) {
        return PBIO_ERROR_IO;
    }
    else{
        return PBIO_SUCCESS;
    }
}

pbio_error_t pbdrv_motor_get_encoder_count(pbio_port_t port, int32_t *count) {
    if (port < PBDRV_CONFIG_FIRST_MOTOR_PORT || port > PBDRV_CONFIG_LAST_MOTOR_PORT) {
        return PBIO_ERROR_INVALID_PORT;
    }
    if (!motor_files[PORT_TO_IDX(port)].connected) {
        return PBIO_ERROR_NO_DEV;
    }
    
    fseek(motor_files[PORT_TO_IDX(port)].f_encoder_count, 0, SEEK_SET);
    int err = fscanf(motor_files[PORT_TO_IDX(port)].f_encoder_count, "%d", count);
    fflush(motor_files[PORT_TO_IDX(port)].f_encoder_count);
    if (err == EOF) {
        return PBIO_ERROR_IO;
    }
    else{
        return PBIO_SUCCESS;
    }
}

pbio_error_t pbdrv_motor_get_encoder_rate(pbio_port_t port, int32_t *rate) {
    if (port < PBDRV_CONFIG_FIRST_MOTOR_PORT || port > PBDRV_CONFIG_LAST_MOTOR_PORT) {
        return PBIO_ERROR_INVALID_PORT;
    }
    if (!motor_files[PORT_TO_IDX(port)].connected) {
        return PBIO_ERROR_NO_DEV;
    }
    fseek(motor_files[PORT_TO_IDX(port)].f_encoder_rate, 0, SEEK_SET);
    fscanf(motor_files[PORT_TO_IDX(port)].f_encoder_rate, "%d", rate);
    fflush(motor_files[PORT_TO_IDX(port)].f_encoder_rate);
    return PBIO_SUCCESS;    
}
