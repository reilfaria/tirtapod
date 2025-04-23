#ifndef PD_CONTROLLER_H
#define PD_CONTROLLER_H

#include <Arduino.h>
#include "compass.h"
#include "cam.h"
#include "TOF.h"

namespace pd_controller {

struct PD_Params {
    float Kp;
    float Kd;
    float setpoint;
    float previous_error;
    float dt;
    unsigned long last_time;
};

// Struktur untuk parameter TOF per ruangan
struct RoomTOFParams {
    PD_Params left;
    PD_Params right;
    PD_Params front;
};

// Struktur untuk target heading tiap ruangan
struct RoomHeadingTargets {
    int home;
    int room[10];  // Array untuk 9 ruangan + 1 (index 0 tidak dipakai)
};

// Target heading untuk arena kanan
RoomHeadingTargets right_arena = {
    .home = 166,
    .room = {0, 0, 178, 270, 187, 186, 100, 102, 94, 88}
};

// Target heading untuk arena kiri
RoomHeadingTargets left_arena = {
    .home = 203,
    .room = {0, 0, 201, 133, 220, 223, 302, 302, 295, 305}
};

// Parameter untuk ruangan 1
RoomTOFParams room1_params = {
    .left = {
        .Kp = 0.8,
        .Kd = 0.2,
        .setpoint = 550,  // Jarak optimal untuk deteksi korban
        .previous_error = 0,
        .dt = 0.02,
        .last_time = 0
    },
    .right = {
        .Kp = 0.8,
        .Kd = 0.2,
        .setpoint = 0,    // Tidak digunakan di R1
        .previous_error = 0,
        .dt = 0.02,
        .last_time = 0
    },
    .front = {
        .Kp = 0.8,
        .Kd = 0.2,
        .setpoint = 0,    // Tidak digunakan di R1
        .previous_error = 0,
        .dt = 0.02,
        .last_time = 0
    }
};

// Parameter untuk ruangan 2
RoomTOFParams room2_params = {
    .left = {
        .Kp = 0.7,
        .Kd = 0.15,
        .setpoint = 200,  // Sesuaikan dengan kebutuhan R2
        .previous_error = 0,
        .dt = 0.02,
        .last_time = 0
    },
    .right = {
        .Kp = 0.7,
        .Kd = 0.15,
        .setpoint = 700,  // Wall following di R2
        .previous_error = 0,
        .dt = 0.02,
        .last_time = 0
    },
    .front = {
        .Kp = 0.7,
        .Kd = 0.15,
        .setpoint = 190,  // Jarak aman ke dinding depan
        .previous_error = 0,
        .dt = 0.02,
        .last_time = 0
    }
};

// Parameter untuk kamera
PD_Params camera_params = {
    .Kp = 0.5,
    .Kd = 0.15,
    .setpoint = cam::center,
    .previous_error = 0,
    .dt = 0.02,
    .last_time = 0
};

// Parameter untuk kompas
PD_Params compass_params = {
    .Kp = 1.2,
    .Kd = 0.3,
    .setpoint = 166,  // Default ke home position arena kanan
    .previous_error = 0,
    .dt = 0.02,
    .last_time = 0
};

// Fungsi untuk mendapatkan parameter berdasarkan ruangan
RoomTOFParams& get_room_params(int room_number) {
    switch(room_number) {
        case 1:
            return room1_params;
        case 2:
            return room2_params;
        // Tambahkan case untuk ruangan lain
        default:
            return room1_params;
    }
}

float calculate_pd(PD_Params& params, float current_value) {
    unsigned long current_time = millis();
    
    if (params.last_time == 0) {
        params.last_time = current_time;
        return 0;
    }
    
    params.dt = (current_time - params.last_time) / 1000.0f;
    float error = params.setpoint - current_value;
    float derivative = (error - params.previous_error) / params.dt;
    float output = (params.Kp * error) + (params.Kd * derivative);
    
    params.previous_error = error;
    params.last_time = current_time;
    
    return output;
}

float calculate_angle_error(float target, float current) {
    float error = target - current;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    return error;
}

float calculate_compass_pd(float target_heading) {
    float current_heading = compass::heading();
    compass_params.setpoint = target_heading;
    
    float error = calculate_angle_error(target_heading, current_heading);
    unsigned long current_time = millis();
    
    if (compass_params.last_time == 0) {
        compass_params.last_time = current_time;
        compass_params.previous_error = error;
        return 0;
    }
    
    compass_params.dt = (current_time - compass_params.last_time) / 1000.0f;
    float derivative = (error - compass_params.previous_error) / compass_params.dt;
    float output = (compass_params.Kp * error) + (compass_params.Kd * derivative);
    
    compass_params.previous_error = error;
    compass_params.last_time = current_time;
    
    return output;
}

void update_compass_setpoint(int room_number, bool is_right_arena) {
    RoomHeadingTargets& targets = is_right_arena ? right_arena : left_arena;
    
    if (room_number < 0 || room_number > 9) {
        Serial.println("Invalid room number");
        return;
    }
    
    float new_setpoint;
    if (room_number == 0) {
        new_setpoint = targets.home;
    } else {
        new_setpoint = targets.room[room_number];
    }
    
    // Validasi heading
    if (new_setpoint >= 0 && new_setpoint < 360) {
        compass_params.setpoint = new_setpoint;
        Serial.println("Updated compass setpoint to: " + String(new_setpoint));
    } else {
        Serial.println("Invalid heading value");
    }
}

void reset_pd(PD_Params& params) {
    params.previous_error = 0;
    params.last_time = 0;
}

void reset_room_params(RoomTOFParams& params) {
    reset_pd(params.left);
    reset_pd(params.right);
    reset_pd(params.front);
}

} // namespace pd_controller

#endif
