namespace pd_controller {

struct PD_Params {
  float Kp;
  float Kd;
  float setpoint;
  float previous_error;
  float dt;  // delta time in seconds
  unsigned long last_time;
};

// PD untuk posisi berdasarkan TOF kiri
PD_Params tof_left_params = {
  .Kp = 0.8,    // Sesuaikan nilai ini
  .Kd = 0.2,    // Sesuaikan nilai ini
  .setpoint = 550,  // Target jarak TOF kiri
  .previous_error = 0,
  .dt = 0.02,   // 20ms
  .last_time = 0
};

// PD untuk posisi korban berdasarkan kamera
PD_Params camera_x_params = {
  .Kp = 0.5,    // Sesuaikan nilai ini
  .Kd = 0.15,   // Sesuaikan nilai ini
  .setpoint = cam::center,  // Menggunakan center dari cam.h
  .previous_error = 0,
  .dt = 0.02,   // 20ms
  .last_time = 0
};

float calculate_pd(PD_Params& params, float current_value) {
  unsigned long current_time = millis();
  
  // Hitung delta time yang sebenarnya
  if (params.last_time == 0) {
    params.last_time = current_time;
    return 0;
  }
  
  params.dt = (current_time - params.last_time) / 1000.0f; // Convert to seconds
  
  float error = params.setpoint - current_value;
  float derivative = (error - params.previous_error) / params.dt;
  
  float output = (params.Kp * error) + (params.Kd * derivative);
  
  params.previous_error = error;
  params.last_time = current_time;
  
  return output;
}

// Fungsi untuk mengatur ulang parameter PD
void reset_pd(PD_Params& params) {
  params.previous_error = 0;
  params.last_time = 0;
}

// Fungsi untuk mengubah setpoint
void set_setpoint(PD_Params& params, float new_setpoint) {
  params.setpoint = new_setpoint;
}

} // namespace pd_controller
