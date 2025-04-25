#ifndef HUMAN_PID_H
#define HUMAN_PID_H

namespace victim_pid {

class VictimPID {
private:
    // Parameter PID
    float Kp = 0.45;
    float Ki = 0.01;
    float Kd = 0.15;
    float setpoint = 330;
    
    // Variabel PID
    float lastError = 0;
    float integral = 0;
    unsigned long lastTime = 0;
    int stabilityCounter = 0;
    const int STABILITY_THRESHOLD = 10;

    float computePID(int currentPosition) {
        if(currentPosition == -1) return 0;
        
        unsigned long now = millis();
        float deltaTime = (now - lastTime) / 1000.0;
        if(deltaTime <= 0) return 0;
        
        float error = setpoint - currentPosition;
        integral = constrain(integral + (error * deltaTime), -100, 100);
        float derivative = (error - lastError) / deltaTime;
        
        lastError = error;
        lastTime = now;
        
        return constrain((Kp * error) + (Ki * integral) + (Kd * derivative), -100, 100);
    }

public:
    // Fungsi untuk mengeksekusi gerakan berbasis PID
    void moveToVictim() {
        int x = cam::camx;
        if(x == -1) return;
        
        float pid_output = computePID(x);
        
        if(abs(x - setpoint) < 8) {
            stabilityCounter++;
            if(stabilityCounter >= STABILITY_THRESHOLD) {
                if(TOF::getdepan() < 240) {
                    // Posisi sudah tepat untuk pengambilan
                    return;
                } else {
                    legs::walkspeed = 150;
                    legs::forward_low();
                }
            } else {
                legs::walkspeed = 100;
                if(pid_output > 0) {
                    legs::shift_right_low_slow();
                } else {
                    legs::shift_left_low_slow();
                }
            }
        } else {
            stabilityCounter = 0;
            legs::walkspeed = 150;
            if(pid_output > 0) {
                if(pid_output > 50) {
                    legs::shift_right_low();
                } else {
                    legs::shift_right_low_slow();
                }
            } else {
                if(abs(pid_output) > 50) {
                    legs::shift_left_low();
                } else {
                    legs::shift_left_low_slow();
                }
            }
        }
    }

    // Fungsi untuk mengecek apakah posisi sudah tepat
    bool isInPosition() {
        int x = cam::camx;
        if(x == -1) return false;
        
        return (abs(x - setpoint) < 8) && 
               (stabilityCounter >= STABILITY_THRESHOLD) && 
               (TOF::getdepan() < 240);
    }

    // Reset PID
    void reset() {
        integral = 0;
        lastError = 0;
        lastTime = millis();
        stabilityCounter = 0;
    }
};

VictimPID pidController;

} // namespace victim_pid

#endif
