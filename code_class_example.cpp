#define ML_pin 9
#define MR_pin 10
#define Feedback_pin A0
#define Setpoint_pin A1

#define max_duty 255
#define min_duty 0

// PID parameters
double Kp = 1.1;
double Ki = 0.00225;
double Kd = 0.15;

// PID variables
double error = 0.0;
double d_error = 0.0;
double last_error = 0.0;
double last_millis = 0.0;
double integrate = 0.0;
double last_integrate = 0.0;
double dt = 0.1;
double u_t = 0.0;

int adc_feedback = 0;
int adc_setpoint = 0;
int center_value = 512; 

void setup() {
  Serial.begin(115200);
  pinMode(ML_pin, OUTPUT);
  pinMode(MR_pin, OUTPUT);
  center_value = analogRead(Setpoint_pin);
}

void loop() {
  if(millis() - last_millis >= 1000*dt) {
    adc_feedback = analogRead(Feedback_pin);
    adc_setpoint = analogRead(Setpoint_pin);
    // Serial.print("Setpoint_pin: ");
    // Serial.print(adc_setpoint);
    // Serial.print("Feedback_pin: ");
    // Serial.println(adc_feedback);
    last_millis = millis();
  }
  
  error = adc_setpoint - adc_feedback;
  integrate = last_integrate + error * dt;
  d_error = (error - last_error) / dt;
  u_t = Kp * error + Ki * integrate + Kd * d_error;
  last_error = error;
  last_integrate = integrate;
  PID_control(adc_setpoint,u_t);
  plotGraph(adc_setpoint, adc_feedback, error, u_t);
  delay(50);  
}

void PID_control(int setpoint_value,double u_t) {
  u_t = constrain(u_t, -max_duty, max_duty);
  // Serial.println(u_t);
  int pwm_left = 0;
  int pwm_right = 0;
  if (u_t > 0) {
    pwm_left = abs(u_t);
    pwm_right = min_duty;
  } 
  else if (u_t < 0) {
    pwm_right = abs(u_t);
    pwm_left = min_duty;
  } 
  else {
    pwm_left = min_duty;
    pwm_right = min_duty;
  }
  analogWrite(ML_pin, pwm_left);
  analogWrite(MR_pin, pwm_right);

}

void plotGraph(double setpoint, double feedback, double error, double u_t) {
  Serial.print("Setpoint:");
  Serial.print(setpoint);
  Serial.print(",");

  Serial.print("Feedback:");
  Serial.print(feedback);
  Serial.print(",");

  Serial.print("Error:");
  Serial.print(error);
  Serial.print(",");

  Serial.print("Output:");
  Serial.println(u_t);
}
