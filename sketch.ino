// include the library code:
#include <LiquidCrystal.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

// Creates an LCD object. Parameters: (rs, enable, d4, d5, d6, d7)
LiquidCrystal lcd(9, 8, 4, 5, 6, 7);

// creates mpu object # 0
Adafruit_MPU6050 mpu_0;
// creates objects used for reading gyro # 0
sensors_event_t a_0, g_0, temp_0;

// creates mpu object # 1
Adafruit_MPU6050 mpu_1;
// creates objects used for reading gyro # 1
sensors_event_t a_1, g_1, temp_1;




Servo azimuth;
Servo altitude;

//rotation values for each axis
//gyro 0 (helmet)
double alt_rad_0 = 0;
double az_rad_0 = 0;

//gyro 1 (turret)
double alt_rad_1 = 0;
double az_rad_1 = 0;

// distance from observer to turret in cm
double turret_x = 10; //horizontal offset
double turret_z = 0; //vertical offset

int alt_pin = 11;
int az_pin = 10;

//angles to set turret servos
int ang_az = 0; // azimuth angle
int ang_alt = 0; // altitude angle

int alt_buffer = 0;
int az_buffer = 90;

int alt_running = 90;
int az_running = 90;

int alt_new = 0;
int az_new = 0;

//location of target in cartesian coordinates
double tar_x, tar_y, tar_z = 0;

//adjusted cartesian coordinates relative to turret
double x_prime, y_prime, z_prime = 0;

double dist = 0; // distance (cm) to target
int dist_thresh = 350; //threshold distance (cm) beyond which distance to target should be considered infinite

//value strings to display on screen for each axis
//gyro 0 (helmet)
String disp_alt_0 = ""; 
String disp_az_0 = "";

//gyro 1 (turret)
String disp_alt_1 = "";
String disp_az_1 = "";

//distance to target string
String disp_dist = "";
 

//define pins for sr04
int trigPin = 13;    // Trigger
int echoPin = 12;    // Echo


// time stamps and delta for gyro 0
long time_0, time_m1_0 = 0;
double time_delta_0 = 0;


// time stamps and delta for gyro 1
long time_1, time_m1_1 = 0;
double time_delta_1 = 0;


//minimum time delay between loops
long t_min = 100;

// //reticle bitmaps
byte ret_UL[8] = {0x00, 0x00, 0x00, 0x07, 0x08, 0x10, 0x10, 0x1d};
byte ret_UR[8] = {0x00, 0x00, 0x00, 0x1c, 0x12, 0x11, 0x01, 0x11};
byte ret_LR[8] = {0x17, 0x01, 0x01, 0x02, 0x1c, 0x00, 0x00, 0x00};
byte ret_LL[8] = {0x11, 0x10, 0x11, 0x09, 0x07, 0x00, 0x00, 0x00};

// };


bool dispay_vals_yn = false;



// take in current angle, the radians per second, and the time elapsed
// calculate the new angle
double update_ang(double ang, double ang_sec, int time_millis)
{

	double ang_new = ang + ang_sec*(double)time_millis/1000;
	//return new angle
	return ang_new;
}

long get_dist_cm()
{
	// clear trigger pin, force low
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

	//trigger ultrasonic pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  long duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  double cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343

	if(cm > dist_thresh)
	{
		return 99999999;
	}
	else
	{
		return cm;
	}
}

String prep_str(double input, int max_prec, int max_len)
{
	// cast double value to a string with max 2 dec places
	String disp_string = String(input, max_prec);

	// if the string is greater than 4 characters total, truncate
	if(disp_string.length() > max_len)
	{
		//iterating through all indeces of the string > max desired length and removing them
		//could be optimized?
		for(int i = max_len; i<disp_string.length();i++)
		{
			disp_string.remove(i);
		};
	};

	// return cleaned string
	return disp_string;
}

void two_lines(String line1, String line2, long d = 0)
{
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print(line1);
	lcd.setCursor(0,1);
	lcd.print(line2);
	delay(d);

}

void read_gyro(Adafruit_MPU6050 &mpu, sensors_event_t &a, sensors_event_t &g, sensors_event_t &temp, long &time, long &time_m1, double &time_delta)
{
	/* get gyroscope readings */
	mpu.getEvent(&a, &g, &temp);
	//identify the time that the gyro readings were taken
	time = millis();
	// calculate the duration between the last time a gyro measurement was taken
	time_delta = time - time_m1;
	//set reference time to current time 
	time_m1 = millis();
}

double rad_to_ang(double rad)
{
	return (180/3.14159)*rad;
}

void target_coordinate_adjust(double alt_0, double az_0, double alt_1, double az_1, double L)
{
	double alt = alt_0-alt_1;

	double az = az_0-az_1;

	tar_x = L*cos(alt)*sin(az);
	tar_y = L*cos(alt)*cos(az);
	tar_z = L*sin(alt);
	
	// Serial.print("T_x:" + String(tar_x)+",");
	// Serial.print("T_y:" + String(tar_y)+",");
	// Serial.print("T_z:" + String(tar_z) + "\n");

	x_prime = tar_x - turret_x;
	y_prime = tar_y;
	z_prime = tar_z - turret_z;
}

int alt_correct()
{
	// Use atan2 so we get negative angles if z_prime is negative.
	// The horizontal distance is the 'radius' in the x-y plane:
	double horizontal_dist = sqrt(x_prime * x_prime + y_prime * y_prime);

	// alt_prime will be between -90 (pointing well below horizontal) and +90 (pointing above horizontal).
	double alt_prime = atan2(z_prime, horizontal_dist);

	// Convert to degrees
	double alt_prime_deg = alt_prime * 180.0 / 3.14159;

	// If you want to ensure you never exceed -90..+90 in case of noisy data:
	if (alt_prime_deg < -90.0)
	{
		alt_prime_deg = -90.0;
	}
	else if (alt_prime_deg > 90.0)
	{
		alt_prime_deg = 90.0;
	}

	// Return an integer in the range -90..+90
	return (int) round(alt_prime_deg);


}

int az_correct()
{
	
	// double az_prime = atan2(y_prime, x_prime);
	
	// double az_prime_deg = degrees(az_prime);
	// if(az_prime_deg < 180)
	// {
	// 	return floor(az_prime_deg);
	// }

	// Compute raw azimuth in radians via atan2(y,x)
	// Range is (-π..+π), i.e. -180..+180 degrees.
	double az_prime = atan2(y_prime, x_prime);

	// Convert to degrees
	double az_prime_deg = az_prime * 180.0 / 3.14159;

	// // If you only want to allow the turret to swivel -90..+90 from "forward":
	// // clamp the value here
	// if (az_prime_deg < -90.0)
	// {
	// 	az_prime_deg = -90.0;
	// }
	// else if (az_prime_deg > 90.0)
	// {
	// 	az_prime_deg = 90.0;
	// }

	// Return integer in [-90..+90]
	return (int)round(az_prime_deg);
}

// Function to generate a progress bar string
String generateProgressBar(int progress) {
    const int barWidth = 14; // Width of the progress bar ignoring endcaps
    String progressBar = "[";

    int pos = barWidth * progress / 100;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) progressBar += "=";
        else if (i == pos) progressBar += ">";
        else progressBar += " ";
    }
    progressBar += "]";
    return progressBar;
}

void setup() 
{
	int load_delay_0 = 250;
	int load_delay_1 = 500;

	Serial.begin(115200);

	// initialize LCD and set up the number of columns and rows: 
	lcd.begin(16, 2);
	
	two_lines("STARTUP SEQUENCE","BEGIN",1000);

	// // create a reticle characters

	lcd.clear();

	lcd.createChar(0, ret_UL);
	lcd.createChar(1, ret_UR);
	lcd.createChar(2, ret_LR);
	lcd.createChar(3, ret_LL);


	// Print a message to the lcd.
	//lcd.print("Custom Character");
	
	//lcd.setCursor(8, 1);
	//lcd.write(byte(8));


	// Try to initialize gyro
	if (!mpu_0.begin(0x68, &Wire, 0)) {
		//if gyro does not initialize, print message
		lcd.print("Failed to find");
		lcd.setCursor(0,1);
		lcd.print("MPU6050_0 chip");
		
		//retry in 10 ms
		while (1) {
		  delay(10);
		}
	}
	two_lines("MPU6050_0 FOUND!",generateProgressBar(15),load_delay_0);
	
	// Try to initialize gyro
	if (!mpu_1.begin(0x69, &Wire, 0)) {
		lcd.clear();
		//if gyro does not initialize, print message
		lcd.print("Failed to find");
		lcd.setCursor(0,1);
		lcd.print("MPU6050_1 chip");
		
		//retry in 10 ms
		while (1) {
		  delay(10);
		}
	}
	two_lines("MPU6050_1 FOUND!",generateProgressBar(30),load_delay_0);
	lcd.clear();

	// set accelerometer range to +-8G
	mpu_0.setAccelerometerRange(MPU6050_RANGE_8_G);
	// initialization of accelerometer success, print message
	two_lines("INIT: Accelermtr",generateProgressBar(40),load_delay_0);

	
	// set gyro range to +- 500 deg/s
	mpu_0.setGyroRange(MPU6050_RANGE_500_DEG);
		// initialization of gyro success, print message
	two_lines("INIT: Gyros",generateProgressBar(50),load_delay_0);
	
	// set filter bandwidth to 21 Hz
	mpu_0.setFilterBandwidth(MPU6050_BAND_21_HZ);
	two_lines("INIT: Filters",generateProgressBar(60),load_delay_0);
	
	// initialization of gyro success, print message
	two_lines("INIT: MPU_0",generateProgressBar(75),load_delay_1);
	two_lines("INIT: MPU_1",generateProgressBar(85),load_delay_1);

	//Initialize pins for ultrasonic
	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);

	two_lines("INIT: SR04",generateProgressBar(100),load_delay_1);
	
	two_lines("SENSORS","INITIALIZED!",load_delay_1);


	two_lines("INIT: SERVO-ALT",generateProgressBar(20));
	altitude.attach(alt_pin);
	two_lines("INIT: SERVO-ALT",generateProgressBar(40),1000);
	altitude.write(0);
  lcd.clear();
	two_lines("INIT: SERVO-ALT",generateProgressBar(60),1000);
	altitude.write(180);
	two_lines("INIT: SERVO-ALT",generateProgressBar(80),1000);
	altitude.write(90);
	two_lines("INIT: SERVO-ALT",generateProgressBar(100),1000);


	two_lines("INIT: SERVO-AZ",generateProgressBar(20));
	azimuth.attach(az_pin);
	two_lines("INIT: SERVO-AZ",generateProgressBar(40),1000);
	azimuth.write(0);
	two_lines("INIT: SERVO-AZ",generateProgressBar(60),1000);
	azimuth.write(180);
	two_lines("INIT: SERVO-AZ",generateProgressBar(80),1000);
	azimuth.write(90);
	two_lines("INIT: SERVO-AZ",generateProgressBar(100),1000);

	two_lines("SERVO-ALT,-AZ","INITIALIZED",load_delay_1);

	two_lines("STARTUP SEQUENCE","SUCCESSFUL",load_delay_1);
	two_lines("TARGETING SYSTEM","ONLINE",load_delay_1);

	lcd.clear();

	//displays aiming reticle in center of display
	lcd.setCursor(7, 0);
	lcd.write(byte(0));
	lcd.write(byte(1));
	lcd.setCursor(7, 1);
	lcd.write(byte(3));
	lcd.write(byte(2));


}

void loop() 
{ 
	//read raw gyro values into global. Update time delta since last measurement.
	read_gyro(mpu_0, a_0, g_0, temp_0, time_0, time_m1_0, time_delta_0);
	read_gyro(mpu_1, a_1, g_1, temp_1, time_1, time_m1_1, time_delta_1);
	
	// calculate current angle of helmet
	alt_rad_0 = update_ang(alt_rad_0, g_0.gyro.x, time_delta_0);
	az_rad_0 = update_ang(az_rad_0, g_0.gyro.z, time_delta_0);

	// calculate current angle of turret
	alt_rad_1 = update_ang(alt_rad_1, g_1.gyro.x, time_delta_1);
	az_rad_1 = update_ang(az_rad_1, g_1.gyro.z, time_delta_1);

	//find distance to target
	dist = get_dist_cm();

	//format distance to be displayed
	if(dist > dist_thresh)
	{
		disp_dist = "INF";
	}
	else{
		disp_dist = prep_str(dist, 2, 3);
	}

	// FOR DEBUGGING, COMMENT WHEN NOT IN USE
	//---------------------------------------
	//display angle value, 2 sig figs, 4 chars max
	
	//helmet
		//altitude
  disp_alt_0 = prep_str(degrees(alt_rad_0), 2, 3);
		//azimuth
	disp_az_0 = prep_str(degrees(az_rad_0), 2, 3);

	//turret
		//altitude
	disp_alt_1 = prep_str(degrees(alt_rad_1), 2, 3);
		//azimuth
	disp_az_1 = prep_str(degrees(az_rad_1), 2, 3);
	
	// print values to display
	lcd.setCursor(0,0);
	lcd.print("A:" + disp_alt_1 +  " Z:" + disp_az_1 + "  R");
	lcd.setCursor(0,1);
	lcd.print("A:" + disp_alt_0 + " Z:" + disp_az_0 + " " + disp_dist);

	//---------------------------------------
	// END DEBUG BLOCK


	//FOR NORMAL USE, UNCOMMENT
	//---------------------------------------
	//displays distance to target in upper right
	// lcd.setCursor(0,0);
	// lcd.print("R:"+disp_dist);
	//---------------------------------------


	// void target_coordinate_adjust(double alt_0, double az_0, double alt_1, double az_1, double L)
	target_coordinate_adjust(alt_rad_0, az_rad_0, alt_rad_1, az_rad_1, dist);

	alt_new = alt_correct();
	az_new = az_correct();

	alt_running = round((alt_new+alt_buffer)/2);
	az_running = round((az_new+az_buffer)/2);

	azimuth.write(az_running);
	altitude.write(90+alt_running);

	//delay by a minimum amount

	alt_buffer = alt_running;
	az_buffer = az_running;
	delay(t_min);
}
