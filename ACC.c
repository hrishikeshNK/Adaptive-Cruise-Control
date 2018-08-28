



/*

Important Notes:
The speed of the obstacle is kept at 50kmph, for simplifying the oode,
it can be changed by editing the MACRO for the same

The following distance is set to be between 100 units and 120 units

The distance unit here is not defined, so it is just in 'units', 
since the ultrasonic sensor doesn't have a realistic range for 
Adaptive cruise control

The oscilloscope shows the duty cycle of PWM which is proportional 
to the vehicle speed.
The two DC motors rotate on the same PWM signal

The pot connect at A0 is used to mimic the driver's manual control 
of the car including braking and acceleration.

The pot knob varies the speed of the vehicle in manual mode only 
and is scaled from 0 to 255 kmph speeds

The first few values on the serial monitor on starting the simulation
or toggling switches are not correct, please wait for 2-3 iterations for
them to stabilize

*/


#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

#define echo PB0  
#define trig PB1
// Ultrasound echo(input) and trigger(output)

#define obstacle_speed 50 
//Speed at which the obstacle ahead of the owner's car is moving
#define kd 0.3  
// kd is Derivative gain; decides how much contribution the rate of error change has in the speed actuation
#define kp 0.1
// kp is Proportional gain; decides how much contribution the the error value itself has in the speed actuation


volatile bool engine_switch = false; // Engine ON switch
volatile bool burst_sent = false; // ultrasonic burst depart flag
volatile bool burst_received = false; // ultrasonic burst arrival flag
volatile bool Autonomy = false; // Autonomous cruise control S/W
volatile float overflows = 0; // 1 overflow = 15.9375 u seconds, counts the overflows until ultrsound burst returns
volatile float distance_to_obstacle = 0; // Distance from owner's car to the obstacle ahead
volatile float last_distance; // distance_to_obstacle from last iteration
volatile float error; // the amount of distance to be covered before reaching following dist 
volatile float last_error; //error from last while(1) iteration  
float default_cruise_speed; // automatic cruising speed set by the driver
float proportional; //The proportional applies correction in proportion to error 
float derivative; //The derivative corrects the error according to how fast the error varies wrt time

void initialize_ACC(){
  DDRD &= ~(1 << PD5); // button to increase speed by 10kmph
  DDRD &= ~(1 << PD6); // button to decrease speed by 10kmph
  /* PD5 and PD6 work by polling and not interrupts, so they
  need to be kept pressed for sometime until the speed changes*/
  
  DDRD &= ~(1 << PD2); //echo input
  EICRA |= (1 << ISC00);
  EICRA &= ~(1 << ISC01); // Any logical change generates an interrupt
  EIMSK |= (1 << INT0);
  sei();
  
}

void initialize_engine_switch(){
   DDRD &= ~(1 << PD3); //Engine switch
  EICRA |= (1 << ISC10); 
  EICRA |= (1 << ISC11);
  EIMSK |= (1 << INT1); // Rising edge trigger for engine switch, so press and leave to toggle
  sei();
}


void initialize_ultrasound(){ //ultrasound echo input at PB0 using PCINT0
  
  DDRB &= ~(1 << echo); //Sets echo mapped pin as input
  PCICR |= 0x01;		//pin change used on pin PB0
  PCMSK0 |= 0x01;	
  sei();  
  DDRB |= (1 << trig); 
}

void init_timer(){ // Timer to 
  TCCR0A |= 0x00;  // Set to normal mode
  TCNT0 = 0x00;
  TCCR0B |=  (1 << CS00); // 001 = No prescaling
  TCCR0B &= ~(1 << CS01);
  TCCR0B &= ~(1 << CS02);
  TIMSK0 |= (1 << TOIE0);
  sei();
}


void init_PWM(){	//Using TIMER2 to generate PWM
  DDRB |= (1 << PB2); 		// PWM output at digital pin 10
  TCCR2A |= (1 << WGM21); 	// CTC mode
  TCCR2B |= ((1 << CS20)|(1 << CS21)|(1 << CS22)); // prescalar set to 1024
  TCNT2 = 0x00;
  OCR2A = 255;
  OCR2B = 255;
  TIMSK2 |= (1 << OCIE2A)|(1 << OCIE2B); // Enable compare A and B
  sei();
}


void init_ADC(){
	ADCSRA |= (1 << ADEN );	// Enable the ADC
    ADMUX |= (1 << REFS0); 	//Scale the voltage wrt ARef
}

int read_ADC(){
  ADCSRA |= (1 << ADSC); 	//Begin conversion
  
  while(ADSC == 1); 		// Wait until conversion is complete i.e, ADSC becomes 0
  return (ADC);
}




int main(){
  Serial.begin(9600);
  
  initialize_engine_switch(); // Engine Switch interrupt
  initialize_ACC(); // Autonomous cruise control setup
  initialize_ultrasound(); // ultasound IO setup
  init_timer(); // Timer initialization to measure arrival time of ultrasonic wave
  init_PWM();
  init_ADC();
  
  
 while(1){
   Serial.println("");
   if(!Autonomy && engine_switch){
   	 	default_cruise_speed = read_ADC(); //Read driver's manual control input when not in ACC mode
    	default_cruise_speed /= 4; // Scale to 0 - 255 kmph
   }
    
    
 
    	PORTB |= (1 << trig);
    	_delay_us(10);
    	PORTB &= ~(1 << trig); // Sending a 10us pulse to trigger ultrasound
    
  		while(!burst_sent); // Wait until ultrasonic burst departs
   
      
  		while(!burst_received); //Wait until ultrasonic burst returns
    
    	burst_received = false;
    	overflows += (TCNT0 * 0.0000000625); // Add the remainder of the time after the last overflow
    	
    	distance_to_obstacle = (overflows* 0.01715)-14278;// - 14342;//14342 is the offset to get the right distance
   
   		error = distance_to_obstacle - 100; //100 units is the following distance
   		derivative = (last_error - error)/ 0.8; // it takes 0.8s in simulation for each consecutive reading, therefore delta t = 0.8
      	Serial.print("Dist. to obstacle = ");
  		Serial.println(distance_to_obstacle);
   		Serial.print("Derivative = ");
    	Serial.println(derivative);
 		if(derivative < 0) derivative = 0; //Negative derivative should not affect the output since it could accelerate the car rapidly
   		overflows = 0;
    	_delay_ms(50);
   
        if((PIND & (1<<PD5)) == 0x20 && Autonomy) // works only in ACC mode
        	default_cruise_speed += 10; //increment cruising speed by 10
     	 
      
  		else if((PIND & (1 << PD6)) == 0x40 && Autonomy) // works only in ACC mode
       		default_cruise_speed -= 10; //Decrement cruising speed by 10
    	
    	
      
   if(Autonomy == true){    //When car is in autonomous cruise mode
        proportional = (default_cruise_speed - obstacle_speed)*kp; // speed  error * proporionality gain
     
     	if(distance_to_obstacle <= 120  && distance_to_obstacle >= 100)
         	OCR2B = (default_cruise_speed)- (proportional * 10) - (derivative * kd);
      
   		else if(distance_to_obstacle > 120 && distance_to_obstacle < 160)
        	OCR2B = (default_cruise_speed)- (proportional * 5) - (derivative * kd);
    
    	else if(distance_to_obstacle >= 160 && distance_to_obstacle < 200)
        	OCR2B = (default_cruise_speed)- (proportional * 2) - (derivative * kd);
                   
    	else if(distance_to_obstacle >= 200)
        	OCR2B = (default_cruise_speed)- (derivative * kd); // Scaling the pot input to 0 - 300 kmph
    	else
          	OCR2B = 0;    
   	}  
 	else{  
     OCR2B = default_cruise_speed; //Manual mode; Speed is equal to driver's input(potentiometer)
    }
   
   Serial.print("Speed = ");
    Serial.println(OCR2B);
   
   
   last_error = error;
 }  // while loop ends here
  return 0;
}


ISR(INT0_vect){ //ACC switch ISR
  
  Autonomy = !Autonomy;
  if(Autonomy == true)
  	Serial.println("Autonomous Cruise Control ACTIVE");
  else
    Serial.println("Autonomous Cruise Control Disengaged");
}

ISR(INT1_vect){ //Engine Switch ISR
  engine_switch = !engine_switch;
  if(engine_switch)
    Serial.println("Engine ON");
  else
    Serial.println("Engine OFF");
}


ISR(PCINT0_vect){ // Ultrasound burst send and recieve flags are set by this ISR
    if(burst_sent == false){ 
    burst_sent = true;
   	TCNT0 = 0;
  }
  
 	else if(burst_sent == true){
    burst_sent = false;
    burst_received = true;
  }
}


ISR(TIMER0_OVF_vect){
  overflows += 15.9375; // Each overflow of timer 0 takes 15.9375 us
}


ISR(TIMER2_COMPA_vect){
  PORTB |= (1 << PB2);
}

ISR(TIMER2_COMPB_vect){
  PORTB &= ~(1 << PB2);
}

