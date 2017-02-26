// ADSRduino
//
// a simple ADSR for the Arduino
// m0xpd
// Feb 2017
//
// see http://m0xpd.blogspot.co.uk/2017/02/signal-processing-on-arduino.html
//
// uses a Microchip MCP4921 DAC on digita pins 4,5,6 & 8 (see below)
// receives gate inputs on digital pin 2 (remember to protect e.g. with a 200R resistor and a 5V1 Zener diode)
// and a loop mode input on digital pin 3 (pulling to 0V selects loop mode)
//
// Voltages between 0 and 5V (e.g. from potentiometers) on analog pins A0:A3 control Attack, Decay, Sustain & Release 

#define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }

// Pin definitions...
const int gatePin = 2;
const int modePin = 3;
// MCP4921...
// (pin allocations for convenience in hardware hook-up)
const int DAC_CS = 6;
const int DAC_SCK = 5;
const int DAC_SDI = 4;
const int DAC_LDAC = 8;
byte upper_byte = 0x10;
byte lower_byte = 0;

float alpha=0.7;   // this is the pole location ('time constant') used for the first-order difference equation
double alpha1=0.9;  // initial value for attack
double alpha2=0.9;  // initial value for decay
double alpha3=0.95; // initial value for release

float envelope = 0.0;  // initialise the envelope
float CV0 = 0.0;       // result of reads from potentiometers (yes - it will only be an int, but helps with the casting!)
float CV1 = 0.0;
int CV2 = 0;
float CV3 = 0.0;

int drive = 0;
int sustain_Level = 0;
int scan = 0;
boolean note_active = false;
boolean loop_mode=false;
boolean trigger = false;
boolean decay = false;
boolean release_done = true;

// subroutine to set DAC on MCP4921
void Set_DAC_4921(int DC_Value){
    lower_byte=DC_Value&0xff;
    upper_byte=(DC_Value>>8)&0x0f;
    bitSet(upper_byte,4);
    bitSet(upper_byte,5);   
    digitalWrite(DAC_CS,LOW);
    tfr_byte(upper_byte);
    tfr_byte(lower_byte);
    digitalWrite(DAC_SDI,LOW);   
    digitalWrite(DAC_CS,HIGH);
    digitalWrite(DAC_LDAC,LOW);
    digitalWrite(DAC_LDAC,HIGH);

}
// transfers a byte, a bit at a time, LSB first to the DAC
void tfr_byte(byte data)
{
  for (int i=0; i<8; i++, data<<=1) {
    digitalWrite(DAC_SDI, data & 0x80);
    pulseHigh(DAC_SCK);   //after each bit sent, CLK is pulsed high
  }
}
void setup() {
  
  pinMode(DAC_CS, OUTPUT);
  pinMode(DAC_SCK, OUTPUT);
  pinMode(DAC_SDI, OUTPUT);  
  pinMode(DAC_LDAC, OUTPUT);  
  pinMode(gatePin, INPUT);  
  pinMode(modePin, INPUT);    
  digitalWrite(DAC_CS,HIGH);  
  digitalWrite(DAC_LDAC,HIGH); 
  digitalWrite(DAC_SCK,LOW);
  digitalWrite(DAC_SDI,HIGH);
  digitalWrite(gatePin,HIGH);   
  digitalWrite(modePin,HIGH);  
  Set_DAC_4921(0); 
}

void loop() {
    boolean gate=digitalRead(gatePin);        // read the gate input every time through the loop
    update_params(scan);                      // scan only one of the other inputs each pass 
    
    boolean trigger=gate||(loop_mode&&release_done);  // trigger a, ADSR even if there's a gate OR if we're in loop mode
    while(trigger){  
    if(note_active==false){                   // if a note isn't active and we're triggered, then start one!
    decay = false;
    drive=4096;                               // drive toward full value
    alpha=alpha1;                             // set 'time constant' alpha1 for attack phase 
    note_active=true;                         // set the note_active flag
  } 
   if((decay==false)&&(envelope>4000)&&(drive==4096)){    // if we've reached envelope >4000 with drive= 4096, we must be at the end of attack phase
                                                          // so switch to decay...
    decay = true;                                         // set decay flag
    drive=sustain_Level;                                  // drive toward sustain level
    alpha=alpha2;                                         // and set 'time constant' alpha2 for decay phase
  } 
  
    envelope=((1.0-alpha)*drive+alpha*envelope);     // implement difference equation: y(k) = (1 - alpha) * x(k) + alpha * y(k-1)
    Set_DAC_4921(round(envelope));                   // and output the envelope to the DAC

     if((loop_mode==true)&&(decay==true) && (envelope<(float)(sustain_Level+1.0))){ // in loop mode, break out at the end of the decay
      decay = false;
      break;
    }
    gate=digitalRead(gatePin);                      // read the gate pin (remember we're in the while loop)
    trigger=gate||(loop_mode&&release_done);        // and re-evaluate the trigger function
    }
    
    if(note_active==true){                // this is the start of the release phase
    drive=0;                              // drive towards zero
    alpha=alpha3;                         // set 'time comnstant' alpha3 for release phase
    note_active=false;                    // turn off note_active flag
    release_done=false;                   // and set release_flag done false
  }    
  
    envelope=((1.0-alpha3)*drive+alpha3*envelope);   // implement the difference equation again (outside the while loop)
    Set_DAC_4921(round(envelope));                   // and output envelope
    gate=digitalRead(gatePin);                       // watch out for a new note
    scan+=1;                                         // prepare to look at a new parameter input
    if(envelope<4){                                  // is the release phase ended?
      release_done=true;                             // yes - so flag it
    }
    if(scan==5){                                     // increment the scan pointer
      scan=0;
    }
}

void update_params(int scan){             // read the input parameters
  switch (scan){
  case 0:
  CV0=analogRead(0);                      // get the attack pole location
  alpha1=0.999*cos((1023-CV0)/795);
  alpha1=sqrt(alpha1);  
  break;
  case 1:
  CV1=analogRead(1);                      // get the release pole location
  alpha2=0.999*cos((1023-CV1)/795);
  alpha2=sqrt(alpha2);   
  break; 
  case 2:
  CV2=analogRead(2);                     // get the (integer) sustain level
  sustain_Level=CV2<<2;
  break;
  case 3:
  CV3=analogRead(3);                     // get the release pole location (potentially closer to 1.0)
  alpha3=0.99999*cos((1023-CV3)/795);
  alpha3=sqrt(alpha3);
  break;  
  case 4:                                // read the loop mode input
  loop_mode=!digitalRead(modePin);
  break;  
  
  }
  
}
