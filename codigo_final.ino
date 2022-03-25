#include <Thermistor.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1X15.h>

// General D/A parameters
const int DAC_12BIT_MAX = 4095;
const float DAC_INPUT_RANGE = 5.0;
const float DAC_BIT_VOLTAGE = DAC_INPUT_RANGE / DAC_12BIT_MAX;

// Temperature control system D/A converter
Adafruit_MCP4725 Tdac;
const int T_DAC_ADDR = 0x61;
const float T_DAC_X0 = 1228.5;  // 1.5V on CTLI port

// Current control system D/A converter
Adafruit_MCP4725 Idac;
const int I_DAC_ADDR = 0x60;
const float I_DAC_X0 = 4095/2;

// A/D converter
Adafruit_ADS1115 adc;
const int ADC_ADDR = 0x48;  // ADDR pin = GND
const int ADC_T_CH = 2;
const int ADC_I_CH = 1;
const int ADC_ADDR_PIN = 46;
const float ADC_16BIT_MAX = 65535.0;
const float ADC_INPUT_RANGE = 6.144f;
const double ADC_BIT_VOLTAGE = (ADC_INPUT_RANGE * 2) / (ADC_16BIT_MAX);  // V/bit

// LCD display
LiquidCrystal lcd(31, 37, 29, 27, 25, 23);

// Loop delay (ms)
const int LOOP_DELAY = 500;

// Potentiometers addresses
const int COARSE_T_REF_ADDR = A13;
const int FINE_T_REF_ADDR = A15;
const int COARSE_I_REF_ADDR = A14;
const int FINE_I_REF_ADDR = A12;

// Reference maximum readings
const float MAX_COARSE_T_REF = 36.0;
const float MAX_FINE_T_REF = 4.0;
const float MAX_COARSE_I_REF = 110.0;
const float MAX_FINE_I_REF = 10.0;

// T reading constants
const float TERMISTOR_NOMINAL_R = 10000.0;
const float NOMINAL_T = 25.0;
const float B_COEFFICIENT = 3460.0;
const float T_SERIES_RESISTOR = 10000.0;

////int Vo;
//float R1 = 10000;
//float logR, R2;
//float A = 0.8745448587e-03, B = 2.541593697e-04, C = 1.773637930e-07;  // Steinhart-Hart and Hart Coefficients

//Thermistor temp(0);

// I reading constant
const float I_V_GAIN = ((820.0 / 68.0) * 3.85);  // (R6/R8) * R4 ... A -> V
const float V_I_GAIN = 1000.0 / I_V_GAIN;  // V -> mA

// Interruption parameters
const uint16_t TIMER_RESET_VAL = 0;  // Timer reset value
const uint16_t TIMER_COMP = 60000;  // Timer limit

// Temperature control parameters
const float TKc = 0.01;
const float Tz = 0.8;

// Current control parameters
const float IKc = 0.01;
const float Iz = 0.7;

// Temperature (C) and current (mA) values
float T, I;

// Control inputs (V)
float Vin, VCTLI;
const float VCTLI_OFFSET = 1.5;
const float MAX_VCTLI = 1.0;
const float MIN_VCTLI = -1.0;

// Temperature and current references
float IRef, TRef, TRefk_1, IRefk_1;

// State variables
float Ie_km1, Iu_km1, Te_km1, Tu_km1;

void initConversors() {
    // A/D converter initialization
    pinMode(ADC_ADDR_PIN, OUTPUT);
    digitalWrite(ADC_ADDR_PIN, LOW);
    adc.setGain(GAIN_TWOTHIRDS);  // +/- 6.144V -> 1 bit = 0.1875mV
    //adc.setGain(GAIN_ONE);
    adc.begin(ADC_ADDR);

    // D/A converters address initialization
    Tdac.begin(T_DAC_ADDR);
    Idac.begin(I_DAC_ADDR); 

    // D/A converters output initialization
    Tdac.setVoltage(T_DAC_X0, false);
    Idac.setVoltage(I_DAC_X0, false);
}

float readTRef() {
    float coarseT = (analogRead(COARSE_T_REF_ADDR) * MAX_COARSE_T_REF) / 1023.0;
    float fineT = (analogRead(FINE_T_REF_ADDR) * MAX_FINE_T_REF) / 1023.0;
    
    TRef = coarseT + fineT;
    TRef = 0.9*TRefk_1 + 0.1*TRef;
    TRefk_1 = TRef;
}

float readIRef() {
    float coarseI = (analogRead(COARSE_I_REF_ADDR) * MAX_COARSE_I_REF) / 1023.0;
    float fineI = (analogRead(FINE_I_REF_ADDR) * MAX_FINE_I_REF) / 1023.0;

    IRef = coarseI + fineI;
    IRef = 0.9*IRefk_1 + 0.1*IRef;
    IRefk_1 = IRef;
}

void readT() {

  int16_t intT = adc.readADC_SingleEnded(ADC_T_CH);
  float v = intT*ADC_BIT_VOLTAGE;
  float x = ((5/v)-1);
  x = ((T_SERIES_RESISTOR/TERMISTOR_NOMINAL_R)*x);
  x = log(x);
  x /=  B_COEFFICIENT;
  x +=  1.0/(NOMINAL_T + 273.15);
  x = 1.0/x;
  T =  x - 273.15;
 
}

float readI() {
    int intI = adc.readADC_SingleEnded(ADC_I_CH);
    //Serial.print("intI: ");
    //Serial.println(intI);
    float vI = intI * ADC_BIT_VOLTAGE;
    //Serial.print("vI: ");
    //Serial.println(vI);
    //Serial.print("ADC_BIT_VOLTAGE: ");
    //Serial.println(ADC_BIT_VOLTAGE);
    I = vI * V_I_GAIN;
}

void updateInterface() {
    // I reference
    lcd.setCursor(0,0);  
    lcd.print("Ir:"); 
    lcd.setCursor(3,0); 
    lcd.print(IRef);
    if (IRef < 100 && IRef > 10){
        lcd.setCursor(8,0);
        lcd.print("  ");
    }
    if (IRef < 10){
        lcd.setCursor(7,0);
        lcd.print("  ");
    }
    lcd.setCursor(9,0);  
    lcd.print(" mA");

    // T reference
    lcd.setCursor(0,1);  
    lcd.print("Tr:"); 
    lcd.setCursor(3,1); 
    lcd.print(TRef);
    lcd.setCursor(7,1);  
    lcd.print("C");

    // T
    lcd.setCursor(9,1);  
    lcd.print("T:"); 
    lcd.setCursor(11,1); 
    lcd.print(T);
    lcd.setCursor(15,1);  
    lcd.print("C");
}

void initRefsAndStateVariables() {
    readTRef();
    readIRef();

    readT();
    readI();

    Te_km1 = T - TRef; 
    Tu_km1 = (T_DAC_X0 * DAC_BIT_VOLTAGE) - VCTLI_OFFSET;
    
    Ie_km1 = IRef - I;
    Iu_km1 = I_DAC_X0 * DAC_BIT_VOLTAGE;
}

void initInterrupt() {
    // Timer 1 control register reset
    TCCR1A = 0;

    // Setting a 256-step prescaler 
    TCCR1B |=  (1 << CS12); // Reset -> 0
    TCCR1B &= ~(1 << CS11); // Set -> 1
    TCCR1B &= ~(1 << CS10);

    // Setting an 8-step prescaler 
//    TCCR1B &= ~(1 << CS12); // Reset -> 0
//    TCCR1B |=  (1 << CS11); // Set -> 1
//    TCCR1B &= ~(1 << CS10);

    // Resetting Timer 1 and setting the interruption metric
    TCNT1 = TIMER_RESET_VAL;
    OCR1A = TIMER_COMP;

    // Turning on the Timer 1 comparsion interruption
    TIMSK1 = (1 << OCIE1A);

    // Turning on global interruptions
    sei();
}

void computeIControl() {
    // Compute control
    
    float Ie_k = IRef - I;
    float Iu_k = Iu_km1 + IKc * Ie_k - IKc * Iz * Ie_km1;

    // Saturation
    Iu_k = max(Iu_k, 0.0);
    Iu_k = min(Iu_k, DAC_INPUT_RANGE);

    // Output value update
    Vin = Iu_k;

    // State variables update
    Iu_km1 = Iu_k;
    Ie_km1 = Ie_k;
}

void computeTControl() {
    // Compute control
    float Te_k = T- TRef;
    float Tu_k = Tu_km1 + TKc * Te_k - TKc * Tz * Te_km1;

    // Offset and saturation
    Tu_k = max(Tu_k, MIN_VCTLI);
    Tu_k = min(Tu_k, MAX_VCTLI);

    // Output value update
    VCTLI = Tu_k;
    
    // State variables update
    Tu_km1 = Tu_k;
    Te_km1 = Te_k;
}

void writeVin() {
    float x = Vin / DAC_BIT_VOLTAGE;
    x = min(DAC_12BIT_MAX, x);
    Idac.setVoltage(x, false);
}

void writeCTLI() {
    float x = (VCTLI + VCTLI_OFFSET) / DAC_BIT_VOLTAGE;
    x = min(DAC_12BIT_MAX, x);
    Tdac.setVoltage(x, false);
}

void serialPrint() {
    Serial.print("Ir: ");
    Serial.print(IRef);
    Serial.print(" mA, I: ");
    Serial.print(I);
    Serial.print(" mA, Tr: ");
    Serial.print(TRef);
    Serial.print(" C, T: ");
    Serial.print(T);
    Serial.print("C, Vin: ");
    Serial.print(Vin);
    Serial.print("V, VCTLI: ");
    Serial.print(VCTLI + VCTLI_OFFSET);
    Serial.println(" V");
}

// -------------------- EXECUTION --------------------
void setup() {
    initConversors();
    initRefsAndStateVariables();
    

    lcd.begin(16, 2);
    updateInterface();

    Serial.begin(9600);
    while(!Serial);
    //initInterrupt();
}

void loop() {
    //unsigned long T0 = micros();
    readTRef();
    readIRef();
    updateInterface();
    
    serialPrint();   
    readI();
    computeIControl();
    writeVin();

    // T control
    readT();
    computeTControl();
    writeCTLI();   
    
    //Serial.println(micros()- T0);
}

ISR(TIMER1_COMPA_vect) {
    // Timer 1 reset
    TCNT1 = TIMER_RESET_VAL;
//    readI();
//    computeIControl();
//    writeVin();
//
//    // T control
//    readT();
//    computeTControl();
//    writeCTLI();
//    
}
