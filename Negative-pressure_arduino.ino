#include <LiquidCrystal.h>    //LCD 2x16 library
LiquidCrystal lcd(12,11,5,4,3,2); //Arduino pins to which LCD is connected
#include <PID_v1.h>    //PID library
#define analog  10   // pin digital 10 as analog output
#define sensorpress A0 //sensorpress pressure adquired in analog channel A0.
#define led_alarma  8  //  pin digital 8 connected to alarm LED


//  global variables
float T1s,T2s,P_sensor;
double Setpoint, Input, Output,Kp=0.12, Ki=0.75, Kd=0;   // PID control variables 
unsigned long t0,tiempo ,t1;
int pmax,pmin,consigna,i,T1m,T2m,Tm =10,pmax_1=-1,pmin_1=-1,T1m_1=-1,T2m_1=-1,a=1,pmaxmmhg,pminmmhg;
int samplepressure[25];
double totalpress = 0,pressinzero,zero,samplezero,totalzero=0;
float calpresmmhg =56.25;     //calibration/sensitivity of sensor in mmHg/V  (-225mmHg=0.5V, 4.5V =0mmhg)
char P_sensorstring[5],pmaxstring[4],pminstring[4],T1mstring[3],T2mstring[3];
byte restard;
int ledState = LOW;
unsigned long previousMillis = 0;        // will store last time LED was updated



PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); 

// calibration routine
void setup() {
  analogWrite(analog,0);        //switchs off the vacuum pump to guarantee zero pressure
  delay(5000);                  //5s delay to guarantee zero pressure
 
  pinMode(led_alarma,OUTPUT);   // LED warning as output
  myPID.SetMode(AUTOMATIC);     // PID control will be in automatic mode
  

  //----------------------------------------- calibration process ------------------------------------------------------------------------
  
  for(i=0; i< 24; i++) {samplepressure[i] = 0;}        //array initialization
  lcd.begin(16,2);
  lcd.setCursor(0,0);                       
  lcd.print("Calibrating zero"); 
  lcd.setCursor(0,1); 


for(i=0; i< 91; i++)           //record every 50ms pressure value for a total time of 4.5s and accumulates the sum in totalzero variable
   
   {

if (i==0 || i==6 || i==12 || i==18 || i==24 || i==30 || i==36 || i==42 || i==48 || i==54 || i==60 ||  i==66 || i==72 || i==78 || i==84 || i==90)   //writes * every 300ms in the LCD

  {
  
  lcd.setCursor(i/6,1); 
  lcd.print(".");  
 
  }

  samplezero = analogRead(sensorpress);   //readings of pressure sensor
  totalzero = totalzero + samplezero;    //recordings accumulated 
  delay(50); 

  }

  zero=totalzero/i; //real zero calculated
  lcd.setCursor(0,1); 
  lcd.print("Press:     mmHg     "); 


//---------------  verification that zero pressure correction is OK (<5mmhg). If not, gives an error and restarts  ----------------------

for(i=0; i< 50; i++) {                                 
  
  pressinzero =analogRead(sensorpress)-zero;         // zero deviations correction                                              
  P_sensor=abs(pressinzero*4.8*calpresmmhg/1023);    //conversion of pressure to mmHg
  dtostrf(P_sensor,3,2, P_sensorstring);             //pressure in string for displaying it in LCD display
  lcd.setCursor(6,1);                                
  lcd.print(P_sensorstring);                         
  delay(30); 

    
    if (P_sensor >5 ){        // calibration error                        

          lcd.setCursor(0,0);                      
          lcd.print("err calibrating ");    
          lcd.setCursor(0,1);                     
          lcd.print("                ");
          lcd.setCursor(4,1);                     
          lcd.print(P_sensorstring);              //excessive pressure written in the LCD
          delay(2000);                            
          lcd.setCursor(0,1);                     
          lcd.print("Restard sofware.");          
          delay(1000);                           
          restard =1;                            //restart
          i=50;  
                    }


                          }

if (restard ==0)    {      // calibration ok
   lcd.setCursor(0,0);    
   lcd.print(String(" Calibration OK ")); 

                     }
  delay(2000);  //espera 2 seg
  lcd.clear(); 
  t0=millis();  //t0 and t1 control the time intervals
  t1=millis(); 
  i=0;  


  // mapping channel A4 fro values between 98 and 666 which correspond to pressures between 25 and 175 mmHg 
  pmin=map (analogRead(A4),0,1023,98,666); 
  consigna=pmin; //minimum pressure setpoint

}

// main
void loop() {  
 
 // ---------------- if error, restart setup calibration routine ---------------
 if (restard ==1)    {        
      totalzero=0; 
      restard=0;  
      setup(); 
  
                      } 


//------------------- record analog channels A1 and A2 and map them for values 0-60 ( 0-60 minutes)-----------------

  T1m =map (analogRead(A1),0,1023,0,60);
  T2m =map (analogRead(A2),0,1023,0,60);

//------------------ record analog channels A3 and A4 and map them for values 98 - 666 (25-175 mmHg)------------------

  pmax=map (analogRead(A3),0,1023,98,666);
  pmin=map (analogRead(A4),0,1023,98,666);


if (pmin>pmax) {pmin=pmax;}      //pmin value should never be greater than pmax value

//------------------------ Pmax,Pmin -> mmHg ------------------------------
 
  pmaxmmhg=(pmax*4.8*calpresmmhg/1023); 
  pminmmhg=(pmin*4.8*calpresmmhg/1023);


//---------------------- determines when to apply Pmin or Pmax in function of T1m and T2m -----------------------------------------

  consigna=pmin; 

if (millis()-(t1)>=((T1m+T2m)*60000))  {  // 60s                          
              
              t1=millis();
                               }
if (millis()-(t1)>=(T2m*60000))  { 
              consigna=pmax;
                                    
                                         }
 
//---------------- recording of pressure sensor in variable totalpress ----------------   

  samplepressure[i]= analogRead(sensorpress);
  totalpress =  samplepressure[i] + totalpress; 
  i++;  


// ------------------------------ sampling time 10ms (Tm) ----------------------------------

if (millis()-(t0)>=Tm)  {        
      
          t0=millis();  //restart the time variable
          pressinzero = abs((totalpress/i)-zero);     // zero-corrected absolute value of pressure calculation
          P_sensor=(pressinzero*4.8*calpresmmhg/1023); //pressure -> mmHg
          
//----------------------------- strings for LCD display ---------------------------- 
          dtostrf(pmaxmmhg,3,0, pmaxstring);
          dtostrf(pminmmhg,3,0, pminstring);
          dtostrf(T1m,2,0, T1mstring);
          dtostrf(T2m,2,0, T2mstring);
          dtostrf(P_sensor,4,0, P_sensorstring);
   


// LCD diplay is refreshed only if value changes happened
     
     if ( pmax_1 != pmax)  {             
                pmax_1=pmax;
                lcd.setCursor(0,0);
                lcd.print(pmaxstring);
                            }

    if ( pmin_1 != pmin)  {         
                pmin_1=pmin;
                lcd.setCursor(7,0);
                lcd.print(pminstring);
                           }

    if ( T1m_1 != T1m)  {    
                T1m_1=T1m;
                lcd.setCursor(1,1);
                lcd.print(T1mstring);
                         }
    
    if ( T2m_1 != T2m)  {      
                T2m_1=T2m;
                lcd.setCursor(8,1);
                lcd.print(T2mstring);
                        }
    

//---------------------------------- alarm when pressure <15 or >200 ----------------------------------
    
    if  ( P_sensor< 15 || P_sensor>200)  {             
          
          //LED alarm intermitent
          parpadeo(750);
          digitalWrite (led_alarma,ledState);
          lcd.setCursor(12,0);
          lcd.print(P_sensorstring);
                
              if     (P_sensor>200){            //p>200 -> lcd High
                  
                          lcd.setCursor(12,1);
                          lcd.print("High" );
                          a=1;     
                                    }
              
              if     (P_sensor<15)  {          //p<15 -> lcd low

                          lcd.setCursor(13,1);
                          lcd.print("Low" ); 
                          a=1;  
                                    }

                                       }
    
    
    
//------------------------------- if alarm, clean LCD ------------------
    if ( a != 0)  {
              a= 0;
              lcd.setCursor(12,1);
              lcd.print("    " );
                   }

//--------------------------------  switch OFF alamr and display pressure -------------------- 
    digitalWrite (led_alarma,LOW);
    lcd.setCursor(12,0);
    lcd.print(P_sensorstring);
    
       
 // ------------------------ PID control -----------------------------  

    Input =pressinzero;         //Input pressure
    Setpoint= consigna;         //setpoint for PID is Pmax or Pmin depending on the time
    myPID.Compute();            //PID calculation
    analogWrite(analog,Output); //PWM in digital 10 
    totalpress=0;               
    i=0;                        
                        
                        
                        }   //end of sampling time Tm



} //-------------------------------------------- main loop end----------------------------------------


//-------------------- alarm function --------------

void parpadeo (int interval)

{
unsigned long currentMillis = millis();
 if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == HIGH) {
      ledState = LOW;
    } else {
      ledState = HIGH;
    }

    // set the LED with the ledState of the variable:
   
  }
}
