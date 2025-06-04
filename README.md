# Electric-Motor-Velocity-Controller

# INTRODUCTİON

In this project, we carried out a study on PI control in a microcontroller. We aimed to drive the motor for certain fixed values by writing a code for P and I controllers. By inserting the codes we wrote into the microcontroller, we enabled communication with the Ardiunho, thus aiming to drive the engine for the KP and K values we entered.

# Code Part
For the first code we right receiver for mikrocontrroller.

    #include <16F877A.h>
    #device ADC=10
    #include <stdlib.h>
    #FUSES XT, NOWDT, NOPROTECT, NOBROWNOUT, NOLVP, NOPUT, NODEBUG, NOCPD
    #use delay(crystal=16000000)
    #include <lcd.c>
    #use rs232 (baud=9600,xmit=PIN_C6, rcv=PIN_C7, parity=N, stop=1) // configuretion for serial communication
    #DEFINE IN1 PIN_C3
    #DEFINE IN2 PIN_C4
    int counter = 0;                          // counter to be used
    char strInput[4];                         // string array for input characters
    unsigned long int inputString;                // inputString value to be used
    unsigned long int revAngle = 0.0f;unsigned long int prevAngle = 0.0f;
    unsigned long int integralError = 0.0f;
    int8 dt = 0.2;
    float Ki = 0.0f;
    float Kp = 0.0f;
    signed int16 error = 0;
    float dx_dt = 0;
    int i = 0;
    #int_ext
    void external_interrupt()
    {
       revAngle++;
    }
    #int_timer0
    void tmr_int()
    {
       set_timer0(60);
       i++;     
       if (i==16)
       {
          integralError = integralError + error * dt;
                dx_dt = (revAngle - prevAngle)*18.75f;    // Caution here
          prevAngle = revAngle;
          i = 0;
       }
    }
    void main()
    {
        unsigned int16 realSpeed = 0;
       unsigned long int pot = 0;
       unsigned long int referance = 0;
       unsigned long int controlOut =0;
       setup_psp(PSP_DISABLED)
      setup_timer_1(T1_DISABLED);
       setup_timer_2(T1_DISABLED,0,1);
       setup_CCP1(CCP_OFF);
       setup_CCP2(CCP_OFF);
          lcd_init();
       lcd_cursor_on(TRUE);
       port_b_pullups(TRUE);
       enable_interrupts(GLOBAL);
       clear_interrupt(int_ext);
       setup_timer_0(RTCC_INTERNAL | RTCC_DIV_256); 
       set_timer0(60);
       enable_interrupts(int_timer0);
       enable_interrupts(int_ext);
       setup_adc_ports(AN0_AN1_AN3); //A0 A1 A3 are configured for analog input pin
       setup_adc(ADC_CLOCK_DIV_32); //enable ADC and set clock for ADC
       //set_tris_c(0x10000000); //set all portb pins as output
       setup_ccp1(CCP_PWM); //4kHz PWM signal output at CCP1 pin 17
       setup_timer_2(T2_DIV_BY_16, 255, 1);
       set_pwm2_duty(0);
       output_low(IN1);
       output_high(IN2);
       
       while(TRUE)
       {
    
      if(kbhit())                            // if data has been received
      {                             
         char i = getc();                    // UART read
         if (i == '*')            // special character for serial input. If the received character is *, then this condition is called
         {
            counter = 0;                             // counter to be zero again
            inputString = atol(strInput); // change string array to long variable
            Kp = inputString/100.0f;
            //printf("\fKp: %f",Kp);                  // print string array to the screen
            printf(LCD_PUTC,"\fKp=%f",Kp);    // print inputString value to the LCD
            memset(strInput, 0, 4);                   // clear the string array
         }
         else if(i== '#')
         {
            counter = 0;                             // counter to be zero again
            inputString = atol(strInput);         // change string array to long variable
            Ki = inputString/100.0f;
            //printf("\nKi: %f",Ki);                  // print string array to the screen
            printf(LCD_PUTC,"\nKi=%f",Ki);    // print inputString value to the LCD
            memset(strInput, 0, 4);                   // clear the string array
         }
         else
         {
            strInput[counter] = i;                    // attend input character to the string array
            counter++;                                // increase the counter by 1
         }
      }
    
     
     set_adc_channel(0);                    // next analog reading will be from channel 0
     delay_us(10);    
    
     pot = read_adc();
     delay_us(10);  
     referance = pot * 7.82f;
     realSpeed = dx_dt;
     error = referance - realSpeed;
     
     controlOut = Kp * error + Ki * integralError ;
     
      if (controlOut >=1023){
         controlOut = 1023;
      
      }
      else if (controlOut <=0){
         controlOut = 0;
      
      }
      
      set_pwm1_duty(controlOut);      //set pulse-width during which signal is high
      /*
      printf(lcd_putc, "\tref_vel:%lu",referance);
      printf(lcd_putc, "\nact_vel:%lu",realSpeed);
      printf(lcd_putc, "\terror:%lu",error);
      printf(lcd_putc, "\tsamp_tm:%d",dt);
      delay_ms(50);
    */
      printf("/nKp: %f",Kp); 
      printf("\nKi: %f",Ki);
      printf("\nReferance_Velocity:%lu",referance);
      printf("\nActual_Velocity:%lu",realSpeed);
      printf("\nError:%lu",error);
      printf("\nSampling_Time:%d",dt);
      delay_ms(1200);

For the reciver code we firstly calculate the reference velocity. Than we try to code P cpntrooler with given information. And we add an I controller.For PI controller we use the Kp and KI values for the code and we try to define them with specific values for every each one.





For the second code transmitter.

    #include <16F877A.h>
    #FUSES XT, NOWDT, NOPROTECT, NOBROWNOUT, NOLVP, NOPUT, NODEBUG, NOCPD
    #use delay(crystal=16000000)
    #use rs232 (baud=9600,xmit=PIN_C6, rcv=PIN_C7, parity=N, stop=1) // configuretion for serial communication
    
    void main()
    {
       setup_psp(PSP_DISABLED);
       setup_timer_1(T1_DISABLED);
       setup_timer_2(T2_DISABLED,0,1);
       setup_adc_ports(NO_ANALOGS);
       setup_adc(ADC_OFF);
       setup_CCP1(CCP_OFF);
       setup_CCP2(CCP_OFF);
       int16 dt = 1000;
    
       while(1)
       {
   

      printf("5");
      delay_ms(dt);
      printf("*");
      delay_ms(dt);
      printf("1");
      delay_ms(dt);
      printf("2");
      delay_ms(dt);
      printf("#");
      delay_ms(dt); }
For the transmitter code we define for ardiunho uno atn we try the communicate the uno and microcontroller. We set the Kp and Kı values for communicate with each other and try to drive and encoder DC motor.


CONCULUTİON

In this project, we learned how to design a PI controller and how to control the motor using Kp Ki values. During these operations, we also learned about communication and value control. While designing the controller, we analyzed, observed and learned how to write code.
 
