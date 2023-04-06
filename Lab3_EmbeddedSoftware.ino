#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 1;
#else
static const BaseType_t app_cpu = 0;
#endif

#define T1_ANALOG_PORT 0 //the GPIO pin responsible for the output of Task1
#define T2_FREQ_IN 1 //GPIO for reading in the frequency for Task2. 333-1000Hz, 50% duty cycle
#define T3_FREQ_IN 2 //GPIO for reading the frequency for Task3. 500-1000Hz, 50% duty cycle
#define V_OUT 4 //pin for turning on the LED for Task4
#define T4_ANALOG_PIN 5 //GPIO for reading in the analog voltage for Task4
#define BUTTON_PIN 7
#define BUTTON_LED LED_BUILTIN

struct frequencyStruct {
int freq1 = 0;
int freq2 = 0;  
};

struct frequencyStruct frequencies;

SemaphoreHandle_t xSemaphore = NULL;

xQueueHandle xQueue;

//bool measurePerformance = false; //setting this to true enables the performance measuring code in loop(); see loop() for more info
//float longestPerformance = 0.0; //longest completion time in microseconds
//int testIterations = 10000; //number of test iterations the timing loop should run
//long freq1 = 0; //the value from Task2 stored as a float in Hz
//long freq2 = 0; //the value from Task3 stored as a float in Hz
float analogueReadings[] = {0,0,0,0}; //initialise the array that stores the last 4 analogue reading of task4
uint8_t arrayIndex = 0; //represents the index at which we write the analog value in the analogueReadings array
uint8_t arrayStoreOperations = 0; //keeps track of how many times we added a number to an array;
float analogValue; //the analog value from task4 

void setup() {
  // initialise all the pins as either inputs or outputs
  pinMode(T1_ANALOG_PORT, OUTPUT);
  pinMode(T2_FREQ_IN, INPUT);
  pinMode(T3_FREQ_IN, INPUT);
  pinMode(V_OUT, OUTPUT);
  pinMode(T4_ANALOG_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(BUTTON_LED, OUTPUT);
  pinMode(8, OUTPUT);
  //internal pull-ups
  Serial.begin(9600); // start serial connection at a baud rate of 9600
  
  //if we arent running the performance loop, start the monitor and start the ticker
  
  xSemaphore = xSemaphoreCreateBinary(); //initialise the semaphore to protect Task2 and Task3 readings
  xSemaphoreGive(xSemaphore);

  xQueue = xQueueCreate(5, sizeof(int)); //create a queue
  
  xTaskCreatePinnedToCore(Task1RTOS,   // pointer to the task function
              "Task 1", // task name
              400,      // stack size in words
			        NULL,     // stack parameter
              1,        // priority
              NULL,
              app_cpu);    // task handle (not used) 
  
  xTaskCreatePinnedToCore(Task2RTOS,   // pointer to the task function
              "Task 2", // task name
              400,      // stack size in words
			        NULL,     // stack parameter
              2,        // priority
              NULL,
              app_cpu);    // task handle (not used) 

  xTaskCreatePinnedToCore(Task3RTOS,   // pointer to the task function
              "Task 3", // task name
              400,      // stack size in words
			        NULL,     // stack parameter
              3,        // priority
              NULL,
              app_cpu);    // task handle (not used) 

  xTaskCreatePinnedToCore(Task4RTOS,   // pointer to the task function
              "Task 4", // task name
              1000,      // stack size in words
			        NULL,     // stack parameter
              1,        // priority
              NULL,
              app_cpu);    // task handle (not used) 
  xTaskCreatePinnedToCore(Task5RTOS,   // pointer to the task function
              "Task 6", // task name
              1000,      // stack size in words
			        NULL,     // stack parameter
              1,        // priority
              NULL,
              app_cpu);    // task handle (not used) 
  xTaskCreatePinnedToCore(buttonPollingTask,   // pointer to the task function
              "Button poll", // task name
              600,      // stack size in words
			        NULL,     // stack parameter
              1,        // priority
              NULL,
              app_cpu);    // task handle (not used) 
  xTaskCreatePinnedToCore(ledTask,   // pointer to the task function
              "led task", // task name
              800,      // stack size in words
			        NULL,     // stack parameter
              1,        // priority
              NULL,
              app_cpu);    // task handle (not used) 

  //vTaskStartScheduler();
  
}

void loop() {

}

/*
the frame function is called by the Ticker object every 4ms
The switch structure controlls which frame is called, indexed by the frameCount integer
this integer is increased by 1 at the end of the function and the % operator keeps it bounded between 0 and 49
*/

//Button read
void buttonPollingTask(void* pvParameters) {

  portTickType xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount(); 

  bool lastRead, newRead = false;

  portBASE_TYPE xStatus;

  int output = 1;

  while(true) {
    vTaskDelayUntil(&xLastWakeTime, (4/portTICK_RATE_MS));
    newRead = digitalRead(BUTTON_PIN);
    /* debouncing code works by comparing the current and previous values to ensure there is no double-shot behaviour8*/
    if (newRead == true && lastRead == false) {
      //event queue sends a value of 1 to be recieved by the led function
      xStatus = xQueueSendToBack(xQueue, &output, 0);

    	if (xStatus != pdPASS) {
    		//Serial.println("Could not send to the queue!\n");
    	}
    }
    lastRead = newRead;

  }
}

//LED task
void ledTask(void* pvParameters) {

  portTickType xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();

  portBASE_TYPE xStatus;

  const portTickType xTicksToWait = 10 / portTICK_RATE_MS;

  int input = 0;

  bool toggle = false;

  while(true) {
    vTaskDelayUntil(&xLastWakeTime, (8/portTICK_RATE_MS));

		xStatus = xQueueReceive(xQueue, &input, xTicksToWait);
  //if we recieved from queue successfuly 
		if (xStatus == pdPASS) {
      if (input == 1) {
        //switch toggle
        toggle = !toggle;
        //set to either 1 or 0
        digitalWrite(BUTTON_LED, toggle);
      }
		}
		else {
			//Serial.println("Could not receive from the queue!");
		}
  }   
}


//Outputs a signal HIGH for 200us and LOW for 50us, and HIGH for 30us
//Repeats every 4ms; T1 = 4ms;
//longest completion time: 304microseconds
//
/*void Task1(uint8_t pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(200);
  digitalWrite(pin, LOW);
  delayMicroseconds(50);
  digitalWrite(pin,HIGH);
  delayMicroseconds(30);
  digitalWrite(pin, LOW);
} */

void Task1RTOS(void* pvParameters) {

  portTickType xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();  

  while(true) {
    
    digitalWrite(T1_ANALOG_PORT, HIGH);
    delayMicroseconds(200);
    digitalWrite(T1_ANALOG_PORT, LOW);
    delayMicroseconds(50);
    digitalWrite(T1_ANALOG_PORT,HIGH);
    delayMicroseconds(30);
    digitalWrite(T1_ANALOG_PORT, LOW);
    vTaskDelayUntil(&xLastWakeTime, (4/portTICK_RATE_MS));
  }
}

/*
Task 2 measures the frequency of a 50% duty cycle square wave
The frequencies are measured between 333-1000Hz
a frequenct less then 333Hz might not be captured reliably due to the timeout

The task reads in the digital state of the wave at the time the task was enetered.
Afterwards the pulse in function waits for a change relative to the value read previously
eg. if the read is LOW(0x0), the pulseIn function is called with a HIGH(0x1) value, meaning
the functions waits for a change from LOW(0x0) to HIGH(0x1) and then waits for a change from HIGH to LOW
the amount of time is measured as a length of one pulse, in microseconds

Since the duty-cycle is 50%, the time is doubled to obtain the time of the full pulse.
Before returning the period is converted to frequency in Hz.
*/
//worst runtime 3020 microseconds
/*void Task2(uint8_t pin) {

  byte pinState = digitalRead(pin); //store the digital value registered on the oin at startup
  float halfWave = pulseIn(pin, !pinState, 3000L); //the timeout of 3ms guaranteed that we will catch a wave with the lowest freq of 333Hz
  float pulse = 2*halfWave;
  //prevent a division by 0
  if (pulse != 0) {
    freq1 = 1000000/pulse;
  }
  else {
    freq1 = 0;
  }

} */

void Task2RTOS(void* pvParameter) {

  portTickType xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount(); 

  long freq1;

  while (true) {
    vTaskDelayUntil(&xLastWakeTime, (20/portTICK_RATE_MS));
    byte pinState = digitalRead(T2_FREQ_IN); //store the digital value registered on the oin at startup
    float halfWave = pulseIn(T2_FREQ_IN, !pinState, 3000L); //the timeout of 3ms guaranteed that we will catch a wave with the lowest freq of 333Hz
    float pulse = 2*halfWave;
    //prevent a division by 0
    if (pulse != 0) {
      freq1 = 1000000/pulse;
      
    }
    else {
      freq1 = 0;
    }

    if (xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
        {
            /*We were able to obtain the semaphore and can now access the
            shared resource.*/
            frequencies.freq1 = (int) freq1;
            xSemaphoreGive( xSemaphore );
        }
        else
        {
            /* We could not obtain the semaphore and can therefore not access
            the shared resource safely. */
        }
    //Serial.println(freq1);
    
  }
}



/*
Task 3 measures the frequency of a 50% duty cycle square wave and is identical to task2
The frequencies are measured between 500-1000Hz
a frequenct less then 500Hz might not be captured reliably due to the timeout

The task reads in the digital state of the wave at the time the task was enetered.
Afterwards the pulse in function waits for a change relative to the value read previously
eg. if the read is LOW(0x0), the pulseIn function is called with a HIGH(0x1) value, meaning
the functions waits for a change from LOW(0x0) to HIGH(0x1) and then waits for a change from HIGH to LOW
the amount of time is measured as a length of one pulse, in microseconds

Since the duty-cycle is 50%, the time is doubled to obtain the time of the full pulse.
Before returning the period is converted to frequency in Hz.
*/
//longest execution 2502microsecond
/*void Task3(uint8_t pin) {

  byte pinState = digitalRead(pin); //store the digital value registered on the pin at startup
  float halfWave = pulseIn(pin, !pinState, 2000L); //the timeout of 2ms guaranteed that we will catch a wave with the lowest freq of 500Hz
  float pulse = 2*halfWave;
  if (pulse != 0) {
    freq2 = 1000000/pulse;
  }
  else {
    freq2 = 0;
  }

} */

void Task3RTOS(void* pvParameter) {

  portTickType xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount(); 

  long freq2;

  while (true) {
    
    byte pinState = digitalRead(T3_FREQ_IN); //store the digital value registered on the oin at startup
    float halfWave = pulseIn(T3_FREQ_IN, !pinState, 2000L); //the timeout of 3ms guaranteed that we will catch a wave with the lowest freq of 333Hz
    float pulse = 2*halfWave;
    //prevent a division by 0
    if (pulse != 0) {
      freq2 = 1000000/pulse;
      
    }
    else {
      freq2 = 0;
    }
    if (xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
        {
            /*We were able to obtain the semaphore and can now access the
            shared resource.*/
            frequencies.freq2 = (int) freq2;
            xSemaphoreGive( xSemaphore );
        }
    else
    {
        /* We could not obtain the semaphore and can therefore not access
        the shared resource safely. */
    }
    vTaskDelayUntil(&xLastWakeTime, (8/portTICK_RATE_MS));
    //Serial.println(freq1);
    
  }
}

/*
Task 4 reads an analog value on readPin and averages it with the last 4 readings. The readings get stored inside
the analogueReadings[] array, which has a fixed size of 4.
The array functions as a circular buffer, ensuring a fast runtime and space efficiency

The arrayStoreOperations counter keeps count of how times a value was read, up to 4.
This is done to ensure the average is computed correctly when less than 4 readings are availabe at the start
After it reaches 4, it will stay 4 untill the system is reset

When the computed sum is more than half of the total range (12bit ADC means 4096 => half is 4096/2) a high signal
is written to the ledPin, toggling an LED
*/
//162 microseconds
/*void Task4(uint8_t readPin, uint8_t ledPin) {

  analogueReadings[arrayIndex] = analogRead(readPin);
  arrayIndex = (arrayIndex + 1)%4;
  if (arrayStoreOperations < 4) {
    arrayStoreOperations++;
  }
  float arraySum = analogueReadings[0] + analogueReadings[1] + analogueReadings[2] + analogueReadings[3];
  arraySum = arraySum/arrayStoreOperations;
  //esp32 has a 12bit adc so 4096 is the highest read possible (3.3V)
  if (arraySum > 4096/2) {
    digitalWrite(ledPin, HIGH);
  }
  else {
    digitalWrite(ledPin, LOW);
  }
  analogValue = arraySum;

} */

void Task4RTOS(void* pvParameter) {

  portTickType xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();

  while(true) {
    
    analogueReadings[arrayIndex] = analogRead(T4_ANALOG_PIN);
    arrayIndex = (arrayIndex + 1)%4;
    if (arrayStoreOperations < 4) {
      arrayStoreOperations++;
    }
    float arraySum = analogueReadings[0] + analogueReadings[1] + analogueReadings[2] + analogueReadings[3];
    arraySum = arraySum/arrayStoreOperations;
    //esp32 has a 12bit adc so 4096 is the highest read possible (3.3V)
    if (arraySum > 4096/2) {
      digitalWrite(V_OUT, HIGH);
    }
    else {
      digitalWrite(V_OUT, LOW);
    }
    analogValue = arraySum;
    vTaskDelayUntil(&xLastWakeTime, (20/portTICK_RATE_MS));
  }
}

/*
Task 5 prints the frequencies from task2 and task3 in a comma deliminated formar "0,0"
in a range between 0-99 where 0 corresponds to a frequency of 333Hz/500hz or lower and 99 to a frequency of 1000Hz and higher

The data is converted using the map() and constrain() methods
*/
//999us
/*void Task5(float freq1In, int freq1low, int freq1high, float freq2In, int freq2low, int freq2high) {

  int freq1mapped = constrain(map(freq1In, freq1low, freq1high, 0, 99), 0, 99);
  int freq2mapped = constrain(map(freq2In, freq2low, freq2high, 0, 99), 0, 99);  
  Serial.print(freq1mapped);
  Serial.print(",");
  Serial.print(freq2mapped);
  Serial.println();
  delay(1); //delay to stabilise the print


} */

void Task5RTOS(void* pvParameter) {

  portTickType xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();

  int freq1 = 0;
  int freq2 = 0;
  while(true) {
    
    
    if (xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
    {
        /*We were able to obtain the semaphore and can now access the
        shared resource.*/
        freq1 = frequencies.freq1;
        freq2 = frequencies.freq2;
        xSemaphoreGive( xSemaphore );
    }
    else
    {

        /* We could not obtain the semaphore and can therefore not access
        the shared resource safely. */
    }
    int freq1mapped = constrain(map(freq1, 333, 1000, 0, 99), 0, 99);
    int freq2mapped = constrain(map(freq2, 500, 1000, 0, 99), 0, 99);  
    Serial.print(freq1mapped);
    Serial.print(",");
    Serial.print(freq2mapped);
    Serial.println();
    vTaskDelayUntil(&xLastWakeTime, (100/portTICK_RATE_MS));
  }
}







