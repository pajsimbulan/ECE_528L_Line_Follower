/**
 * @file main.c
 *
 * @brief Main source code for the Line_Follower program.
 *
 * This file contains the main entry point for the Line_Follower program.
 * The main controller demonstrates a Line Follower robot without using an advanced algorithm.
 *
 * It interfaces the following peripherals using GPIO to demonstrate line following:
 *  - 8-Channel QTRX Sensor Array module
 *
 * Timers are used in this lab:
 *  - SysTick:  Used to generate periodic interrupts at a specified rate (1 kHz)
 *  - Timer A0: Used to generate PWM signals that will be used to drive the DC motors
 *  - Timer A1: Used to generate periodic interrupts at a specified rate (1 kHz)
 *
 * @note For more information regarding the 8-Channel QTRX Sensor Array module,
 * refer to the product page: https://www.pololu.com/product/3672
 *
 * @author
 *
 */

#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Reflectance_Sensor.h"


// Initialize a global variable for SysTick to keep track of elapsed time in milliseconds
volatile uint32_t SysTick_ms_elapsed = 0;

/**
 * @brief Interrupt service routine for the SysTick timer.
 *
 * The interrupt service routine for the SysTick timer increments the SysTick_ms_elapsed
 * global variable to keep track of the elapsed milliseconds.
 *
 * @param None
 *
 * @return None
 */
void SysTick_Handler(void)
{
    SysTick_ms_elapsed++;
}

//Initialize constant PWM duty cycle values for the motors
#define PWM_NOMINAL 2500


#define TURN_CONFIRM_MS 12
#define CORNER_STOP_MS 100
#define T_WAIT_MS 500
#define T_TURN_MS 2000
#define CORNER_TURN_MS 1000

typedef enum {
    NO_TURN,
    CHECK_JUNCTION,
    PIVOT_LEFT,
    PIVOT_RIGHT,
    PIVOT_AROUND
} Pivot;

static Pivot pending_turn = NO_TURN;
static Pivot previous_junction = NO_TURN;
static Pivot confirmed_turn = NO_TURN;
static uint32_t turn_start_ms = 0;
static uint8_t pivot_started = 0;

//initialized a global variable for Timer A1 to keep track of elapsed time in milliseconds
uint32_t Timer_A1_ms_elapsed = 1;

typedef enum {
    CENTER = 0x18,
    SLIGHTLEFT = 0x1C,
    SLIGHTRIGHT = 0x38,
    FARLEFT = 0x01,
    FARRIGHT = 0x80,
    LEFTTURN = 0xF8,
    RIGHTTURN = 0x1F,
    TINT = 0xFF,
    DEADEND = 0x00,
}Line_Position;

/**
 * @brief Maps the observed junction patterns to a pivot direction.
 */
static Pivot Read_Junction(uint8_t sensor_data)
{
    switch (sensor_data)
    {
        case 0xB8:
        case 0xFC:
        case 0xFE:
        case LEFTTURN:
            return PIVOT_LEFT;
        case 0x3F:
        case 0x7F:
        case RIGHTTURN:
            return PIVOT_RIGHT;
        case TINT:
            return PIVOT_AROUND;
        default:
            return NO_TURN;
    }
}

/**
 * @brief Stops the robot and schedules a timed pivot.
 * @param turn Left corner, right corner, or T-intersection turnaround.
 */
static void Start_Turn(Pivot turn)
{
    Motor_Stop();
    if (turn == PIVOT_AROUND)
    {
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_BLUE);
    }
    else
    {
        LED1_Output(RED_LED_ON);
        LED2_Output((turn == PIVOT_LEFT) ? RGB_LED_WHITE : RGB_LED_SKY_BLUE);
    }

    pending_turn = turn;
    turn_start_ms = SysTick_ms_elapsed;
    pivot_started = 0;
}

/**
 * @brief Advances the stop, pivot, and forward sequence on periodic updates.
 */
static void Update_Turn(void)
{
    uint32_t elapsed_ms = SysTick_ms_elapsed - turn_start_ms;

    if (!pivot_started)
    {
        uint32_t stop_ms = (pending_turn == PIVOT_AROUND) ? T_WAIT_MS : CORNER_STOP_MS;
        if (elapsed_ms < stop_ms)
            return;

        if (pending_turn == PIVOT_LEFT)
            Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
        else
            Motor_Right(PWM_NOMINAL, PWM_NOMINAL);

        turn_start_ms = SysTick_ms_elapsed;
        pivot_started = 1;
        return;
    }

    uint32_t pivot_ms = (pending_turn == PIVOT_AROUND) ? T_TURN_MS : CORNER_TURN_MS;
    if (elapsed_ms >= pivot_ms)
    {
        pending_turn = NO_TURN;
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_GREEN);
        Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
    }
}

/**
 * @brief Steers from one sensor reading or advances a timed turn in progress.
 * @param reflectance_sensor_data Eight-bit reflectance sensor reading.
 */
void Detect_Line_Position(uint8_t reflectance_sensor_data)
{
    Pivot junction = Read_Junction(reflectance_sensor_data);

    if (pending_turn == CHECK_JUNCTION)
    {
        // Confirm a direction only when two fresh readings agree.
        if ((junction != NO_TURN) && (junction == previous_junction))
        {
            if ((confirmed_turn == NO_TURN) || (junction == PIVOT_AROUND))
                confirmed_turn = junction;
        }
        previous_junction = junction;

        if ((SysTick_ms_elapsed - turn_start_ms) >= TURN_CONFIRM_MS)
        {
            if (confirmed_turn != NO_TURN)
                Start_Turn(confirmed_turn);
            else
                pending_turn = NO_TURN;
        }
        return;
    }

    if (pending_turn != NO_TURN)
    {
        Update_Turn();
        return;
    }

    if (junction != NO_TURN)
    {
        pending_turn = CHECK_JUNCTION;
        turn_start_ms = SysTick_ms_elapsed;
        previous_junction = junction;
        confirmed_turn = NO_TURN;
        return;
    }

    switch(reflectance_sensor_data) {
        case CENTER:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_GREEN);
            Motor_Forward(PWM_NOMINAL,PWM_NOMINAL);
            break;
        case 0x06:  // 0000_0110
        case 0x07:  // 0000_0111
        case 0x03:  // 0000_0011
        case SLIGHTLEFT:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_YELLOW);

            // Robot is left of the line: steer right slightly
            Motor_Forward(PWM_NOMINAL, PWM_NOMINAL * 3 / 5);
            break;
        case 0x30:  // 0011_0000
        case 0x70:  // 0111_0000
        case 0xE0:  // 1110_0000
        case 0xC0:  // 1100_0000
        case SLIGHTRIGHT:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_PINK);

            // Robot is right of the line: steer left slightly
            Motor_Forward(PWM_NOMINAL * 3 / 5, PWM_NOMINAL);
             break;
        case FARLEFT:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_WHITE);
            Motor_Forward(PWM_NOMINAL, PWM_NOMINAL * 3 / 7);
             break;
        case FARRIGHT:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_SKY_BLUE);
            Motor_Forward(PWM_NOMINAL * 3 / 7, PWM_NOMINAL);
             break;
        case DEADEND:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_RED);
            Motor_Stop();
             break;
        default:
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_OFF);
             break;
    }
}

void Timer_A1_Periodic_Task(void) {
    //Increment Timer_A1_ms_elapsed by 1 every time the Timer A1 periodic interrupt occrs
    Timer_A1_ms_elapsed++;

    // Start a sensor reading every 2 ms; finish it on the next 1 ms interrupt
    if((Timer_A1_ms_elapsed %2) == 0) {
        Reflectance_Sensor_Start();
    }
    if((Timer_A1_ms_elapsed %2) == 1) {
        uint8_t Reflectance_Sensor_Data = Reflectance_Sensor_End();
        Detect_Line_Position(Reflectance_Sensor_Data);
    }
}


int main(void)
{
    //Ensure that interrupts are disabled during initialization
    DisableInterrupts();

    //Initialize the 48 MHz clock
    Clock_Init48MHz();

    //Initialize the built-in red LED and the RGB LED on the MSP432 microcontroller
    LED1_Init();
    LED2_Init();

    //Initialize the EUSCI_A0_UART module
    EUSCI_A0_UART_Init_Printf();

    //Initialize the 8-channel QTRX Reflectance Sensor Array moduile
    Reflectance_Sensor_Init();

    //Initialize the SysTick timer to generate periiodic interrupts every 1ms
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    //Initialize Timer A1 periodic interrupts every 1ms
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    // Initialize the DC motors
    Motor_Init();

    //Enable the interrupts used by the modules
    EnableInterrupts();


    while(1)
    {

    }
}
