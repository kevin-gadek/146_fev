/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "scheduler_task.hpp"
#include "event_groups.h"
#include "storage.hpp"
#include "lpc_pwm.hpp"
#include "time.h"
#include "string.h"
#include <io.hpp>
#include <stdio.h>
#include <queue.h>
#include "adc0.h"


//class pwm_task : public scheduler_task{
//public:
//	pwm_task(uint8_t priority) : scheduler_task("pwm_task", 2000, priority){}
//	bool init(void){

//		return true;
//	}
//
//	bool run(void *p){
//
//		return true;
//	}
//
//
//};


/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */
		//P2.4, 50Hz frequency
PWM motor1(PWM::pwm5, 50);
PWM motor2(PWM::pwm4, 50);

void turn_left(){

	LPC_GPIO0->FIOSET |= (1 << 0);
	LPC_GPIO0->FIOCLR |= (1 << 1);
	motor1.set(100);
	motor2.set(100);

}

void turn_right(){

	LPC_GPIO0->FIOSET |= (1 << 1);
	LPC_GPIO0->FIOCLR |= (1 << 0);
	motor1.set(100);
	motor2.set(100);

}

void go_forward(){

	LPC_GPIO0->FIOSET |= (1 << 0);
	LPC_GPIO0->FIOSET |= (1 << 1);
	motor1.set(100);
	motor2.set(100);

}

void go_reverse(){

	LPC_GPIO0->FIOCLR |= (1 << 0);
	LPC_GPIO0->FIOCLR |= (1 << 1);
	motor1.set(100);
	motor2.set(100);

}

void motor_stop(){
	motor1.set(0);
	motor2.set(0);
}
//0 = idle( searching in place), 1 = traveling to flame, 2 = pumping water
int state = 0;
int count = 1000;
class pwm_task : public scheduler_task{
public:
	pwm_task(uint8_t priority) : scheduler_task("pwm_task", 2000, priority){}
	bool init(void){
		//PINSEL GPIO
		//DIR1 and DIR2
		LPC_PINCON->PINSEL0 &= ~(3 << 0);
		LPC_PINCON->PINSEL0 &= ~(3 << 2);
		//P1.20
		LPC_PINCON->PINSEL3 &= ~(3 << 8);

		//LPC_PINCON->PINSEL3 &= ~(3 << 12);
		//disable pull-up and pull-down resistors for P1.20
		LPC_PINCON->PINMODE3 |= (2 << 8);
		LPC_PINCON->PINMODE_OD1 |= (1 << 20);
		//LPC_PINCON->PINMODE1 |= (1 << 9);
		//LPC_PINCON->PINMODE_OD1
		//GPIO Outputs
		LPC_GPIO0->FIODIR |= (1 << 0);
		LPC_GPIO0->FIODIR |= (1 << 1);
		LPC_GPIO1->FIODIR |= (1 << 20);
		LPC_GPIO1->FIODIR |= (1 << 22);
		LPC_PINCON->PINSEL2 &= (3 << 18);
		LPC_GPIO1->FIODIR |= (0 << 9);

		return true;
	}

	bool run(void *p){
		LPC_GPIO1->FIOSET |= (1 << 20);
		while(state == 0){
			//LPC_GPIO1->FIOCLR |= (1 << 20);
			turn_right();
			vTaskDelay(350);
			motor_stop();
			vTaskDelay(500);
		}
		while(state == 1){
			go_forward();
			vTaskDelay(1000);
		}
		while(state == 2){
			motor_stop();
			LPC_GPIO1->FIOSET |= (1 << 20);
			vTaskDelay(5000);
		}


		return true;
	}
};

class flame_task : public scheduler_task{
public:
	flame_task(uint8_t priority) : scheduler_task("flame_task", 2000, priority){}
	bool init(void){
		LPC_PINCON->PINSEL1 |= (1 << 20);
		//LPC_PINCON->PINSEL3 |=  (3 << 28);
		return true;
	}

	bool run(void *p){
		while(1){
			reading = adc0_get_reading(3);
			//reading = adc0_get_reading(4);
			printf("ADC: %i, State: %i\n", reading, state);
			if(reading > 3600){
				state = 0;
			}
			if(reading < 2000 && state == 0){
				state = 1;
			}
			if(reading < 350){
				state = 2;
			}
			if(reading > 750 && state == 2){
				state = 1;
			}
			vTaskDelay(100);
		}
		return true;
	}
private:
	int i = 0, reading = 0;
	//int readings[50];
};

int main(void)
{
    /**
     * A few basic tasks for this bare-bone system :
     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
     *      2.  Remote task allows you to use remote control to interact with the board.
     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
     *
     * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
     * such that it can save remote control codes to non-volatile memory.  IR remote
     * control codes can be learned by typing the "learn" terminal command.
     */
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));
    scheduler_add_task(new flame_task(PRIORITY_HIGH));
   scheduler_add_task(new pwm_task(PRIORITY_HIGH));
   // scheduler_add_task(new acc_task(PRIORITY_HIGH));


    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
    #if 0
    scheduler_add_task(new periodicSchedulerTask());
    #endif

    /* The task for the IR receiver */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
    #if 0
        Uart3 &u3 = Uart3::getInstance();
        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif

    scheduler_start(); ///< This shouldn't return
    return -1;
}
