#include <avr/io.h>
#include "avr_compiler.h"

#include "filter.h"

void sweep(int *stuff) // takes a pointer to structure containg 3 dimmensional array, sweeps back and forth

{                         // moving servo first up, then down, and reading for each value
                          // reads a "LENGHT" number of sweeps before returning
        int j = 0;
        move_servo(10);

        for(j = 0; j < STEPS; j++)
        {
                move_servo(10 + j); 
                _delay_ms(50);
                stuff[j] = readadc(0); 
                //sprintf(uartBuffer, "%d %d \n" , set(stuff[j]), j);
                //sendstring(uartBuffer);
        }

        for (; j >= 0; j--)
        {
                move_servo(10 + j);
                _delay_ms(50);
                stuff[j] = (stuff[j] + readadc(0))/2;
                stuff[j] = set(stuff[j]);
                //sprintf(uartBuffer, "%d %d \n" , set(stuff[j]), j);
                //sendstring(uartBuffer);
        }
}



int set(int raw)
{
        int news;
        // this array contains the corresponding voltage values for the corresponding distance in voltage X 100 values
        // the ith term of the array is the i X 10 cm mark
        int scale[16] = {1, 275, 260, 200, 160, 125, 115, 90, 80, 70, 65, 60, 55, 50, 47, 45};

        // this is the scaling of the read ADC value into voltages, this should be raw / 205, but since the voltage is
        // scaled by 100, this yields the correct numbers
        news = raw/2.05;
        int i = 1;

        // if the value is too large, it indicates the distance to be at 10cm, the closest mark
        if(news >= 275)
        {
                return 10;
        }
        // if the value is too small, it indicates the distance to be at 180cm, the furthest mark
        if(news <= 45)
        {
                return 180;
        }
        // this section of code is responsible determing which two reference points the voltage is between, and take a linear
        // approxmation of the line between those points, and calculating a distance based on those values
        for(i = 1; i < 16; i++)
        {
                if(news == scale[i])
                        return 10*i;
                if((news < scale[i]) && (news > scale[i+1]))
                {
                        return(10*(scale[i] - news)/(scale[i] - scale[i+1])+ 10*i);
                }
        }
        // return failure
        return(0);
}

void derivitive(int* dummy, int* deriv)
{
        int i;
        deriv[0] = dummy[0];
        for(i = 1; i < STEPS; i++)
        {
                deriv[i] = dummy[i] - dummy[i -1];
        }
}

void average(coord* data, coord* dummy, int c)
{
        int i;
        char buffer[20];
        for(i = 0; i < STEPS; i++)
                {
                        data[i].x = (data[i].x*c + dummy[i].x)/(c + 1);
			data[i].y = (data[i].y*c + dummy[i].y)/(c + 1);
                        sprintf(buffer,"%d %d \n", data[i].x, data[i].y);
                        sendstring(buffer);
                }
}
void smooth(int* dummy, int * deriv)
{
        int i;
        for(i = 0; i < LENGTH; i ++)
                {
                        // if high rate of change, check the following rates of change in the cycle, if also large, mark as 
                        // uncertain zone, aka, high
                        // if hight rate of change, followed by low rate of change, leave alone
                        if ((deriv[i] > 50)||(deriv[i] < -50))
                        {
                                if((deriv[i+1] >  50) || (deriv[i+1]) < -50)
                                        dummy[i] = 180;
                        }
                }
}



