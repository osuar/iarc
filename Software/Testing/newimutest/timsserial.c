#include <stdio.h>
#include <stdlib.h>
    
void binaryfunk2(unsigned char *bits);
void floatToBinaryPrint();
char * floatToBinary(float input);
int floatToChar1(char* input);
int floatToChar2(char* input);

int main(){
        float test = .5;
        char * bits = floatToBinary(test);
        floatToChar1(bits);
        //printf("%i\n",floatToChar2(bits));
        
        
        
        return 0;
}

int floatToChar2(char * input){
        int exp=-127, half = 64, i, value=1;
        for(i=2;i<9;i++){
                if(input[i] & 01){
                        exp += half;
                }
                half /=2;
        }
        //printf("Exp: %i\n", exp);
        if(exp >= 0 || exp < -7){
                printf("Too Large or Small\n");
                return -1;
        }
        /*  -1 64 96 80 
                -2 32 48 40
                -3 16 24 20
                -4 8 12 10
                -5 4 6  5
                -6 2 3  2.5
                -7 1 1.5 1.25
        */
        half =96;
        value = 64;
        for(i=-1; i> exp; i--){
                half /=2;
                value /= 2;
        }
        half = (half-((half/6)*5))*2;
        //printf("value: %i, half: %i\n", value, half);
        for(i=9; i<16; i++){
                if(input[i] & 1){
                        value += half;
                }
                half /=2;
        }
        if(input[0] & 01){
        value *= -1;
        }
        printf("%i\n",value);
        return value;        
}

int floatToChar1(char * input){
        int exp=-127, half = 128, i, value=1;
        for(i=1;i<9;i++){
                if(input[i] & 01){
                        exp += half;
                }
                half /=2;
        }
        //printf("Exp: %i\n", exp);
        if(exp > 1 || exp < -7){
                printf("Too Large or Small\n");
                return -1;
        }
        if(input[0] & 1){
                printf("Negative!\n");
                return -1;
        }
        /*  -1 64 96 80 
                -2 32 48 40
                -3 16 24 20
                -4 8 12 10
                -5 4 6  5
                -6 2 3  2.5
                -7 1 1.5 1.25
        */
        half =96;
        value = 64;
        for(i=-1; i> exp; i--){
                half /=2;
                value /= 2;
        }
        half = (half-((half/6)*5))*2;
        //printf("value: %i, half: %i\n", value, half);
        for(i=9; i<16; i++){
                if(input[i] & 1){
                        value += half;
                }
                half /=2;
        }
        if(exp >= 0){
                value *=2;
        }
        printf("%i\n",value);
        return value;        
}



char * floatToBinary(float input){
        int i;
        char * bits = malloc( sizeof(char) * 32);
        union{
                char imu[4];
                float output;
        }u;
        u.output = input;
        for(i=0; i<32; i++){
                if(u.imu[i/8] & 0x80){
                        bits[i] = 1;
                }
                else{
                        bits[i] = 0;
                }
                u.imu[i/8] = u.imu[i/8]<<1;
        }
        return bits;
}

void floatToBinaryPrint(){
        int i;
        union{
                char imu[4];
                float output;
        }u;
                printf("Enter a float Value: ");
                scanf ("%f",&u.output);
        for(i=0; i<32; i++){
                if(u.imu[i/8] & 0x80){
                        printf("1");
                }
                else{
                        printf("0");
                }
                u.imu[i/8] = u.imu[i/8]<<1;
        }
        printf("\n");
}



void binaryfunk2(unsigned char *bits){                
        unsigned char sign[1];        
        unsigned char exponent[8];
        unsigned char mantissa[23];
        int i;
        unsigned char usefulBitsOneToOne[8];
        unsigned char usefulBitsZeroToTwo[8];
        sign[0] = bits[0] & 0x80;                                        //fills sign
        
        for(i = 0; i < 7; i++){                                                //fills exponent
                exponent[i] = bits[0] & (2 ^ (6 - i)); 
                printf("%d", (2 ^ (6-i)));
        }
        exponent[7] = bits[1] & 0x80;
        
        for(i = 0; i < 7; i++){                                                //fills mantissa
                mantissa[i] = bits[1] & (2 ^ (6 - i));
        }
        for(i = 7; i < 15; i++){
                mantissa[i] = bits[2] & (2 ^ (14 - i));
        }
        for(i = 15; i < 23; i++){
                mantissa[i] = bits[3] & (2 ^ (22 - i));
        }
        
        if(!(exponent[0] == 0 && exponent[1] == exponent[2] == exponent[3] == exponent[4]
== 1)){
                printf("Too Large or Small");
                return;
        }
        //for Zero to Two first bit represents wheather it is in the range 1-2 or 0-1.
        usefulBitsZeroToTwo[0] = (exponent[7] & exponent[6] & exponent[5]);
        for(i=22; i>=15; i--)
        usefulBitsZeroToTwo[i-15] = mantissa[i];
        
        //for -1 to 1 first bit represents wheather its sign.
        usefulBitsOneToOne[0] = sign[0];
        for(i=22; i>=15; i--)
        usefulBitsOneToOne[i-15] = mantissa[i];

                
                
                //prints out everything.
        printf("signed bit: %s\n", sign);
        printf("exponent: %s\n", exponent);
        printf("mantissa: %s\n", mantissa);

        printf("Entire 32bit binary: %s%s%s\n", sign, exponent, mantissa);
        printf("Useful Bits from Zero to Two: %s\n", usefulBitsZeroToTwo);
        printf("Useful Bits from -1 to 1: %s\n", usefulBitsOneToOne);
        
}

