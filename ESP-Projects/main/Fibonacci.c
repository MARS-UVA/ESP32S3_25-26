#include <stdio.h>
#include "freertos/FreeRTOS.h"

int name(int value) {
    int value2 = value;
    return value2;
}

int fibonacci(int n) {
    vTaskDelay(1);
    if (n < 0) {
        return 0;
    }
    else if (n == 0) {
        return 0;
    }
    else if (n == 1) {
        return 1;
    }
    
    else {
        int a = 0;
        int b = 1;
        int new = a+b;
        for (int i=2; i<=n; i++) {
            new = a+b;
            a = b;
            b = new;
        }
        return b;
    }
    

    
   
}

void app_main(void)
{
int n = 10;
printf("Number of terms: %d\n", n);
int result = fibonacci(n);
printf("Fibonacci number: %d\n", result);
}