#include "MainControlLoop.hpp"
#include <Arduino.h>

void setup(){
    MainControlLoop mcl = MainControlLoop();

    //delay for 2 hours
    //delay(7200000);
    delay(2000);
    while(true){
        mcl.execute();
    }
}

void loop(){}


