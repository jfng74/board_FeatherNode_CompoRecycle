#ifndef NTC_CONFIG_H
#define NTC_CONFIG_H

// NTC
// which analog pin to connect
#define THERMISTORPIN_A0 A0
#define THERMISTORPIN_A1 A1
#define THERMISTORPIN_A2 A2
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

#endif