EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:jfng
LIBS:noeud_senseur-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L R R17
U 1 1 58DBF562
P 2950 2350
F 0 "R17" H 3020 2396 50  0000 L CNN
F 1 "10k" H 3020 2305 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2880 2350 50  0001 C CNN
F 3 "" H 2950 2350 50  0001 C CNN
	1    2950 2350
	-1   0    0    -1  
$EndComp
$Comp
L R R16
U 1 1 58DBF60A
P 2950 1750
F 0 "R16" H 3020 1796 50  0000 L CNN
F 1 "10k" H 3020 1705 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2880 1750 50  0001 C CNN
F 3 "" H 2950 1750 50  0001 C CNN
	1    2950 1750
	-1   0    0    -1  
$EndComp
$Comp
L R R15
U 1 1 58DBF63E
P 2400 1775
F 0 "R15" H 2470 1821 50  0000 L CNN
F 1 "100" H 2470 1730 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2330 1775 50  0001 C CNN
F 3 "" H 2400 1775 50  0001 C CNN
	1    2400 1775
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR054
U 1 1 58DBFD32
P 2950 1375
F 0 "#PWR054" H 2950 1225 50  0001 C CNN
F 1 "+3.3V" H 2965 1548 50  0000 C CNN
F 2 "" H 2950 1375 50  0001 C CNN
F 3 "" H 2950 1375 50  0001 C CNN
	1    2950 1375
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR055
U 1 1 58DBFD9A
P 2575 2600
F 0 "#PWR055" H 2575 2350 50  0001 C CNN
F 1 "GND" H 2580 2427 50  0000 C CNN
F 2 "" H 2575 2600 50  0001 C CNN
F 3 "" H 2575 2600 50  0001 C CNN
	1    2575 2600
	-1   0    0    -1  
$EndComp
$Comp
L CP C9
U 1 1 58DC0245
P 2575 2350
F 0 "C9" H 2692 2396 50  0000 L CNN
F 1 "10uF" H 2692 2305 50  0000 L CNN
F 2 "Capacitors_SMD:CP_Elec_4x4.5" H 2613 2200 50  0001 C CNN
F 3 "" H 2575 2350 50  0001 C CNN
	1    2575 2350
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2575 2600 2575 2500
Wire Wire Line
	1575 2050 2950 2050
Wire Wire Line
	2950 1900 2950 2200
Connection ~ 2950 2050
$Comp
L GND #PWR056
U 1 1 58DC135A
P 2950 2600
F 0 "#PWR056" H 2950 2350 50  0001 C CNN
F 1 "GND" H 2955 2427 50  0000 C CNN
F 2 "" H 2950 2600 50  0001 C CNN
F 3 "" H 2950 2600 50  0001 C CNN
	1    2950 2600
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2950 2500 2950 2600
Wire Wire Line
	2950 1600 2950 1375
Connection ~ 2575 2050
Wire Wire Line
	2575 2200 2575 2050
Wire Wire Line
	2400 1925 2400 2050
Connection ~ 2400 2050
Wire Wire Line
	2400 1625 2400 1550
Wire Wire Line
	1800 1550 3425 1550
Connection ~ 2400 1550
Text Label 2050 1550 0    60   ~ 0
TIP
Text Label 2000 2050 0    60   ~ 0
SLEEVE
Text Notes 1950 1850 0    60   ~ 0
Burden\nResistor
Text Notes 850  1025 0    60   ~ 0
Burden Resistor\n30A -> 100 ohm, 3.3V\n100A -> 18 ohm, 3.3V - 33 ohm, 5V
$Comp
L R R21
U 1 1 58DC40AC
P 6475 2350
F 0 "R21" H 6545 2396 50  0000 L CNN
F 1 "10k" H 6545 2305 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 6405 2350 50  0001 C CNN
F 3 "" H 6475 2350 50  0001 C CNN
	1    6475 2350
	-1   0    0    -1  
$EndComp
$Comp
L R R20
U 1 1 58DC40B2
P 6475 1750
F 0 "R20" H 6545 1796 50  0000 L CNN
F 1 "10k" H 6545 1705 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 6405 1750 50  0001 C CNN
F 3 "" H 6475 1750 50  0001 C CNN
	1    6475 1750
	-1   0    0    -1  
$EndComp
$Comp
L R R19
U 1 1 58DC40B8
P 5925 1775
F 0 "R19" H 5995 1821 50  0000 L CNN
F 1 "10k" H 5995 1730 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5855 1775 50  0001 C CNN
F 3 "" H 5925 1775 50  0001 C CNN
	1    5925 1775
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR057
U 1 1 58DC40BE
P 6475 1375
F 0 "#PWR057" H 6475 1225 50  0001 C CNN
F 1 "+3.3V" H 6490 1548 50  0000 C CNN
F 2 "" H 6475 1375 50  0001 C CNN
F 3 "" H 6475 1375 50  0001 C CNN
	1    6475 1375
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR058
U 1 1 58DC40C4
P 6100 2600
F 0 "#PWR058" H 6100 2350 50  0001 C CNN
F 1 "GND" H 6105 2427 50  0000 C CNN
F 2 "" H 6100 2600 50  0001 C CNN
F 3 "" H 6100 2600 50  0001 C CNN
	1    6100 2600
	-1   0    0    -1  
$EndComp
$Comp
L CP C10
U 1 1 58DC40CA
P 6100 2350
F 0 "C10" H 6217 2396 50  0000 L CNN
F 1 "10uF" H 6217 2305 50  0000 L CNN
F 2 "Capacitors_SMD:CP_Elec_4x4.5" H 6138 2200 50  0001 C CNN
F 3 "" H 6100 2350 50  0001 C CNN
	1    6100 2350
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6100 2600 6100 2500
Wire Wire Line
	5175 2050 6475 2050
Wire Wire Line
	6475 1900 6475 2200
Connection ~ 6475 2050
$Comp
L GND #PWR059
U 1 1 58DC40D8
P 6475 2600
F 0 "#PWR059" H 6475 2350 50  0001 C CNN
F 1 "GND" H 6480 2427 50  0000 C CNN
F 2 "" H 6475 2600 50  0001 C CNN
F 3 "" H 6475 2600 50  0001 C CNN
	1    6475 2600
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6475 2500 6475 2600
Wire Wire Line
	6475 1600 6475 1375
Connection ~ 6100 2050
Wire Wire Line
	6100 2200 6100 2050
Wire Wire Line
	5925 1925 5925 2050
Connection ~ 5925 2050
Wire Wire Line
	5925 1625 5925 1550
Wire Wire Line
	5450 1550 6950 1550
Connection ~ 5925 1550
$Comp
L R R18
U 1 1 58DC43F4
P 5300 1550
F 0 "R18" V 5093 1550 50  0000 C CNN
F 1 "100k" V 5184 1550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5230 1550 50  0001 C CNN
F 3 "" H 5300 1550 50  0001 C CNN
	1    5300 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	5150 1550 5000 1550
Text HLabel 3425 1550 2    60   Output ~ 0
CT_OUT
Text HLabel 6950 1550 2    60   Output ~ 0
VT_OUT
$Comp
L JACK_TRS_3PINS CT-1
U 1 1 58E21616
P 1175 1850
F 0 "CT-1" H 1156 2367 50  0000 C CNN
F 1 "JACK_TRS_3PINS" H 1156 2276 50  0000 C CNN
F 2 "caribou:SJ1-3523N" H 1275 1700 50  0001 C CNN
F 3 "" H 1275 1700 50  0001 C CNN
	1    1175 1850
	1    0    0    1   
$EndComp
Wire Wire Line
	1800 1550 1800 1850
Wire Wire Line
	1800 1850 1575 1850
$Comp
L BARREL_JACK VT-1
U 1 1 58E22486
P 4700 1650
F 0 "VT-1" H 4681 1975 50  0000 C CNN
F 1 "BARREL_JACK" H 4681 1884 50  0000 C CNN
F 2 "Connectors_Molex:Molex_KK-6410-03_03x2.54mm_Straight" H 4700 1650 50  0001 C CNN
F 3 "" H 4700 1650 50  0001 C CNN
	1    4700 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5175 2050 5175 1650
Wire Wire Line
	5175 1650 5000 1650
Wire Wire Line
	5000 1750 5175 1750
Connection ~ 5175 1750
NoConn ~ 1575 1650
$EndSCHEMATC
