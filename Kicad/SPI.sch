EESchema Schematic File Version 4
LIBS:noeud_senseur-cache
EELAYER 26 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 2 5
Title "SPI"
Date "2017-06-06"
Rev "1.1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR16
U 1 1 58C15170
P 4875 4275
F 0 "#PWR16" H 1350 -1200 50  0001 C CNN
F 1 "+3.3V" H 5000 4300 50  0000 C CNN
F 2 "" H 1350 -1050 50  0001 C CNN
F 3 "" H 1350 -1050 50  0001 C CNN
	1    4875 4275
	-1   0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR17
U 1 1 58C15188
P 5150 5850
F 0 "#PWR17" H 5200 5900 50  0001 C CNN
F 1 "GND" H 5155 5677 50  0000 C CNN
F 2 "" H -2750 3750 50  0001 C CNN
F 3 "" H -2750 3750 50  0001 C CNN
	1    5150 5850
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:C_Small-RESCUE-noeud_senseur C4
U 1 1 58C151A0
P 5100 4475
F 0 "C4" H 5192 4521 50  0000 L CNN
F 1 "0.1uF" H 5192 4430 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H -875 -2300 50  0001 C CNN
F 3 "" H -875 -2300 50  0001 C CNN
	1    5100 4475
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:C_Small-RESCUE-noeud_senseur C5
U 1 1 58C151A7
P 5475 4475
F 0 "C5" H 5567 4521 50  0000 L CNN
F 1 "0.1uF" H 5567 4430 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H -500 -2300 50  0001 C CNN
F 3 "" H -500 -2300 50  0001 C CNN
	1    5475 4475
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR18
U 1 1 58C151B6
P 5275 4650
F 0 "#PWR18" H -2750 3375 50  0001 C CNN
F 1 "GND" H 5280 4477 50  0000 C CNN
F 2 "" H -2750 3625 50  0001 C CNN
F 3 "" H -2750 3625 50  0001 C CNN
	1    5275 4650
	1    0    0    -1  
$EndComp
$Comp
L jfng:MCP2515 U5
U 1 1 58C161DD
P 6375 2750
F 0 "U5" H 6025 3300 60  0000 C CNN
F 1 "MCP2515" H 6650 2200 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-18W_7.5x11.6mm_Pitch1.27mm" H 2700 -650 60  0001 C CNN
F 3 "" H 2700 -650 60  0000 C CNN
	1    6375 2750
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR20
U 1 1 58C161E4
P 7150 1800
F 0 "#PWR20" H 7200 1850 50  0001 C CNN
F 1 "+3.3V" H 7165 1973 50  0000 C CNN
F 2 "" H -625 -1075 50  0001 C CNN
F 3 "" H -625 -1075 50  0001 C CNN
	1    7150 1800
	1    0    0    -1  
$EndComp
NoConn ~ 7025 3050
NoConn ~ 7025 3150
NoConn ~ 5725 2650
NoConn ~ 5725 2750
NoConn ~ 5725 2850
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR19
U 1 1 58C161F0
P 5725 3150
F 0 "#PWR19" H 5775 3200 50  0001 C CNN
F 1 "GND" H 5730 2977 50  0000 C CNN
F 2 "" H -500 -700 50  0001 C CNN
F 3 "" H -500 -700 50  0001 C CNN
	1    5725 3150
	1    0    0    -1  
$EndComp
NoConn ~ 5725 2550
$Comp
L noeud_senseur-rescue:Crystal_Small-RESCUE-noeud_senseur Y1
U 1 1 58C16201
P 5225 3025
F 0 "Y1" V 5325 2975 50  0000 R CNN
F 1 "16MHz" V 5125 3000 50  0000 R CNN
F 2 "caribou:SMD-2520" H -350 -800 50  0001 C CNN
F 3 "" H -350 -800 50  0001 C CNN
	1    5225 3025
	0    -1   -1   0   
$EndComp
$Comp
L noeud_senseur-rescue:C_Small-RESCUE-noeud_senseur C2
U 1 1 58C16208
P 5050 2775
F 0 "C2" V 4925 2775 50  0000 C CNN
F 1 "10pF" V 5150 2775 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H -550 -900 50  0001 C CNN
F 3 "" H -550 -900 50  0001 C CNN
	1    5050 2775
	0    1    1    0   
$EndComp
$Comp
L noeud_senseur-rescue:C_Small-RESCUE-noeud_senseur C3
U 1 1 58C1620F
P 5050 3275
F 0 "C3" V 4925 3275 50  0000 C CNN
F 1 "10pF" V 5175 3275 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H -550 -400 50  0001 C CNN
F 3 "" H -550 -400 50  0001 C CNN
	1    5050 3275
	0    1    1    0   
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR15
U 1 1 58C16216
P 4825 3425
F 0 "#PWR15" H -500 -950 50  0001 C CNN
F 1 "GND" H 4830 3252 50  0000 C CNN
F 2 "" H -500 -700 50  0001 C CNN
F 3 "" H -500 -700 50  0001 C CNN
	1    4825 3425
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR12
U 1 1 58C1621C
P 4075 1175
F 0 "#PWR12" H 4125 1225 50  0001 C CNN
F 1 "+3.3V" H 4090 1348 50  0000 C CNN
F 2 "" H -3700 -1700 50  0001 C CNN
F 3 "" H -3700 -1700 50  0001 C CNN
	1    4075 1175
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR13
U 1 1 58C16222
P 4075 3025
F 0 "#PWR13" H 4125 3075 50  0001 C CNN
F 1 "GND" H 4080 2852 50  0000 C CNN
F 2 "" H -2150 -825 50  0001 C CNN
F 3 "" H -2150 -825 50  0001 C CNN
	1    4075 3025
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:C_Small-RESCUE-noeud_senseur C6
U 1 1 58C1623F
P 7350 1950
F 0 "C6" V 7450 1900 50  0000 L CNN
F 1 "0.1uF" V 7225 1850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1375 -4825 50  0001 C CNN
F 3 "" H 1375 -4825 50  0001 C CNN
	1    7350 1950
	0    -1   -1   0   
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR21
U 1 1 58C16246
P 7525 2075
F 0 "#PWR21" H 1800 -1250 50  0001 C CNN
F 1 "GND" H 7530 1902 50  0000 C CNN
F 2 "" H 1800 -1000 50  0001 C CNN
F 3 "" H 1800 -1000 50  0001 C CNN
	1    7525 2075
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:C_Small-RESCUE-noeud_senseur C1
U 1 1 58C1624C
P 4400 1300
F 0 "C1" V 4300 1250 50  0000 L CNN
F 1 "0.1uF" V 4525 1225 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H -1575 -5475 50  0001 C CNN
F 3 "" H -1575 -5475 50  0001 C CNN
	1    4400 1300
	0    1    1    0   
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR14
U 1 1 58C16253
P 4650 1425
F 0 "#PWR14" H 4700 1475 50  0001 C CNN
F 1 "GND" H 4655 1252 50  0000 C CNN
F 2 "" H -1575 -2425 50  0001 C CNN
F 3 "" H -1575 -2425 50  0001 C CNN
	1    4650 1425
	1    0    0    -1  
$EndComp
Text HLabel 3475 2450 0    60   BiDi ~ 0
CAN+
Text HLabel 3475 2650 0    60   BiDi ~ 0
CAN-
Text HLabel 7025 2550 2    60   Input ~ 0
MCP2515_CS
Text HLabel 7025 2650 2    60   Output ~ 0
MISO
Text HLabel 7025 2750 2    60   Input ~ 0
MOSI
Text HLabel 7025 2850 2    60   Input ~ 0
SCK
Text HLabel 7025 2950 2    60   Output ~ 0
MCP2515_INT
Text HLabel 5925 4975 0    60   Output ~ 0
MISO
Text HLabel 5925 5175 0    60   Input ~ 0
SCK
Text HLabel 5925 5375 0    60   Input ~ 0
MOSI
Text HLabel 5925 5475 0    60   Input ~ 0
SD_CS
NoConn ~ 6175 4875
NoConn ~ 6175 5575
$Comp
L noeud_senseur-rescue:C_Small-RESCUE-noeud_senseur C12
U 1 1 58EFF980
P 5900 4475
F 0 "C12" H 5992 4521 50  0000 L CNN
F 1 "1uF" H 5992 4430 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H -75 -2300 50  0001 C CNN
F 3 "" H -75 -2300 50  0001 C CNN
	1    5900 4475
	1    0    0    -1  
$EndComp
Text Label 4650 2350 0    60   ~ 0
TXC
Text Label 4650 2450 0    60   ~ 0
RXC
$Comp
L jfng:Micro_SD_Card_no_shield uSD-1
U 1 1 5900282A
P 7075 5275
F 0 "uSD-1" H 7019 4458 50  0000 C CNN
F 1 "Micro_SD_Card_no_shield" H 7019 4549 50  0000 C CNN
F 2 "caribou:114-00841-68" H 8225 5575 50  0001 C CNN
F 3 "" H 7075 5275 50  0001 C CNN
	1    7075 5275
	1    0    0    1   
$EndComp
$Comp
L jfng:SN65HVD232Q U4
U 1 1 5938C27E
P 4075 2550
F 0 "U4" H 4400 2900 50  0000 C CNN
F 1 "SN65HVD232Q" H 3750 2175 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 4075 2050 50  0001 C CIN
F 3 "" H 4075 2550 50  0001 C CNN
	1    4075 2550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6175 4975 5925 4975
Wire Wire Line
	6175 5075 5150 5075
Wire Wire Line
	5925 5175 6175 5175
Wire Wire Line
	4875 5275 6175 5275
Wire Wire Line
	6175 5375 5925 5375
Wire Wire Line
	6175 5475 5925 5475
Wire Wire Line
	5150 5075 5150 5850
Wire Wire Line
	4875 4275 4875 5275
Wire Wire Line
	4875 4350 5900 4350
Wire Wire Line
	5475 4350 5475 4375
Connection ~ 4875 4350
Wire Wire Line
	5100 4375 5100 4350
Connection ~ 5100 4350
Wire Wire Line
	5100 4575 5900 4575
Wire Wire Line
	5275 4575 5275 4650
Connection ~ 5275 4575
Connection ~ 4075 1300
Wire Wire Line
	4300 1300 4075 1300
Wire Wire Line
	7525 1950 7525 2075
Wire Wire Line
	3475 2650 3575 2650
Wire Wire Line
	3475 2450 3575 2450
Wire Wire Line
	4075 1175 4075 2150
Wire Wire Line
	4075 3025 4075 2950
Connection ~ 4825 3275
Wire Wire Line
	4950 3275 4825 3275
Wire Wire Line
	4825 2775 4825 3425
Wire Wire Line
	4950 2775 4825 2775
Connection ~ 5225 3275
Wire Wire Line
	5225 3275 5225 3125
Wire Wire Line
	5150 3275 5550 3275
Connection ~ 5225 2775
Wire Wire Line
	5150 2775 5550 2775
Wire Wire Line
	5225 2775 5225 2925
Wire Wire Line
	5550 3275 5550 3050
Wire Wire Line
	5550 3050 5725 3050
Wire Wire Line
	5550 2775 5550 2950
Wire Wire Line
	5550 2950 5725 2950
Wire Wire Line
	7150 1800 7150 2350
Wire Wire Line
	7150 2350 7025 2350
Wire Wire Line
	5725 2450 4575 2450
Wire Wire Line
	5725 2350 4575 2350
Connection ~ 5475 4575
Wire Wire Line
	5900 4350 5900 4375
Connection ~ 5475 4350
Wire Wire Line
	7025 2450 7775 2450
Wire Wire Line
	7775 2450 7775 2250
Wire Wire Line
	7250 1950 7150 1950
Connection ~ 7150 1950
Wire Wire Line
	7450 1950 7525 1950
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R23
U 1 1 5938D9DF
P 7775 2100
F 0 "R23" H 7845 2146 50  0000 L CNN
F 1 "10k" H 7845 2055 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7705 2100 50  0001 C CNN
F 3 "" H 7775 2100 50  0001 C CNN
	1    7775 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7775 1950 7775 1800
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR22
U 1 1 5938DB22
P 7775 1800
F 0 "#PWR22" H 7825 1850 50  0001 C CNN
F 1 "+3.3V" H 7790 1973 50  0000 C CNN
F 2 "" H 0   -1075 50  0001 C CNN
F 3 "" H 0   -1075 50  0001 C CNN
	1    7775 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 1425 4650 1300
Wire Wire Line
	4650 1300 4500 1300
$EndSCHEMATC
