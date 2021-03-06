EESchema Schematic File Version 4
LIBS:noeud_senseur-cache
EELAYER 26 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 3 5
Title "I2C"
Date "2017-06-06"
Rev "1.1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L jfng:AT24LC256 U6
U 1 1 58C0D972
P 4300 5325
F 0 "U6" H 4125 5575 50  0000 C CNN
F 1 "AT24LC256" H 4575 5025 50  0000 C CNN
F 2 "Housings_SOIC:SOIJ-8_5.3x5.3mm_Pitch1.27mm" H -1250 4050 50  0001 C CIN
F 3 "" H -1250 4050 50  0001 C CNN
	1    4300 5325
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR24
U 1 1 58C0D979
P 4300 5800
F 0 "#PWR24" H -1425 2475 50  0001 C CNN
F 1 "GND" H 4305 5627 50  0000 C CNN
F 2 "" H -1425 2725 50  0001 C CNN
F 3 "" H -1425 2725 50  0001 C CNN
	1    4300 5800
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR23
U 1 1 58C0D97F
P 4300 4625
F 0 "#PWR23" H -1000 1525 50  0001 C CNN
F 1 "+3.3V" H 4315 4798 50  0000 C CNN
F 2 "" H -1000 1675 50  0001 C CNN
F 3 "" H -1000 1675 50  0001 C CNN
	1    4300 4625
	1    0    0    -1  
$EndComp
Text Label 4750 5375 0    60   ~ 0
SCL
Text Label 4750 5225 0    60   ~ 0
SDA
$Comp
L noeud_senseur-rescue:C_Small-RESCUE-noeud_senseur C7
U 1 1 58C0D987
P 5150 4825
F 0 "C7" H 5242 4871 50  0000 L CNN
F 1 "0.1uF" H 5242 4780 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H -825 -1950 50  0001 C CNN
F 3 "" H -825 -1950 50  0001 C CNN
	1    5150 4825
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR27
U 1 1 58C0D98E
P 5150 4950
F 0 "#PWR27" H 5200 5000 50  0001 C CNN
F 1 "GND" H 5155 4777 50  0000 C CNN
F 2 "" H -625 -1975 50  0001 C CNN
F 3 "" H -625 -1975 50  0001 C CNN
	1    5150 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 4925 5150 4950
Connection ~ 4300 4700
Wire Wire Line
	5150 4700 4300 4700
Wire Wire Line
	5150 4725 5150 4700
Connection ~ 4300 5750
Wire Wire Line
	3775 5750 4300 5750
Connection ~ 3775 5525
Wire Wire Line
	3775 5525 3900 5525
Connection ~ 3775 5375
Wire Wire Line
	3775 5375 3900 5375
Connection ~ 3775 5275
Wire Wire Line
	3900 5275 3775 5275
Wire Wire Line
	3775 5175 3775 5750
Wire Wire Line
	3900 5175 3775 5175
Wire Wire Line
	4700 5375 4950 5375
Wire Wire Line
	4700 5225 4950 5225
Wire Wire Line
	4300 5725 4300 5800
Wire Wire Line
	4300 4625 4300 4975
$Comp
L jfng:DS3231 U7
U 1 1 58C1AA49
P 4675 2900
F 0 "U7" H 4950 3300 50  0000 C CNN
F 1 "DS3231" H 4425 2475 50  0000 C CNN
F 2 "caribou:SOIC-16W_7.5x10.3mm_Pitch1.27mm" H -2100 -375 50  0001 L CNN
F 3 "" H -1880 275 50  0001 C CNN
	1    4675 2900
	-1   0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR25
U 1 1 58C1AA50
P 4675 1925
F 0 "#PWR25" H -625 -1175 50  0001 C CNN
F 1 "+3.3V" H 4690 2098 50  0000 C CNN
F 2 "" H -625 -1025 50  0001 C CNN
F 3 "" H -625 -1025 50  0001 C CNN
	1    4675 1925
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR26
U 1 1 58C1AA56
P 4675 3700
F 0 "#PWR26" H -1050 375 50  0001 C CNN
F 1 "GND" H 4680 3527 50  0000 C CNN
F 2 "" H -1050 625 50  0001 C CNN
F 3 "" H -1050 625 50  0001 C CNN
	1    4675 3700
	1    0    0    -1  
$EndComp
Text Label 5200 2800 0    60   ~ 0
SCL
Text Label 5200 2900 0    60   ~ 0
SDA
$Comp
L noeud_senseur-rescue:Battery_Cell-RESCUE-noeud_senseur BT1
U 1 1 58C1AA5E
P 3950 3375
F 0 "BT1" H 4068 3471 50  0000 L CNN
F 1 "Battery_Cell" H 4068 3380 50  0000 L CNN
F 2 "caribou:BS-7" V -2500 -715 50  0001 C CNN
F 3 "" V -2500 -715 50  0001 C CNN
	1    3950 3375
	1    0    0    -1  
$EndComp
Text Label 4125 2175 0    60   ~ 0
SQW
NoConn ~ 4175 2900
NoConn ~ 5175 3100
$Comp
L noeud_senseur-rescue:PWR_FLAG-RESCUE-noeud_senseur #FLG3
U 1 1 58C1AA68
P 3950 3075
F 0 "#FLG3" H -2150 -300 50  0001 C CNN
F 1 "PWR_FLAG" H 3950 3249 50  0000 C CNN
F 2 "" H -2150 -375 50  0001 C CNN
F 3 "" H -2150 -375 50  0001 C CNN
	1    3950 3075
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:C_Small-RESCUE-noeud_senseur C8
U 1 1 58C1AA6E
P 5325 2275
F 0 "C8" H 5417 2321 50  0000 L CNN
F 1 "0.1uF" H 5417 2230 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H -650 -4500 50  0001 C CNN
F 3 "" H -650 -4500 50  0001 C CNN
	1    5325 2275
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR28
U 1 1 58C1AA75
P 5325 2425
F 0 "#PWR28" H -400 -900 50  0001 C CNN
F 1 "GND" H 5330 2252 50  0000 C CNN
F 2 "" H -400 -650 50  0001 C CNN
F 3 "" H -400 -650 50  0001 C CNN
	1    5325 2425
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5325 2375 5325 2425
Connection ~ 4675 2150
Wire Wire Line
	5325 2150 4675 2150
Wire Wire Line
	5325 2175 5325 2150
Connection ~ 3950 3100
Connection ~ 4100 2800
Wire Wire Line
	4100 2175 4350 2175
Wire Wire Line
	4100 2800 4100 2175
Connection ~ 4675 3575
Wire Wire Line
	3950 3575 4675 3575
Wire Wire Line
	3950 3475 3950 3575
Wire Wire Line
	3950 3100 4175 3100
Wire Wire Line
	3950 3075 3950 3175
Wire Wire Line
	5175 2900 5450 2900
Wire Wire Line
	5175 2800 5450 2800
Wire Wire Line
	4675 3400 4675 3700
Wire Wire Line
	4675 1925 4675 2400
Wire Wire Line
	3525 2800 4175 2800
Text HLabel 3525 2800 0    60   Output ~ 0
INT_SQW
Text HLabel 5450 2800 2    60   Input ~ 0
I2C_SCL
Text HLabel 5450 2900 2    60   BiDi ~ 0
I2C_SDA
Text HLabel 4950 5375 2    60   Input ~ 0
I2C_SCL
Text HLabel 4950 5225 2    60   BiDi ~ 0
I2C_SDA
$EndSCHEMATC
