EESchema Schematic File Version 4
LIBS:noeud_senseur-cache
EELAYER 26 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 4 5
Title "Connect"
Date "2017-06-06"
Rev "1.1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R7
U 1 1 58C21546
P 5500 1125
F 0 "R7" H 5570 1171 50  0000 L CNN
F 1 "10k" H 5570 1080 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V -20 -2975 50  0001 C CNN
F 3 "" H 50  -2975 50  0001 C CNN
	1    5500 1125
	-1   0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR42
U 1 1 58C2154D
P 5500 1575
F 0 "#PWR42" H -225 -1750 50  0001 C CNN
F 1 "GND" H 5505 1402 50  0000 C CNN
F 2 "" H -225 -1500 50  0001 C CNN
F 3 "" H -225 -1500 50  0001 C CNN
	1    5500 1575
	-1   0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R13
U 1 1 58C21559
P 5500 2325
F 0 "R13" H 5570 2371 50  0000 L CNN
F 1 "10k" H 5570 2280 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V -20 -1775 50  0001 C CNN
F 3 "" H 50  -1775 50  0001 C CNN
	1    5500 2325
	-1   0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR44
U 1 1 58C21560
P 5500 2825
F 0 "#PWR44" H -225 -500 50  0001 C CNN
F 1 "GND" H 5505 2652 50  0000 C CNN
F 2 "" H -225 -250 50  0001 C CNN
F 3 "" H -225 -250 50  0001 C CNN
	1    5500 2825
	-1   0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR39
U 1 1 58C21566
P 5450 3800
F 0 "#PWR39" H 150 700 50  0001 C CNN
F 1 "+3.3V" H 5465 3973 50  0000 C CNN
F 2 "" H 150 850 50  0001 C CNN
F 3 "" H 150 850 50  0001 C CNN
	1    5450 3800
	-1   0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R8
U 1 1 58C2156C
P 5450 4200
F 0 "R8" H 5520 4246 50  0000 L CNN
F 1 "10k" H 5520 4155 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V -70 100 50  0001 C CNN
F 3 "" H 0   100 50  0001 C CNN
	1    5450 4200
	-1   0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR40
U 1 1 58C21573
P 5450 4450
F 0 "#PWR40" H -275 1125 50  0001 C CNN
F 1 "GND" H 5455 4277 50  0000 C CNN
F 2 "" H -275 1375 50  0001 C CNN
F 3 "" H -275 1375 50  0001 C CNN
	1    5450 4450
	-1   0    0    -1  
$EndComp
Text Label 5300 1375 2    60   ~ 0
A0
Text Label 5275 2625 2    60   ~ 0
A1
Text Label 5250 4000 2    60   ~ 0
A2
$Comp
L noeud_senseur-rescue:CONN_01X02-RESCUE-noeud_senseur NTC-1
U 1 1 58C2157C
P 7100 1450
F 0 "NTC-1" H 7178 1491 50  0000 L CNN
F 1 "NTC_1" H 7178 1400 50  0000 L CNN
F 2 "Connectors_Molex:Molex_KK-6410-02_02x2.54mm_Straight" H 3875 -3575 50  0001 C CNN
F 3 "" H 3875 -3575 50  0001 C CNN
	1    7100 1450
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:CONN_01X02-RESCUE-noeud_senseur NTC-2
U 1 1 58C21583
P 7100 2675
F 0 "NTC-2" H 7178 2716 50  0000 L CNN
F 1 "NTC_2" H 7178 2625 50  0000 L CNN
F 2 "Connectors_Molex:Molex_KK-6410-02_02x2.54mm_Straight" H 3875 -2350 50  0001 C CNN
F 3 "" H 3875 -2350 50  0001 C CNN
	1    7100 2675
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:MMBT3904-RESCUE-noeud_senseur Q4
U 1 1 58C21591
P 2125 1475
F 0 "Q4" H 2316 1521 50  0000 L CNN
F 1 "MMBT3904" H 2316 1430 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H -300 -1350 50  0001 L CIN
F 3 "" H -500 -1275 50  0001 L CNN
	1    2125 1475
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R9
U 1 1 58C21598
P 1575 1475
F 0 "R9" V 1700 1475 50  0000 L CNN
F 1 "1k" V 1475 1450 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V -170 25  50  0001 C CNN
F 3 "" H -100 25  50  0001 C CNN
	1    1575 1475
	0    1    1    0   
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR32
U 1 1 58C2159F
P 2225 1950
F 0 "#PWR32" H 2275 2000 50  0001 C CNN
F 1 "GND" H 2230 1777 50  0000 C CNN
F 2 "" H -4000 -1900 50  0001 C CNN
F 3 "" H -4000 -1900 50  0001 C CNN
	1    2225 1950
	1    0    0    -1  
$EndComp
Text Label 1350 1475 0    60   ~ 0
A4
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R11
U 1 1 58C215A6
P 1825 1700
F 0 "R11" H 1900 1700 50  0000 L CNN
F 1 "10k" H 1625 1725 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 80  250 50  0001 C CNN
F 3 "" H 150 250 50  0001 C CNN
	1    1825 1700
	-1   0    0    1   
$EndComp
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR31
U 1 1 58C215AD
P 2225 1000
F 0 "#PWR31" H -3075 -2100 50  0001 C CNN
F 1 "+3.3V" H 2240 1173 50  0000 C CNN
F 2 "" H -3075 -1950 50  0001 C CNN
F 3 "" H -3075 -1950 50  0001 C CNN
	1    2225 1000
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:CONN_01X02-RESCUE-noeud_senseur SSR-1
U 1 1 58C215B3
P 3825 1150
F 0 "SSR-1" H 3903 1191 50  0000 L CNN
F 1 "SSR_1" H 3903 1100 50  0000 L CNN
F 2 "Connectors_Molex:Molex_KK-6410-02_02x2.54mm_Straight" H 600 -3875 50  0001 C CNN
F 3 "" H 600 -3875 50  0001 C CNN
	1    3825 1150
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:MMBT3904-RESCUE-noeud_senseur Q5
U 1 1 58C215BA
P 2150 2875
F 0 "Q5" H 2341 2921 50  0000 L CNN
F 1 "MMBT3904" H 2341 2830 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H -275 50  50  0001 L CIN
F 3 "" H -475 125 50  0001 L CNN
	1    2150 2875
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R10
U 1 1 58C215C1
P 1600 2875
F 0 "R10" V 1725 2875 50  0000 L CNN
F 1 "1k" V 1500 2850 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V -145 1425 50  0001 C CNN
F 3 "" H -75 1425 50  0001 C CNN
	1    1600 2875
	0    1    1    0   
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR34
U 1 1 58C215C8
P 2250 3350
F 0 "#PWR34" H 2300 3400 50  0001 C CNN
F 1 "GND" H 2255 3177 50  0000 C CNN
F 2 "" H -3975 -500 50  0001 C CNN
F 3 "" H -3975 -500 50  0001 C CNN
	1    2250 3350
	1    0    0    -1  
$EndComp
Text Label 1375 2875 0    60   ~ 0
A5
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R12
U 1 1 58C215CF
P 1850 3100
F 0 "R12" H 1925 3100 50  0000 L CNN
F 1 "10k" H 1650 3125 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 105 1650 50  0001 C CNN
F 3 "" H 175 1650 50  0001 C CNN
	1    1850 3100
	-1   0    0    1   
$EndComp
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR33
U 1 1 58C215D6
P 2250 2400
F 0 "#PWR33" H -3050 -700 50  0001 C CNN
F 1 "+3.3V" H 2265 2573 50  0000 C CNN
F 2 "" H -3050 -550 50  0001 C CNN
F 3 "" H -3050 -550 50  0001 C CNN
	1    2250 2400
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:CONN_01X02-RESCUE-noeud_senseur SSR-2
U 1 1 58C215DC
P 3825 2550
F 0 "SSR-2" H 3903 2591 50  0000 L CNN
F 1 "SSR_2" H 3903 2500 50  0000 L CNN
F 2 "Connectors_Molex:Molex_KK-6410-02_02x2.54mm_Straight" H 600 -2475 50  0001 C CNN
F 3 "" H 600 -2475 50  0001 C CNN
	1    3825 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 2675 2250 2600
Wire Wire Line
	2250 2400 2250 2500
Connection ~ 2250 3325
Wire Wire Line
	1850 3325 2250 3325
Wire Wire Line
	1850 3250 1850 3325
Connection ~ 1850 2875
Wire Wire Line
	1850 2950 1850 2875
Wire Wire Line
	2250 3075 2250 3350
Wire Wire Line
	1450 2875 1175 2875
Wire Wire Line
	1750 2875 1950 2875
Wire Wire Line
	2225 1275 2225 1200
Wire Wire Line
	2225 1000 2225 1100
Connection ~ 2225 1925
Wire Wire Line
	1825 1925 2225 1925
Wire Wire Line
	1825 1850 1825 1925
Connection ~ 1825 1475
Wire Wire Line
	1825 1550 1825 1475
Wire Wire Line
	2225 1675 2225 1950
Wire Wire Line
	1425 1475 1150 1475
Wire Wire Line
	1725 1475 1925 1475
Connection ~ 5450 4000
Wire Wire Line
	5450 3900 5450 3800
Wire Wire Line
	5450 4350 5450 4450
Wire Wire Line
	5450 4000 5450 4050
Wire Wire Line
	5100 4000 5875 4000
Wire Wire Line
	5500 2625 5500 2475
Wire Wire Line
	5500 2725 5500 2825
Wire Wire Line
	5500 1475 5950 1475
Wire Wire Line
	5500 1475 5500 1575
$Comp
L noeud_senseur-rescue:CONN_01X04-RESCUE-noeud_senseur I2C-1
U 1 1 58C2216C
P 10075 3225
F 0 "I2C-1" H 10153 3266 50  0000 L CNN
F 1 "I2C" H 10153 3175 50  0000 L CNN
F 2 "caribou:0436500417" H 175 -925 50  0001 C CNN
F 3 "" H 175 -925 50  0001 C CNN
	1    10075 3225
	1    0    0    -1  
$EndComp
Text Label 8500 3375 0    60   ~ 0
SCL
Text Label 8500 3275 0    60   ~ 0
SDA
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR29
U 1 1 58C22181
P 1625 6125
F 0 "#PWR29" H -4100 2800 50  0001 C CNN
F 1 "GND" H 1630 5952 50  0000 C CNN
F 2 "" H -4100 3050 50  0001 C CNN
F 3 "" H -4100 3050 50  0001 C CNN
	1    1625 6125
	1    0    0    -1  
$EndComp
Text Label 1675 6025 0    60   ~ 0
TX
Text Label 1675 5925 0    60   ~ 0
RX
$Comp
L noeud_senseur-rescue:CONN_01X03-RESCUE-noeud_senseur SER-1
U 1 1 58C22189
P 2025 6025
F 0 "SER-1" H 2102 6066 50  0000 L CNN
F 1 "SERIAL" H 2102 5975 50  0000 L CNN
F 2 "Connectors_Molex:Molex_KK-6410-03_03x2.54mm_Straight" H 625 -1275 50  0001 C CNN
F 3 "" H 625 -1275 50  0001 C CNN
	1    2025 6025
	1    0    0    -1  
$EndComp
Text Label 1700 4425 0    60   ~ 0
VBAT_FEATHER
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR35
U 1 1 58C22198
P 2325 4850
F 0 "#PWR35" H -3400 1525 50  0001 C CNN
F 1 "GND" H 2330 4677 50  0000 C CNN
F 2 "" H -3400 1775 50  0001 C CNN
F 3 "" H -3400 1775 50  0001 C CNN
	1    2325 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2325 4525 2675 4525
Wire Wire Line
	2325 4850 2325 4525
Wire Wire Line
	1225 4425 2675 4425
Wire Wire Line
	1825 5925 1625 5925
Wire Wire Line
	1825 6025 1625 6025
Wire Wire Line
	1825 6125 1625 6125
Wire Wire Line
	8025 2700 8025 3175
Wire Wire Line
	8500 2700 8500 3075
$Comp
L noeud_senseur-rescue:CONN_01X02-RESCUE-noeud_senseur CAN-1
U 1 1 58C31FB1
P 9500 950
F 0 "CAN-1" H 9578 991 50  0000 L CNN
F 1 "CAN_1" H 9578 900 50  0000 L CNN
F 2 "caribou:0436500219" H 6275 -4075 50  0001 C CNN
F 3 "" H 6275 -4075 50  0001 C CNN
	1    9500 950 
	1    0    0    -1  
$EndComp
$Comp
L jfng:CAN_NODE_JUNCTION CNJ-1
U 1 1 58C31FB8
P 9025 1450
F 0 "CNJ-1" H 9253 1404 50  0000 L CNN
F 1 "CAN_NODE_JUNCTION" H 9253 1495 50  0000 L CNN
F 2 "caribou:CAN_NODE_JONCTION" H 5675 -1500 50  0001 C CNN
F 3 "" H 5675 -1500 50  0001 C CNN
	1    9025 1450
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:CONN_01X02-RESCUE-noeud_senseur CAN-2
U 1 1 58C31FBF
P 9500 1950
F 0 "CAN-2" H 9578 1991 50  0000 L CNN
F 1 "CAN_2" H 9578 1900 50  0000 L CNN
F 2 "caribou:0436500219" H 6275 -3075 50  0001 C CNN
F 3 "" H 6275 -3075 50  0001 C CNN
	1    9500 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 2000 9150 1750
Wire Wire Line
	9300 2000 9150 2000
Wire Wire Line
	9050 1900 9050 1750
Wire Wire Line
	9300 1900 9050 1900
Wire Wire Line
	9050 900  9300 900 
Wire Wire Line
	9050 1150 9050 900 
Wire Wire Line
	9150 1000 9300 1000
Wire Wire Line
	9150 1150 9150 1000
Text HLabel 8825 1400 0    60   BiDi ~ 0
CAN+
Text HLabel 8825 1500 0    60   BiDi ~ 0
CAN-
Text Label 9050 1150 2    60   ~ 0
CAN+
Text Label 9050 1800 2    60   ~ 0
CAN+
Text Label 9150 1800 0    60   ~ 0
CAN-
Text HLabel 1150 1475 0    60   Input ~ 0
SSR_1
Text HLabel 1175 2875 0    60   Input ~ 0
SSR_2
Text HLabel 5100 4000 0    60   Output ~ 0
PTC
Text HLabel 4650 6100 0    60   Output ~ 0
MOISTURE
Text HLabel 1625 5925 0    60   Input ~ 0
RX
Text HLabel 1625 6025 0    60   Output ~ 0
TX
Text HLabel 1225 4425 0    60   UnSpc ~ 0
VBAT
$Comp
L noeud_senseur-rescue:L_Core_Ferrite-RESCUE-noeud_senseur L6
U 1 1 58DDAE55
P 3150 1300
F 0 "L6" V 3275 1275 50  0000 C CNN
F 1 "L_Core_Ferrite" V 3063 1300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 3150 1300 50  0001 C CNN
F 3 "" H 3150 1300 50  0001 C CNN
	1    3150 1300
	0    1    1    0   
$EndComp
$Comp
L noeud_senseur-rescue:L_Core_Ferrite-RESCUE-noeud_senseur L5
U 1 1 58DDAEDD
P 3125 975
F 0 "L5" V 3250 975 50  0000 C CNN
F 1 "L_Core_Ferrite" V 3038 975 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 3125 975 50  0001 C CNN
F 3 "" H 3125 975 50  0001 C CNN
	1    3125 975 
	0    1    1    0   
$EndComp
Wire Wire Line
	2225 1100 2825 1100
Wire Wire Line
	3450 1100 3625 1100
Wire Wire Line
	2225 1200 2825 1200
Wire Wire Line
	2825 1200 2825 1300
Wire Wire Line
	2825 1300 3000 1300
Wire Wire Line
	3300 1300 3450 1300
Wire Wire Line
	3450 1300 3450 1200
Wire Wire Line
	3450 1200 3625 1200
$Comp
L noeud_senseur-rescue:L_Core_Ferrite-RESCUE-noeud_senseur L8
U 1 1 58DDB704
P 3175 2825
F 0 "L8" V 3275 2800 50  0000 C CNN
F 1 "L_Core_Ferrite" V 3088 2825 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 3175 2825 50  0001 C CNN
F 3 "" H 3175 2825 50  0001 C CNN
	1    3175 2825
	0    1    1    0   
$EndComp
$Comp
L noeud_senseur-rescue:L_Core_Ferrite-RESCUE-noeud_senseur L7
U 1 1 58DDB7C0
P 3150 2375
F 0 "L7" V 3275 2375 50  0000 C CNN
F 1 "L_Core_Ferrite" V 3063 2375 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 3150 2375 50  0001 C CNN
F 3 "" H 3150 2375 50  0001 C CNN
	1    3150 2375
	0    1    1    0   
$EndComp
Wire Wire Line
	2250 2500 2875 2500
Wire Wire Line
	2875 2500 2875 2375
Wire Wire Line
	2875 2375 3000 2375
Wire Wire Line
	3300 2375 3475 2375
Wire Wire Line
	3475 2375 3475 2500
Wire Wire Line
	3475 2500 3625 2500
Wire Wire Line
	2250 2600 2875 2600
Wire Wire Line
	2875 2600 2875 2825
Wire Wire Line
	2875 2825 3025 2825
Wire Wire Line
	3325 2825 3475 2825
Wire Wire Line
	3475 2825 3475 2600
Wire Wire Line
	3475 2600 3625 2600
$Comp
L noeud_senseur-rescue:L_Core_Ferrite-RESCUE-noeud_senseur L13
U 1 1 58DDE0D8
P 9400 2675
F 0 "L13" V 9500 2675 50  0000 C CNN
F 1 "L_Core_Ferrite" V 9313 2675 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9400 2675 50  0001 C CNN
F 3 "" H 9400 2675 50  0001 C CNN
	1    9400 2675
	0    1    1    0   
$EndComp
$Comp
L noeud_senseur-rescue:L_Core_Ferrite-RESCUE-noeud_senseur L14
U 1 1 58DDE29F
P 9400 2975
F 0 "L14" V 9500 2975 50  0000 C CNN
F 1 "L_Core_Ferrite" V 9313 2975 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9400 2975 50  0001 C CNN
F 3 "" H 9400 2975 50  0001 C CNN
	1    9400 2975
	0    1    1    0   
$EndComp
$Comp
L noeud_senseur-rescue:L_Core_Ferrite-RESCUE-noeud_senseur L15
U 1 1 58DDE2FD
P 9400 3275
F 0 "L15" V 9500 3275 50  0000 C CNN
F 1 "L_Core_Ferrite" V 9313 3275 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9400 3275 50  0001 C CNN
F 3 "" H 9400 3275 50  0001 C CNN
	1    9400 3275
	0    1    1    0   
$EndComp
$Comp
L noeud_senseur-rescue:L_Core_Ferrite-RESCUE-noeud_senseur L16
U 1 1 58DDF543
P 9400 3550
F 0 "L16" V 9500 3550 50  0000 C CNN
F 1 "L_Core_Ferrite" V 9313 3550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9400 3550 50  0001 C CNN
F 3 "" H 9400 3550 50  0001 C CNN
	1    9400 3550
	0    1    1    0   
$EndComp
Wire Wire Line
	9875 3075 9800 3075
Wire Wire Line
	9800 3075 9800 2675
Wire Wire Line
	9800 2675 9550 2675
Wire Wire Line
	8500 3075 9000 3075
Wire Wire Line
	9000 3075 9000 2675
Wire Wire Line
	9000 2675 9250 2675
Wire Wire Line
	8025 3175 9075 3175
Wire Wire Line
	9075 3175 9075 2975
Wire Wire Line
	9075 2975 9250 2975
Wire Wire Line
	9550 2975 9700 2975
Wire Wire Line
	9700 2975 9700 3175
Wire Wire Line
	9700 3175 9875 3175
Wire Wire Line
	8250 3275 9250 3275
Wire Wire Line
	9550 3275 9875 3275
Wire Wire Line
	8250 3375 9075 3375
Wire Wire Line
	9075 3375 9075 3550
Wire Wire Line
	9075 3550 9250 3550
Wire Wire Line
	9550 3550 9700 3550
Wire Wire Line
	9700 3550 9700 3375
Wire Wire Line
	9700 3375 9875 3375
Wire Wire Line
	2825 1100 2825 975 
Wire Wire Line
	2825 975  2975 975 
Wire Wire Line
	3450 1100 3450 975 
Wire Wire Line
	3450 975  3275 975 
$Comp
L noeud_senseur-rescue:L_Core_Ferrite-RESCUE-noeud_senseur L3
U 1 1 58DF39DD
P 6325 1300
F 0 "L3" V 6450 1300 50  0000 C CNN
F 1 "L_Core_Ferrite" V 6238 1300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 6325 1300 50  0001 C CNN
F 3 "" H 6325 1300 50  0001 C CNN
	1    6325 1300
	0    1    1    0   
$EndComp
$Comp
L noeud_senseur-rescue:L_Core_Ferrite-RESCUE-noeud_senseur L4
U 1 1 58DF3B0D
P 6325 1675
F 0 "L4" V 6450 1675 50  0000 C CNN
F 1 "L_Core_Ferrite" V 6238 1675 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 6325 1675 50  0001 C CNN
F 3 "" H 6325 1675 50  0001 C CNN
	1    6325 1675
	0    1    1    0   
$EndComp
Wire Wire Line
	5150 1375 5950 1375
Wire Wire Line
	5950 1375 5950 1300
Wire Wire Line
	5950 1300 6175 1300
Wire Wire Line
	6650 1300 6650 1400
Wire Wire Line
	6650 1300 6475 1300
Wire Wire Line
	5950 1475 5950 1675
Wire Wire Line
	5950 1675 6175 1675
Wire Wire Line
	6650 1500 6650 1675
Wire Wire Line
	6650 1675 6475 1675
$Comp
L noeud_senseur-rescue:L_Core_Ferrite-RESCUE-noeud_senseur L2
U 1 1 58DF6081
P 6200 4150
F 0 "L2" V 6325 4150 50  0000 C CNN
F 1 "L_Core_Ferrite" V 6113 4150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 6200 4150 50  0001 C CNN
F 3 "" H 6200 4150 50  0001 C CNN
	1    6200 4150
	0    1    1    0   
$EndComp
$Comp
L noeud_senseur-rescue:L_Core_Ferrite-RESCUE-noeud_senseur L1
U 1 1 58DF616B
P 6200 3850
F 0 "L1" V 6300 3850 50  0000 C CNN
F 1 "L_Core_Ferrite" V 6113 3850 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 6200 3850 50  0001 C CNN
F 3 "" H 6200 3850 50  0001 C CNN
	1    6200 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	5450 3900 5875 3900
Wire Wire Line
	5875 3900 5875 3850
Wire Wire Line
	5875 3850 6050 3850
Wire Wire Line
	6350 3850 6625 3850
Wire Wire Line
	6525 4150 6350 4150
Wire Wire Line
	6050 4150 5875 4150
Wire Wire Line
	5875 4150 5875 4000
$Comp
L noeud_senseur-rescue:L_Core_Ferrite-RESCUE-noeud_senseur L10
U 1 1 58DF84B8
P 6250 2800
F 0 "L10" V 6375 2800 50  0000 C CNN
F 1 "L_Core_Ferrite" V 6163 2800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 6250 2800 50  0001 C CNN
F 3 "" H 6250 2800 50  0001 C CNN
	1    6250 2800
	0    1    1    0   
$EndComp
$Comp
L noeud_senseur-rescue:L_Core_Ferrite-RESCUE-noeud_senseur L9
U 1 1 58DF84BE
P 6250 2500
F 0 "L9" V 6350 2500 50  0000 C CNN
F 1 "L_Core_Ferrite" V 6163 2500 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 6250 2500 50  0001 C CNN
F 3 "" H 6250 2500 50  0001 C CNN
	1    6250 2500
	0    1    1    0   
$EndComp
Wire Wire Line
	5925 2500 6100 2500
Wire Wire Line
	6400 2500 6575 2500
Wire Wire Line
	6575 2800 6400 2800
Wire Wire Line
	6100 2800 5925 2800
Wire Wire Line
	5100 2625 5925 2625
Wire Wire Line
	5925 2625 5925 2500
Wire Wire Line
	6575 2500 6575 2625
Wire Wire Line
	6575 2625 6900 2625
Wire Wire Line
	5925 2800 5925 2725
Wire Wire Line
	6575 2800 6575 2725
Wire Wire Line
	6575 2725 6900 2725
$Comp
L noeud_senseur-rescue:CP-RESCUE-noeud_senseur C11
U 1 1 58E08EEC
P 1650 4650
F 0 "C11" H 1768 4696 50  0000 L CNN
F 1 "10uF" H 1768 4605 50  0000 L CNN
F 2 "Capacitors_SMD:CP_Elec_4x4.5" H 1688 4500 50  0001 C CNN
F 3 "" H 1650 4650 50  0001 C CNN
	1    1650 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 4500 1650 4425
Connection ~ 1650 4425
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR30
U 1 1 58E09A07
P 1650 4850
F 0 "#PWR30" H -4075 1525 50  0001 C CNN
F 1 "GND" H 1655 4677 50  0000 C CNN
F 2 "" H -4075 1775 50  0001 C CNN
F 3 "" H -4075 1775 50  0001 C CNN
	1    1650 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 4800 1650 4850
$Comp
L noeud_senseur-rescue:CONN_01X03-RESCUE-noeud_senseur OW-1
U 1 1 58E335DF
P 10075 4150
F 0 "OW-1" H 9993 3825 50  0000 C CNN
F 1 "1-Wire" H 9993 3916 50  0000 C CNN
F 2 "caribou:0436500317" H 10075 4150 50  0001 C CNN
F 3 "" H 10075 4150 50  0001 C CNN
	1    10075 4150
	1    0    0    1   
$EndComp
Wire Wire Line
	9875 4050 9700 4050
Wire Wire Line
	9700 4050 9700 3900
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR49
U 1 1 58E3496B
P 9700 3900
F 0 "#PWR49" H 9700 3750 50  0001 C CNN
F 1 "+3.3V" H 9715 4073 50  0000 C CNN
F 2 "" H 9700 3900 50  0001 C CNN
F 3 "" H 9700 3900 50  0001 C CNN
	1    9700 3900
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR50
U 1 1 58E34A13
P 9725 4325
F 0 "#PWR50" H 9725 4075 50  0001 C CNN
F 1 "GND" H 9730 4152 50  0000 C CNN
F 2 "" H 9725 4325 50  0001 C CNN
F 3 "" H 9725 4325 50  0001 C CNN
	1    9725 4325
	1    0    0    -1  
$EndComp
Wire Wire Line
	9875 4250 9725 4250
Wire Wire Line
	9725 4250 9725 4325
Wire Wire Line
	9875 4150 9525 4150
Text HLabel 9525 4150 0    60   BiDi ~ 0
1Wire
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR45
U 1 1 58E3EE78
P 8025 2700
F 0 "#PWR45" H 2725 -400 50  0001 C CNN
F 1 "+3.3V" H 8040 2873 50  0000 C CNN
F 2 "" H 2725 -250 50  0001 C CNN
F 3 "" H 2725 -250 50  0001 C CNN
	1    8025 2700
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR46
U 1 1 58E3EED3
P 8500 2700
F 0 "#PWR46" H 2775 -625 50  0001 C CNN
F 1 "GND" H 8505 2527 50  0000 C CNN
F 2 "" H 2775 -375 50  0001 C CNN
F 3 "" H 2775 -375 50  0001 C CNN
	1    8500 2700
	-1   0    0    1   
$EndComp
Text HLabel 8250 3275 0    60   BiDi ~ 0
SDA
Text HLabel 8250 3375 0    60   Input ~ 0
SCL
Wire Wire Line
	6650 1500 6900 1500
Wire Wire Line
	6650 1400 6900 1400
Text Label 9150 1150 0    60   ~ 0
CAN-
$Comp
L noeud_senseur-rescue:CONN_01X04-RESCUE-noeud_senseur TM-1
U 1 1 58F821C4
P 7350 4750
F 0 "TM-1" H 7428 4791 50  0000 L CNN
F 1 "TmpMoisture" H 7428 4700 50  0000 L CNN
F 2 "Connectors_Molex:Molex_KK-6410-04_04x2.54mm_Straight" H -2550 600 50  0001 C CNN
F 3 "" H -2550 600 50  0001 C CNN
	1    7350 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 5525 5400 5525
Wire Wire Line
	5925 5850 5400 5850
$Comp
L noeud_senseur-rescue:MMBT3904-RESCUE-noeud_senseur Q6
U 1 1 58F834C2
P 5200 5850
F 0 "Q6" H 5391 5896 50  0000 L CNN
F 1 "MMBT3904" H 5391 5805 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 2775 3025 50  0001 L CIN
F 3 "" H 2575 3100 50  0001 L CNN
	1    5200 5850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5100 5650 5100 5000
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR36
U 1 1 58F83A9F
P 5100 5000
F 0 "#PWR36" H 5100 4850 50  0001 C CNN
F 1 "+3.3V" H 5115 5173 50  0000 C CNN
F 2 "" H 5100 5000 50  0001 C CNN
F 3 "" H 5100 5000 50  0001 C CNN
	1    5100 5000
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR38
U 1 1 58F83DFB
P 5400 5000
F 0 "#PWR38" H 5400 4850 50  0001 C CNN
F 1 "+3.3V" H 5415 5173 50  0000 C CNN
F 2 "" H 5400 5000 50  0001 C CNN
F 3 "" H 5400 5000 50  0001 C CNN
	1    5400 5000
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R22
U 1 1 58F83E84
P 5400 5250
F 0 "R22" H 5470 5296 50  0000 L CNN
F 1 "100" H 5470 5205 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V -120 1150 50  0001 C CNN
F 3 "" H -50 1150 50  0001 C CNN
	1    5400 5250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5400 5100 5400 5000
Wire Wire Line
	5400 5525 5400 5400
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R14
U 1 1 58F8443B
P 5100 6300
F 0 "R14" H 5170 6346 50  0000 L CNN
F 1 "10k" H 5170 6255 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V -420 2200 50  0001 C CNN
F 3 "" H -350 2200 50  0001 C CNN
	1    5100 6300
	-1   0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR37
U 1 1 58F844F7
P 5100 6550
F 0 "#PWR37" H -625 3225 50  0001 C CNN
F 1 "GND" H 5105 6377 50  0000 C CNN
F 2 "" H -625 3475 50  0001 C CNN
F 3 "" H -625 3475 50  0001 C CNN
	1    5100 6550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5100 6050 5100 6150
Wire Wire Line
	5100 6550 5100 6450
Wire Wire Line
	5100 6100 4650 6100
Connection ~ 5100 6100
Wire Wire Line
	6525 4150 6525 4700
Wire Wire Line
	6525 4700 7150 4700
Wire Wire Line
	6625 3850 6625 4600
Wire Wire Line
	6625 4600 7150 4600
$Comp
L noeud_senseur-rescue:L_Core_Ferrite-RESCUE-noeud_senseur L11
U 1 1 58F889B9
P 6050 5525
F 0 "L11" V 6150 5525 50  0000 C CNN
F 1 "L_Core_Ferrite" V 5963 5525 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 6050 5525 50  0001 C CNN
F 3 "" H 6050 5525 50  0001 C CNN
	1    6050 5525
	0    1    1    0   
$EndComp
$Comp
L noeud_senseur-rescue:L_Core_Ferrite-RESCUE-noeud_senseur L12
U 1 1 58F88CC8
P 6075 5850
F 0 "L12" V 6175 5825 50  0000 C CNN
F 1 "L_Core_Ferrite" V 5988 5850 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 6075 5850 50  0001 C CNN
F 3 "" H 6075 5850 50  0001 C CNN
	1    6075 5850
	0    1    1    0   
$EndComp
Wire Wire Line
	6200 5525 6700 5525
Wire Wire Line
	6700 5525 6700 4800
Wire Wire Line
	6700 4800 7150 4800
Wire Wire Line
	6225 5850 6825 5850
Wire Wire Line
	6825 5850 6825 4900
Wire Wire Line
	6825 4900 7150 4900
Text Label 4800 6100 2    60   ~ 0
A3
$Comp
L noeud_senseur-rescue:CONN_01X04-RESCUE-noeud_senseur PWR-1
U 1 1 58F90612
P 2875 4575
F 0 "PWR-1" H 2953 4616 50  0000 L CNN
F 1 "CONN_01X04" H 2953 4525 50  0000 L CNN
F 2 "caribou:0430450428" H 2875 4575 50  0001 C CNN
F 3 "" H 2875 4575 50  0001 C CNN
	1    2875 4575
	1    0    0    -1  
$EndComp
NoConn ~ 2675 4625
NoConn ~ 2675 4725
$Comp
L noeud_senseur-rescue:C_Small-RESCUE-noeud_senseur C13
U 1 1 58FE90F0
P 8250 2950
F 0 "C13" V 8375 2900 50  0000 L CNN
F 1 "1uF" V 8125 2900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2275 -3825 50  0001 C CNN
F 3 "" H 2275 -3825 50  0001 C CNN
	1    8250 2950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8350 2950 8500 2950
Connection ~ 8500 2950
Wire Wire Line
	8150 2950 8025 2950
Connection ~ 8025 2950
$Comp
L noeud_senseur-rescue:C_Small-RESCUE-noeud_senseur C14
U 1 1 58FECBD0
P 8825 4125
F 0 "C14" H 8917 4171 50  0000 L CNN
F 1 "1uF" H 8917 4080 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2850 -2650 50  0001 C CNN
F 3 "" H 2850 -2650 50  0001 C CNN
	1    8825 4125
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR47
U 1 1 58FECEDD
P 8825 3950
F 0 "#PWR47" H 8825 3800 50  0001 C CNN
F 1 "+3.3V" H 8840 4123 50  0000 C CNN
F 2 "" H 8825 3950 50  0001 C CNN
F 3 "" H 8825 3950 50  0001 C CNN
	1    8825 3950
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR48
U 1 1 58FECF3E
P 8825 4300
F 0 "#PWR48" H 8825 4050 50  0001 C CNN
F 1 "GND" H 8830 4127 50  0000 C CNN
F 2 "" H 8825 4300 50  0001 C CNN
F 3 "" H 8825 4300 50  0001 C CNN
	1    8825 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8825 3950 8825 4025
Wire Wire Line
	8825 4225 8825 4300
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR41
U 1 1 5937FEAA
P 5500 900
F 0 "#PWR41" H 200 -2200 50  0001 C CNN
F 1 "+3.3V" H 5515 1073 50  0000 C CNN
F 2 "" H 200 -2050 50  0001 C CNN
F 3 "" H 200 -2050 50  0001 C CNN
	1    5500 900 
	-1   0    0    -1  
$EndComp
Text HLabel 5150 1375 0    60   Output ~ 0
NTC_1
Wire Wire Line
	5500 1275 5500 1375
Connection ~ 5500 1375
Wire Wire Line
	5500 975  5500 900 
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR43
U 1 1 593814A4
P 5500 2125
F 0 "#PWR43" H 200 -975 50  0001 C CNN
F 1 "+3.3V" H 5515 2298 50  0000 C CNN
F 2 "" H 200 -825 50  0001 C CNN
F 3 "" H 200 -825 50  0001 C CNN
	1    5500 2125
	-1   0    0    -1  
$EndComp
Text HLabel 5100 2625 0    60   Output ~ 0
NTC_2
Connection ~ 5500 2625
Wire Wire Line
	5500 2175 5500 2125
Wire Wire Line
	5925 2725 5500 2725
$EndSCHEMATC
