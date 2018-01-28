EESchema Schematic File Version 4
LIBS:noeud_senseur-cache
EELAYER 26 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 5 5
Title "Power Monitoring"
Date "2017-06-06"
Rev "1.1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R17
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
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R16
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
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R15
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
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR52
U 1 1 58DBFD32
P 2950 1375
F 0 "#PWR52" H 2950 1225 50  0001 C CNN
F 1 "+3.3V" H 2965 1548 50  0000 C CNN
F 2 "" H 2950 1375 50  0001 C CNN
F 3 "" H 2950 1375 50  0001 C CNN
	1    2950 1375
	-1   0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR51
U 1 1 58DBFD9A
P 2575 2600
F 0 "#PWR51" H 2575 2350 50  0001 C CNN
F 1 "GND" H 2580 2427 50  0000 C CNN
F 2 "" H 2575 2600 50  0001 C CNN
F 3 "" H 2575 2600 50  0001 C CNN
	1    2575 2600
	-1   0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:CP-RESCUE-noeud_senseur C9
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
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR53
U 1 1 58DC135A
P 2950 2600
F 0 "#PWR53" H 2950 2350 50  0001 C CNN
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
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R21
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
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R20
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
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R19
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
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR55
U 1 1 58DC40BE
P 6475 1375
F 0 "#PWR55" H 6475 1225 50  0001 C CNN
F 1 "+3.3V" H 6490 1548 50  0000 C CNN
F 2 "" H 6475 1375 50  0001 C CNN
F 3 "" H 6475 1375 50  0001 C CNN
	1    6475 1375
	-1   0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR54
U 1 1 58DC40C4
P 6100 2600
F 0 "#PWR54" H 6100 2350 50  0001 C CNN
F 1 "GND" H 6105 2427 50  0000 C CNN
F 2 "" H 6100 2600 50  0001 C CNN
F 3 "" H 6100 2600 50  0001 C CNN
	1    6100 2600
	-1   0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:CP-RESCUE-noeud_senseur C10
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
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR56
U 1 1 58DC40D8
P 6475 2600
F 0 "#PWR56" H 6475 2350 50  0001 C CNN
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
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R18
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
L jfng:JACK_TRS_3PINS CT-1
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
Wire Wire Line
	5175 2050 5175 1650
Wire Wire Line
	5175 1650 5000 1650
Wire Wire Line
	5000 1750 5175 1750
Connection ~ 5175 1750
NoConn ~ 1575 1650
$Comp
L jfng:Barrel_Jack VT-1
U 1 1 5939738C
P 4700 1650
F 0 "VT-1" H 4778 1975 50  0000 C CNN
F 1 "Barrel_Jack" H 4778 1884 50  0000 C CNN
F 2 "Connectors_Molex:Molex_KK-6410-03_03x2.54mm_Straight" H 4750 1610 50  0001 C CNN
F 3 "" H 4750 1610 50  0001 C CNN
	1    4700 1650
	1    0    0    -1  
$EndComp
$EndSCHEMATC
