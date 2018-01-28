EESchema Schematic File Version 4
LIBS:noeud_senseur-cache
EELAYER 26 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 5
Title "Noeud Senseur"
Date "2017-06-06"
Rev "1.1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Label 3550 2900 0    60   ~ 0
SCL
Text Label 3550 3000 0    60   ~ 0
SDA
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR8
U 1 1 588F82DF
P 4725 2375
F 0 "#PWR8" H -575 -725 50  0001 C CNN
F 1 "+3.3V" H 4740 2548 50  0000 C CNN
F 2 "" H -575 -575 50  0001 C CNN
F 3 "" H -575 -575 50  0001 C CNN
	1    4725 2375
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R4
U 1 1 588F82F4
P 4725 2650
F 0 "R4" H 4795 2696 50  0000 L CNN
F 1 "2.2k" H 4795 2605 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V -795 -1450 50  0001 C CNN
F 3 "" H -725 -1450 50  0001 C CNN
	1    4725 2650
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R6
U 1 1 588F834B
P 5025 2650
F 0 "R6" H 5095 2696 50  0000 L CNN
F 1 "2.2k" H 5095 2605 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V -495 -1450 50  0001 C CNN
F 3 "" H -425 -1450 50  0001 C CNN
	1    5025 2650
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR3
U 1 1 588FA539
P 1875 1525
F 0 "#PWR3" H -3425 -1575 50  0001 C CNN
F 1 "+3.3V" H 1890 1698 50  0000 C CNN
F 2 "" H -3425 -1425 50  0001 C CNN
F 3 "" H -3425 -1425 50  0001 C CNN
	1    1875 1525
	1    0    0    -1  
$EndComp
Text Label 3550 1500 0    60   ~ 0
VBAT_FEATHER
Text Label 1750 1900 0    60   ~ 0
A0
Text Label 1750 2000 0    60   ~ 0
A1
Text Label 1750 2100 0    60   ~ 0
A2
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR1
U 1 1 588FEF8A
P 1325 1975
F 0 "#PWR1" H -4400 -1350 50  0001 C CNN
F 1 "GND" H 1330 1802 50  0000 C CNN
F 2 "" H -4400 -1100 50  0001 C CNN
F 3 "" H -4400 -1100 50  0001 C CNN
	1    1325 1975
	1    0    0    -1  
$EndComp
Text Label 1750 2200 0    60   ~ 0
A3
Text Label 4125 2800 2    60   ~ 0
1_WIRE_IO
Text Label 4250 5775 0    60   ~ 0
1_WIRE_IO
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR9
U 1 1 5890E20D
P 5050 5325
F 0 "#PWR9" H -250 2225 50  0001 C CNN
F 1 "+3.3V" H 5065 5498 50  0000 C CNN
F 2 "" H -250 2375 50  0001 C CNN
F 3 "" H -250 2375 50  0001 C CNN
	1    5050 5325
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR10
U 1 1 5890E24B
P 5050 5950
F 0 "#PWR10" H -675 2625 50  0001 C CNN
F 1 "GND" H 5055 5777 50  0000 C CNN
F 2 "" H -675 2875 50  0001 C CNN
F 3 "" H -675 2875 50  0001 C CNN
	1    5050 5950
	1    0    0    -1  
$EndComp
Text Label 1750 2500 0    60   ~ 0
SCK
Text Label 1750 2600 0    60   ~ 0
MOSI
Text Label 4125 2600 2    60   ~ 0
MCP2515_CS
Text Label 4125 2500 2    60   ~ 0
SD_CS
Text Label 4125 2400 2    60   ~ 0
LED_1
Text Label 4125 2300 2    60   ~ 0
LED_2
NoConn ~ 2100 1500
NoConn ~ 2700 3600
NoConn ~ 2900 3600
NoConn ~ 3450 2200
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R5
U 1 1 58A27935
P 4775 5550
F 0 "R5" H 4845 5596 50  0000 L CNN
F 1 "4.7k" H 4845 5505 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V -170 -250 50  0001 C CNN
F 3 "" H -100 -250 50  0001 C CNN
	1    4775 5550
	1    0    0    -1  
$EndComp
Text Label 1750 2300 0    60   ~ 0
A4
Text Label 1750 2400 0    60   ~ 0
A5
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR5
U 1 1 58A6E58C
P 2125 5925
F 0 "#PWR5" H -3600 2600 50  0001 C CNN
F 1 "GND" H 2130 5752 50  0000 C CNN
F 2 "" H -3600 2850 50  0001 C CNN
F 3 "" H -3600 2850 50  0001 C CNN
	1    2125 5925
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:LED-RESCUE-noeud_senseur D1
U 1 1 58A70BCB
P 2125 5200
F 0 "D1" V 2163 5083 50  0000 R CNN
F 1 "LED" V 2072 5083 50  0000 R CNN
F 2 "LEDs:LED-5MM" H -425 4100 50  0001 C CNN
F 3 "" H -425 4100 50  0001 C CNN
	1    2125 5200
	0    -1   -1   0   
$EndComp
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R1
U 1 1 58A71790
P 2125 4825
F 0 "R1" H 2195 4871 50  0000 L CNN
F 1 "1k" H 2195 4780 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 380 3375 50  0001 C CNN
F 3 "" H 450 3375 50  0001 C CNN
	1    2125 4825
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR4
U 1 1 58A718EA
P 2125 4625
F 0 "#PWR4" H -3175 1525 50  0001 C CNN
F 1 "+3.3V" H 2140 4798 50  0000 C CNN
F 2 "" H -3175 1675 50  0001 C CNN
F 3 "" H -3175 1675 50  0001 C CNN
	1    2125 4625
	1    0    0    -1  
$EndComp
Text Label 1550 5700 0    60   ~ 0
LED_1
$Comp
L jfng:DMG1012T Q1
U 1 1 58A736DE
P 2025 5650
F 0 "Q1" H 2216 5696 50  0000 L CNN
F 1 "DMG1012UW" H 2216 5605 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 4450 3250 50  0001 L CIN
F 3 "" H 4250 3325 50  0001 L CNN
	1    2025 5650
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR7
U 1 1 58A74946
P 3300 5925
F 0 "#PWR7" H -2425 2600 50  0001 C CNN
F 1 "GND" H 3305 5752 50  0000 C CNN
F 2 "" H -2425 2850 50  0001 C CNN
F 3 "" H -2425 2850 50  0001 C CNN
	1    3300 5925
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:LED-RESCUE-noeud_senseur D2
U 1 1 58A7494E
P 3300 5200
F 0 "D2" V 3338 5083 50  0000 R CNN
F 1 "LED" V 3247 5083 50  0000 R CNN
F 2 "LEDs:LED-5MM" H 750 4100 50  0001 C CNN
F 3 "" H 750 4100 50  0001 C CNN
	1    3300 5200
	0    -1   -1   0   
$EndComp
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R2
U 1 1 58A74954
P 3300 4825
F 0 "R2" H 3370 4871 50  0000 L CNN
F 1 "1k" H 3370 4780 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1555 3375 50  0001 C CNN
F 3 "" H 1625 3375 50  0001 C CNN
	1    3300 4825
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR6
U 1 1 58A7495A
P 3300 4625
F 0 "#PWR6" H -2000 1525 50  0001 C CNN
F 1 "+3.3V" H 3315 4798 50  0000 C CNN
F 2 "" H -2000 1675 50  0001 C CNN
F 3 "" H -2000 1675 50  0001 C CNN
	1    3300 4625
	1    0    0    -1  
$EndComp
Text Label 2725 5700 0    60   ~ 0
LED_2
$Comp
L jfng:DMG1012T Q2
U 1 1 58A74964
P 3200 5650
F 0 "Q2" H 3391 5696 50  0000 L CNN
F 1 "DMG1012UW" H 3391 5605 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 5625 3250 50  0001 L CIN
F 3 "" H 5425 3325 50  0001 L CNN
	1    3200 5650
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:+3.3V-RESCUE-noeud_senseur #PWR2
U 1 1 58A74E2F
P 1625 1525
F 0 "#PWR2" H -3675 -1575 50  0001 C CNN
F 1 "+3.3V" H 1640 1698 50  0000 C CNN
F 2 "" H -3675 -1425 50  0001 C CNN
F 3 "" H -3675 -1425 50  0001 C CNN
	1    1625 1525
	1    0    0    -1  
$EndComp
Text Label 1750 2700 0    60   ~ 0
MISO
Text Label 1750 2800 0    60   ~ 0
RX
Text Label 1750 2900 0    60   ~ 0
TX
NoConn ~ 2100 3000
Text Label 4125 2700 2    60   ~ 0
MCP2515_INT
NoConn ~ 3450 1900
$Comp
L noeud_senseur-rescue:FEATHER_MO_LORA-RESCUE-noeud_senseur Adafruit_Feather-1
U 1 1 58A7C9E5
P 2800 2400
F 0 "Adafruit_Feather-1" H 2787 3687 60  0000 C CNN
F 1 "FEATHER_MO_LORA" H 2787 3581 60  0000 C CNN
F 2 "caribou:FEATHER_MO_LORA" H 525 -2000 60  0001 C CNN
F 3 "" H 525 -2000 60  0001 C CNN
	1    2800 2400
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:PWR_FLAG-RESCUE-noeud_senseur #FLG2
U 1 1 58A85600
P 4400 1425
F 0 "#FLG2" H 0   -3175 50  0001 C CNN
F 1 "PWR_FLAG" H 4400 1599 50  0000 C CNN
F 2 "" H 0   -3250 50  0001 C CNN
F 3 "" H 0   -3250 50  0001 C CNN
	1    4400 1425
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:PWR_FLAG-RESCUE-noeud_senseur #FLG1
U 1 1 58A87270
P 1325 1650
F 0 "#FLG1" H 225 -3225 50  0001 C CNN
F 1 "PWR_FLAG" H 1325 1824 50  0000 C CNN
F 2 "" H 225 -3300 50  0001 C CNN
F 3 "" H 225 -3300 50  0001 C CNN
	1    1325 1650
	1    0    0    -1  
$EndComp
$Comp
L jfng:ORD211-1015 S1
U 1 1 58AD8EEB
P 5775 1900
F 0 "S1" V 5722 1988 60  0000 L CNN
F 1 "ORD211-1015" V 5828 1988 60  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 1625 -2150 60  0001 C CNN
F 3 "" H 1625 -2150 60  0001 C CNN
	1    5775 1900
	0    1    1    0   
$EndComp
$Comp
L noeud_senseur-rescue:GND-RESCUE-noeud_senseur #PWR11
U 1 1 58AE33ED
P 5775 2175
F 0 "#PWR11" H 50  -1150 50  0001 C CNN
F 1 "GND" H 5780 2002 50  0000 C CNN
F 2 "" H 50  -900 50  0001 C CNN
F 3 "" H 50  -900 50  0001 C CNN
	1    5775 2175
	1    0    0    -1  
$EndComp
$Comp
L jfng:Q_PMOS_SGD Q3
U 1 1 58AF3766
P 4350 1775
F 0 "Q3" H 4556 1729 50  0000 L CNN
F 1 "LP0701_SGD" H 4556 1820 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Narrow_Oval" H -575 -3150 50  0001 C CNN
F 3 "" H -775 -3250 50  0001 C CNN
	1    4350 1775
	-1   0    0    1   
$EndComp
$Comp
L noeud_senseur-rescue:R-RESCUE-noeud_senseur R3
U 1 1 58AF4304
P 4700 1500
F 0 "R3" V 4625 1450 50  0000 L CNN
F 1 "50k" V 4800 1450 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V -820 -2600 50  0001 C CNN
F 3 "" H -750 -2600 50  0001 C CNN
	1    4700 1500
	0    1    1    0   
$EndComp
Connection ~ 4975 1625
Wire Wire Line
	4975 1500 4850 1500
Wire Wire Line
	4975 1500 4975 1625
Wire Wire Line
	4975 1775 4550 1775
Connection ~ 4250 1500
Wire Wire Line
	4250 1500 4250 1575
Wire Wire Line
	3525 1500 4250 1500
Wire Wire Line
	4250 2000 4250 1975
Wire Wire Line
	3525 2100 3450 2100
Wire Wire Line
	4725 2450 5025 2450
Wire Wire Line
	4250 2000 3450 2000
Connection ~ 1325 1800
Connection ~ 4400 1500
Wire Wire Line
	4400 1500 4400 1425
Wire Wire Line
	3450 2700 4475 2700
Wire Wire Line
	3450 2300 4125 2300
Wire Wire Line
	2100 2900 1750 2900
Wire Wire Line
	2100 2800 1750 2800
Wire Wire Line
	2100 2700 1750 2700
Wire Wire Line
	2100 2600 1750 2600
Wire Wire Line
	2100 2400 1750 2400
Wire Wire Line
	2100 2300 1750 2300
Wire Wire Line
	2100 2200 1750 2200
Wire Wire Line
	2100 2100 1750 2100
Wire Wire Line
	2100 1900 1750 1900
Wire Wire Line
	1625 1700 1625 1525
Wire Wire Line
	3300 5350 3300 5450
Wire Wire Line
	3300 4975 3300 5050
Wire Wire Line
	3300 4625 3300 4675
Wire Wire Line
	3000 5700 2700 5700
Wire Wire Line
	3300 5925 3300 5850
Wire Wire Line
	2125 5350 2125 5450
Wire Wire Line
	2125 4975 2125 5050
Wire Wire Line
	2125 4625 2125 4675
Wire Wire Line
	1825 5700 1525 5700
Wire Wire Line
	2125 5925 2125 5850
Wire Wire Line
	2100 2500 1750 2500
Connection ~ 5050 5400
Wire Wire Line
	4775 5400 5050 5400
Connection ~ 4775 5775
Wire Wire Line
	4775 5700 4775 5775
Wire Wire Line
	3450 2400 4125 2400
Wire Wire Line
	3450 2500 4125 2500
Wire Wire Line
	3450 2600 4125 2600
Wire Wire Line
	3525 2100 3525 1500
Wire Wire Line
	5050 5875 5050 5950
Wire Wire Line
	5525 5875 5050 5875
Wire Wire Line
	5050 5675 5525 5675
Wire Wire Line
	5050 5325 5050 5400
Wire Wire Line
	4200 5775 4775 5775
Wire Wire Line
	3450 2800 4125 2800
Wire Wire Line
	1625 1700 2100 1700
Wire Wire Line
	1325 1650 1325 1800
Wire Wire Line
	2100 2000 1750 2000
Wire Wire Line
	2100 1800 1325 1800
Wire Wire Line
	1875 1600 1875 1525
Wire Wire Line
	1875 1600 2100 1600
Wire Wire Line
	5025 3000 5025 2800
Wire Wire Line
	4725 2900 4725 2800
Connection ~ 4725 2450
Wire Wire Line
	5025 2450 5025 2500
Wire Wire Line
	4725 2375 4725 2450
Wire Wire Line
	3450 3000 5025 3000
Wire Wire Line
	3450 2900 4725 2900
Wire Wire Line
	5775 2175 5775 2100
$Sheet
S 6575 2350 1175 850 
U 58BFA1BE
F0 "SPI" 60
F1 "SPI.sch" 60
F2 "CAN+" B R 7750 2600 60 
F3 "CAN-" B R 7750 2700 60 
F4 "MCP2515_CS" I L 6575 2600 60 
F5 "MISO" O L 6575 2800 60 
F6 "MOSI" I L 6575 2700 60 
F7 "SCK" I L 6575 2900 60 
F8 "MCP2515_INT" O L 6575 3000 60 
F9 "SD_CS" I L 6575 2500 60 
$EndSheet
$Sheet
S 6600 1525 525  450 
U 58C0992B
F0 "I2C" 60
F1 "I2C.sch" 60
F2 "INT_SQW" O L 6600 1625 60 
F3 "I2C_SCL" I L 6600 1725 60 
F4 "I2C_SDA" B L 6600 1825 60 
$EndSheet
$Sheet
S 8700 2475 825  1850
U 58C09996
F0 "Connect" 60
F1 "Connect.sch" 60
F2 "CAN+" B L 8700 2600 60 
F3 "CAN-" B L 8700 2700 60 
F4 "SSR_1" I L 8700 2800 60 
F5 "SSR_2" I L 8700 2900 60 
F6 "NTC_1" O L 8700 3025 60 
F7 "NTC_2" O L 8700 3125 60 
F8 "SCL" I L 8700 3450 60 
F9 "SDA" B L 8700 3550 60 
F10 "RX" I L 8700 3650 60 
F11 "TX" O L 8700 3750 60 
F12 "VBAT" U L 8700 3900 60 
F13 "1Wire" B L 8700 4100 60 
F14 "PTC" O L 8700 3225 60 
F15 "MOISTURE" O L 8700 3325 60 
$EndSheet
Wire Wire Line
	4975 1625 5400 1625
Wire Wire Line
	6600 1725 6225 1725
Wire Wire Line
	6600 1825 6225 1825
Text Label 6225 1725 0    60   ~ 0
SCL
Text Label 6225 1825 0    60   ~ 0
SDA
Wire Wire Line
	6575 2500 5950 2500
Wire Wire Line
	6575 2600 5950 2600
Wire Wire Line
	6575 2700 5950 2700
Wire Wire Line
	6575 2800 5950 2800
Wire Wire Line
	6575 2900 5950 2900
Wire Wire Line
	6575 3000 5950 3000
Wire Wire Line
	7750 2600 8700 2600
Wire Wire Line
	7750 2700 8700 2700
Wire Wire Line
	8325 2800 8700 2800
Wire Wire Line
	8700 2900 8325 2900
Text Label 8325 2800 0    60   ~ 0
A4
Text Label 8325 2900 0    60   ~ 0
A5
Text Label 5950 2500 0    60   ~ 0
SD_CS
Text Label 5950 2600 0    60   ~ 0
MCP2515_CS
Text Label 5950 2700 0    60   ~ 0
MOSI
Text Label 5950 2800 0    60   ~ 0
MISO
Text Label 5950 2900 0    60   ~ 0
SCK
Text Label 5950 3000 0    60   ~ 0
MCP2515_INT
Wire Wire Line
	8700 3025 8325 3025
Wire Wire Line
	8700 3125 8325 3125
Wire Wire Line
	8700 3225 8325 3225
Wire Wire Line
	8700 3325 8325 3325
Wire Wire Line
	8700 3450 8325 3450
Wire Wire Line
	8700 3550 8325 3550
Wire Wire Line
	8700 3650 8325 3650
Wire Wire Line
	8700 3750 8325 3750
Wire Wire Line
	8700 3900 7950 3900
Text Label 8325 3025 0    60   ~ 0
A0
Text Label 8325 3125 0    60   ~ 0
A1
Text Label 8325 3225 0    60   ~ 0
A2
Text Label 8325 3325 0    60   ~ 0
A3
Text Label 8325 3450 0    60   ~ 0
SCL
Text Label 8325 3550 0    60   ~ 0
SDA
Text Label 8325 3650 0    60   ~ 0
RX
Text Label 8325 3750 0    60   ~ 0
TX
Text Label 7950 3900 0    60   ~ 0
VBAT_FEATHER
Text Notes 4700 1275 0    60   ~ 0
R3 : Pull Up de 50k à 100k
$Sheet
S 5400 3725 1150 900 
U 58DBF4EE
F0 "PowerMonitoring" 60
F1 "PowerMonitoring.sch" 60
F2 "CT_OUT" O L 5400 3925 60 
F3 "VT_OUT" O L 5400 4075 60 
$EndSheet
Wire Wire Line
	5400 3925 4975 3925
Wire Wire Line
	5400 4075 4975 4075
Text Label 5050 3925 0    60   ~ 0
A2
Text Label 5050 4075 0    60   ~ 0
A3
Wire Wire Line
	8700 4100 7950 4100
Text Label 8025 4100 0    60   ~ 0
1_WIRE_IO
$Comp
L jfng:DS18B20 U2
U 1 1 593A33A5
P 5825 5775
F 0 "U2" H 6052 5821 50  0000 L CNN
F 1 "DS18B20" H 6052 5730 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Wide" H 5675 6025 50  0001 C CNN
F 3 "" H 5675 6025 50  0001 C CNN
	1    5825 5775
	1    0    0    -1  
$EndComp
$Comp
L noeud_senseur-rescue:Jumper-RESCUE-noeud_senseur JP1
U 1 1 5949FBE4
P 5100 2050
F 0 "JP1" H 5100 2314 50  0000 C CNN
F 1 "Jumper" H 5100 2223 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 5100 2050 50  0001 C CNN
F 3 "" H 5100 2050 50  0001 C CNN
	1    5100 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5775 1700 5775 1625
Connection ~ 5775 1625
Wire Wire Line
	5400 2050 5400 1625
Connection ~ 5400 1625
Wire Wire Line
	4475 2700 4475 2050
Wire Wire Line
	4475 2050 4800 2050
Wire Wire Line
	4975 1625 4975 1775
Wire Wire Line
	4250 1500 4400 1500
Wire Wire Line
	1325 1800 1325 1975
Wire Wire Line
	4400 1500 4550 1500
Wire Wire Line
	5050 5400 5050 5675
Wire Wire Line
	4775 5775 5525 5775
Wire Wire Line
	4725 2450 4725 2500
Wire Wire Line
	5775 1625 6600 1625
Wire Wire Line
	5400 1625 5775 1625
$EndSCHEMATC
