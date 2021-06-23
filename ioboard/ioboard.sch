EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr User 20000 16000
encoding utf-8
Sheet 1 1
Title "GRBL-STM32 I/O Board"
Date "2021-03-28"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	2050 2200 1700 2200
Wire Wire Line
	2050 2100 1700 2100
Wire Wire Line
	2050 2000 1700 2000
Wire Wire Line
	2050 1900 1700 1900
Wire Wire Line
	2050 1800 1700 1800
Wire Wire Line
	2050 1700 1700 1700
Wire Wire Line
	2050 1600 1700 1600
Wire Wire Line
	2050 1500 1700 1500
Text Label 1700 1500 0    50   ~ 10
XPUL+
Text Label 1700 1600 0    50   ~ 10
XPUL-
Text Label 1700 1700 0    50   ~ 10
XDIR+
Text Label 1700 1800 0    50   ~ 10
XDIR-
Text Label 1700 1900 0    50   ~ 10
XENA+
Text Label 1700 2000 0    50   ~ 10
XENA-
Text Label 1700 2100 0    50   ~ 10
XALM+
Text Label 1700 2200 0    50   ~ 10
XALM-
$Comp
L Connector_Generic:Conn_01x08 J?
U 1 1 6064E449
P 2250 1800
F 0 "J?" H 2330 1792 50  0000 L CNN
F 1 "Signal_XAxis" H 2330 1701 50  0000 L CNN
F 2 "" H 2250 1800 50  0001 C CNN
F 3 "~" H 2250 1800 50  0001 C CNN
	1    2250 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 3700 1700 3700
Wire Wire Line
	2050 3600 1700 3600
Wire Wire Line
	2050 3500 1700 3500
Wire Wire Line
	2050 3400 1700 3400
Wire Wire Line
	2050 3300 1700 3300
Wire Wire Line
	2050 3200 1700 3200
Wire Wire Line
	2050 3100 1700 3100
Wire Wire Line
	2050 3000 1700 3000
Text Label 1700 3000 0    50   ~ 10
YPUL+
Text Label 1700 3100 0    50   ~ 10
YPUL-
Text Label 1700 3200 0    50   ~ 10
YDIR+
Text Label 1700 3300 0    50   ~ 10
YDIR-
Text Label 1700 3400 0    50   ~ 10
YENA+
Text Label 1700 3500 0    50   ~ 10
YENA-
Text Label 1700 3600 0    50   ~ 10
YALM+
Text Label 1700 3700 0    50   ~ 10
YALM-
$Comp
L Connector_Generic:Conn_01x08 J?
U 1 1 6067439E
P 2250 3300
F 0 "J?" H 2330 3292 50  0000 L CNN
F 1 "Signal_YAxis" H 2330 3201 50  0000 L CNN
F 2 "" H 2250 3300 50  0001 C CNN
F 3 "~" H 2250 3300 50  0001 C CNN
	1    2250 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 5200 1700 5200
Wire Wire Line
	2050 5100 1700 5100
Wire Wire Line
	2050 5000 1700 5000
Wire Wire Line
	2050 4900 1700 4900
Wire Wire Line
	2050 4800 1700 4800
Wire Wire Line
	2050 4700 1700 4700
Wire Wire Line
	2050 4600 1700 4600
Wire Wire Line
	2050 4500 1700 4500
Text Label 1700 4500 0    50   ~ 10
ZPUL+
Text Label 1700 4600 0    50   ~ 10
ZPUL-
Text Label 1700 4700 0    50   ~ 10
ZDIR+
Text Label 1700 4800 0    50   ~ 10
ZDIR-
Text Label 1700 4900 0    50   ~ 10
ZENA+
Text Label 1700 5000 0    50   ~ 10
ZENA-
Text Label 1700 5100 0    50   ~ 10
ZALM+
Text Label 1700 5200 0    50   ~ 10
ZALM-
$Comp
L Connector_Generic:Conn_01x08 J?
U 1 1 6067AC7A
P 2250 4800
F 0 "J?" H 2330 4792 50  0000 L CNN
F 1 "Signal_ZAxis" H 2330 4701 50  0000 L CNN
F 2 "" H 2250 4800 50  0001 C CNN
F 3 "~" H 2250 4800 50  0001 C CNN
	1    2250 4800
	1    0    0    -1  
$EndComp
$Comp
L power:+24V #PWR?
U 1 1 6063393F
P 12360 1500
F 0 "#PWR?" H 12360 1350 50  0001 C CNN
F 1 "+24V" H 12375 1673 50  0000 C CNN
F 2 "" H 12360 1500 50  0001 C CNN
F 3 "" H 12360 1500 50  0001 C CNN
	1    12360 1500
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Switching:LM2576HVT-5 U?
U 1 1 60642992
P 12950 1700
F 0 "U?" H 12950 2067 50  0000 C CNN
F 1 "LM2576HVT-5" H 12950 1976 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-5_Vertical" H 12950 1450 50  0001 L CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm2576.pdf" H 12950 1700 50  0001 C CNN
	1    12950 1700
	1    0    0    -1  
$EndComp
Text Label 11580 1600 0    50   ~ 0
VDC
$Comp
L power:GND #PWR?
U 1 1 6064C6F0
P 12950 2140
F 0 "#PWR?" H 12950 1890 50  0001 C CNN
F 1 "GND" H 12955 1967 50  0000 C CNN
F 2 "" H 12950 2140 50  0001 C CNN
F 3 "" H 12950 2140 50  0001 C CNN
	1    12950 2140
	1    0    0    -1  
$EndComp
Wire Wire Line
	12950 2000 12950 2100
Wire Wire Line
	14160 1800 14160 1600
Wire Wire Line
	14160 1600 13450 1600
Text Label 15030 1800 2    50   ~ 0
VCC
Wire Wire Line
	15030 1800 14770 1800
$Comp
L Device:L L?
U 1 1 6068D237
P 13860 1800
F 0 "L?" V 13679 1800 50  0000 C CNN
F 1 "100uH" V 13770 1800 50  0000 C CNN
F 2 "" H 13860 1800 50  0001 C CNN
F 3 "~" H 13860 1800 50  0001 C CNN
	1    13860 1800
	0    1    1    0   
$EndComp
Wire Wire Line
	14160 1800 14010 1800
Connection ~ 14160 1800
$Comp
L Device:D_Schottky D?
U 1 1 6068E467
P 13550 1950
F 0 "D?" V 13504 2030 50  0000 L CNN
F 1 "1N5822" V 13595 2030 50  0000 L CNN
F 2 "" H 13550 1950 50  0001 C CNN
F 3 "~" H 13550 1950 50  0001 C CNN
	1    13550 1950
	0    1    1    0   
$EndComp
Wire Wire Line
	13710 1800 13550 1800
Wire Wire Line
	13550 1800 13450 1800
Connection ~ 13550 1800
Wire Wire Line
	13550 2100 12950 2100
Connection ~ 12950 2100
Wire Wire Line
	12950 2100 12950 2140
Wire Wire Line
	12450 1800 12240 1800
Wire Wire Line
	12240 1800 12240 2100
Wire Wire Line
	12240 2100 12950 2100
$Comp
L power:GND #PWR?
U 1 1 606A073E
P 6800 11800
F 0 "#PWR?" H 6800 11550 50  0001 C CNN
F 1 "GND" H 6805 11627 50  0000 C CNN
F 2 "" H 6800 11800 50  0001 C CNN
F 3 "" H 6800 11800 50  0001 C CNN
	1    6800 11800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 606A4231
P 4100 11500
F 0 "#PWR?" H 4100 11250 50  0001 C CNN
F 1 "GND" H 4105 11327 50  0000 C CNN
F 2 "" H 4100 11500 50  0001 C CNN
F 3 "" H 4100 11500 50  0001 C CNN
	1    4100 11500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 1200 5600 1200
Wire Wire Line
	5050 1640 4700 1640
Wire Wire Line
	4800 7900 4450 7900
Wire Wire Line
	5050 1540 4700 1540
Wire Wire Line
	4800 9300 4450 9300
Wire Wire Line
	5050 1440 4700 1440
Text Label 4450 9300 0    50   ~ 10
XPUL-
Text Label 4450 7900 0    50   ~ 10
XDIR-
Text Label 5600 1200 0    50   ~ 10
XALM-
Wire Wire Line
	6000 1400 5600 1400
Wire Wire Line
	5050 2140 4700 2140
Wire Wire Line
	4800 8100 4450 8100
Wire Wire Line
	5050 2040 4700 2040
Wire Wire Line
	5050 1940 4700 1940
Text Label 4450 8100 0    50   ~ 10
YDIR-
Text Label 4450 7100 0    50   ~ 10
YALM+
Text Label 5600 1400 0    50   ~ 10
YALM-
Wire Wire Line
	6000 1600 5600 1600
Wire Wire Line
	4800 9100 4450 9100
Wire Wire Line
	5050 2540 4700 2540
Text Label 4450 9100 0    50   ~ 10
ZDIR-
Text Label 4450 6900 0    50   ~ 10
ZALM+
Text Label 5600 1600 0    50   ~ 10
ZALM-
Wire Wire Line
	5050 2440 4700 2440
Wire Wire Line
	5050 2640 4700 2640
Wire Wire Line
	14360 1600 14360 1800
Connection ~ 14360 1800
Wire Wire Line
	14360 1800 14160 1800
Wire Wire Line
	12450 1600 12360 1600
Wire Wire Line
	12360 1600 12360 1500
Wire Wire Line
	12360 1600 11920 1600
Connection ~ 12360 1600
$Comp
L power:+5V #PWR?
U 1 1 607115CA
P 14360 1600
F 0 "#PWR?" H 14360 1450 50  0001 C CNN
F 1 "+5V" H 14375 1773 50  0000 C CNN
F 2 "" H 14360 1600 50  0001 C CNN
F 3 "" H 14360 1600 50  0001 C CNN
	1    14360 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 1440 5050 1540
Wire Wire Line
	5050 2640 5050 2540
Connection ~ 5050 1540
Connection ~ 5050 1640
Wire Wire Line
	5050 1640 5050 1540
Connection ~ 5050 1940
Wire Wire Line
	5050 1940 5050 1640
Connection ~ 5050 2040
Wire Wire Line
	5050 2040 5050 1940
Connection ~ 5050 2140
Wire Wire Line
	5050 2140 5050 2040
Connection ~ 5050 2440
Wire Wire Line
	5050 2440 5050 2140
Connection ~ 5050 2540
Wire Wire Line
	5050 2540 5050 2440
$Comp
L power:GND #PWR?
U 1 1 6073A77E
P 6000 2390
F 0 "#PWR?" H 6000 2140 50  0001 C CNN
F 1 "GND" H 6005 2217 50  0000 C CNN
F 2 "" H 6000 2390 50  0001 C CNN
F 3 "" H 6000 2390 50  0001 C CNN
	1    6000 2390
	1    0    0    -1  
$EndComp
Connection ~ 6000 1400
Wire Wire Line
	6000 1400 6000 1200
Wire Wire Line
	6000 1600 6000 1400
Wire Wire Line
	2050 6600 1700 6600
Wire Wire Line
	2050 6500 1700 6500
Wire Wire Line
	2050 6400 1700 6400
Wire Wire Line
	2050 6300 1700 6300
Wire Wire Line
	2050 6200 1700 6200
Wire Wire Line
	2050 6100 1700 6100
Wire Wire Line
	2050 6000 1700 6000
Text Label 1700 6000 0    50   ~ 10
APUL+
Text Label 1700 6100 0    50   ~ 10
APUL-
Text Label 1700 6200 0    50   ~ 10
ADIR+
Text Label 1700 6300 0    50   ~ 10
ADIR-
Text Label 1700 6400 0    50   ~ 10
AENA+
Text Label 1700 6500 0    50   ~ 10
AENA-
$Comp
L Connector_Generic:Conn_01x08 J?
U 1 1 607582D3
P 2250 6300
F 0 "J?" H 2330 6292 50  0000 L CNN
F 1 "Signal_AAxis" H 2330 6201 50  0000 L CNN
F 2 "" H 2250 6300 50  0001 C CNN
F 3 "~" H 2250 6300 50  0001 C CNN
	1    2250 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 8200 1700 8200
Wire Wire Line
	2050 8100 1700 8100
Wire Wire Line
	2050 8000 1700 8000
Wire Wire Line
	2050 7900 1700 7900
Wire Wire Line
	2050 7800 1700 7800
Wire Wire Line
	2050 7700 1700 7700
Wire Wire Line
	2050 7600 1700 7600
Wire Wire Line
	2050 7500 1700 7500
Text Label 1700 7500 0    50   ~ 10
BPUL+
Text Label 1700 7600 0    50   ~ 10
BPUL-
Text Label 1700 7700 0    50   ~ 10
BDIR+
Text Label 1700 7800 0    50   ~ 10
BDIR-
Text Label 1700 7900 0    50   ~ 10
BENA+
Text Label 1700 8000 0    50   ~ 10
BENA-
Text Label 1700 8100 0    50   ~ 10
BALM+
Text Label 1700 8200 0    50   ~ 10
BALM-
$Comp
L Connector_Generic:Conn_01x08 J?
U 1 1 6075E2D2
P 2250 7800
F 0 "J?" H 2330 7792 50  0000 L CNN
F 1 "Signal_BAxis" H 2330 7701 50  0000 L CNN
F 2 "" H 2250 7800 50  0001 C CNN
F 3 "~" H 2250 7800 50  0001 C CNN
	1    2250 7800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 3150 4700 3150
Wire Wire Line
	6000 9300 6600 9300
Wire Wire Line
	5050 3050 4700 3050
Wire Wire Line
	6000 6000 6600 6000
Wire Wire Line
	5050 2940 4700 2940
Text Label 7250 6460 2    50   ~ 10
APUL-
Text Label 6600 9300 2    50   ~ 10
ADIR-
Wire Wire Line
	6000 2000 5600 2000
Wire Wire Line
	5050 3640 4700 3640
Wire Wire Line
	6000 9100 6590 9100
Wire Wire Line
	5050 3540 4700 3540
Wire Wire Line
	6000 5800 6600 5800
Wire Wire Line
	5050 3450 4700 3450
Text Label 7260 6100 2    50   ~ 10
BPUL-
Text Label 6590 9100 2    50   ~ 10
BDIR-
Text Label 5600 2000 0    50   ~ 10
BALM-
Wire Wire Line
	5050 3640 5050 3540
Connection ~ 5050 2640
Connection ~ 5050 2940
Wire Wire Line
	5050 2940 5050 2640
Connection ~ 5050 3050
Wire Wire Line
	5050 3050 5050 2940
Connection ~ 5050 3150
Wire Wire Line
	5050 3150 5050 3050
Connection ~ 5050 3450
Wire Wire Line
	5050 3450 5050 3150
Connection ~ 5050 3540
Wire Wire Line
	5050 3540 5050 3450
Text Label 5600 1800 0    50   ~ 10
AALM-
Wire Wire Line
	6000 1800 5600 1800
Wire Wire Line
	6000 1600 6000 1800
Connection ~ 6000 1600
Wire Wire Line
	6000 1800 6000 2000
Connection ~ 6000 1800
Wire Wire Line
	6000 2000 6000 2200
Connection ~ 6000 2000
$Comp
L Device:CP C?
U 1 1 60949C36
P 11920 1890
F 0 "C?" H 11620 1940 50  0000 L CNN
F 1 "470uF" H 11520 1850 50  0000 L CNN
F 2 "" H 11958 1740 50  0001 C CNN
F 3 "~" H 11920 1890 50  0001 C CNN
	1    11920 1890
	1    0    0    -1  
$EndComp
Wire Wire Line
	11920 2040 11920 2100
Wire Wire Line
	11920 2100 12240 2100
Connection ~ 12240 2100
Wire Wire Line
	11920 1740 11920 1600
Connection ~ 11920 1600
Wire Wire Line
	11920 1600 11780 1600
$Comp
L Device:CP C?
U 1 1 6095CFC4
P 14600 1950
F 0 "C?" H 14718 1996 50  0000 L CNN
F 1 "1000uF" H 14718 1905 50  0000 L CNN
F 2 "" H 14638 1800 50  0001 C CNN
F 3 "~" H 14600 1950 50  0001 C CNN
	1    14600 1950
	1    0    0    -1  
$EndComp
Connection ~ 14600 1800
Wire Wire Line
	14600 1800 14360 1800
Wire Wire Line
	14600 2100 13550 2100
Connection ~ 13550 2100
Text Label 4450 9600 0    50   ~ 10
XALM+
Text Label 6590 6400 2    50   ~ 10
YPUL-
Wire Wire Line
	6000 6400 6590 6400
Text Label 7310 6810 2    50   ~ 10
SPPWM
Wire Wire Line
	6000 6200 6600 6200
Text Label 6590 6600 2    50   ~ 10
XENA-
Text Label 6590 6800 2    50   ~ 10
YENA-
Text Label 6590 7000 2    50   ~ 10
ZENA-
Text Label 6590 7200 2    50   ~ 10
AENA-
Text Label 6590 7400 2    50   ~ 10
BENA-
Text Label 4400 11600 0    50   ~ 0
VCC
Wire Wire Line
	4800 11600 4400 11600
$Comp
L power:GND #PWR?
U 1 1 60B2FA0A
P 4400 11800
F 0 "#PWR?" H 4400 11550 50  0001 C CNN
F 1 "GND" H 4405 11627 50  0000 C CNN
F 2 "" H 4400 11800 50  0001 C CNN
F 3 "" H 4400 11800 50  0001 C CNN
	1    4400 11800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 11700 4400 11700
Wire Wire Line
	4400 11700 4400 11800
Wire Wire Line
	4100 11500 4800 11500
Wire Wire Line
	6000 11200 6800 11200
Wire Wire Line
	6800 11200 6800 11800
Wire Wire Line
	6000 6600 6590 6600
Wire Wire Line
	6000 6800 6590 6800
Wire Wire Line
	6000 7000 6590 7000
Wire Wire Line
	6000 7200 6590 7200
Wire Wire Line
	6000 7400 6590 7400
$Comp
L power:VCC #PWR?
U 1 1 60C717BA
P 14770 1800
F 0 "#PWR?" H 14770 1650 50  0001 C CNN
F 1 "VCC" H 14785 1973 50  0000 C CNN
F 2 "" H 14770 1800 50  0001 C CNN
F 3 "" H 14770 1800 50  0001 C CNN
	1    14770 1800
	1    0    0    -1  
$EndComp
Connection ~ 14770 1800
Wire Wire Line
	14770 1800 14600 1800
$Comp
L power:VDC #PWR?
U 1 1 60C71D2D
P 11780 1600
F 0 "#PWR?" H 11780 1500 50  0001 C CNN
F 1 "VDC" H 11795 1773 50  0000 C CNN
F 2 "" H 11780 1600 50  0001 C CNN
F 3 "" H 11780 1600 50  0001 C CNN
	1    11780 1600
	1    0    0    -1  
$EndComp
Connection ~ 11780 1600
Wire Wire Line
	11780 1600 11580 1600
Text Label 4700 3640 0    50   ~ 10
BENA+
Text Label 4700 3540 0    50   ~ 10
BDIR+
Text Label 4700 3450 0    50   ~ 10
BPUL+
Text Label 4700 3150 0    50   ~ 10
AENA+
Text Label 4700 3050 0    50   ~ 10
ADIR+
Text Label 4700 2940 0    50   ~ 10
APUL+
Text Label 4700 2640 0    50   ~ 10
ZENA+
Text Label 4700 2540 0    50   ~ 10
ZDIR+
Text Label 4700 2440 0    50   ~ 10
ZPUL+
Text Label 4700 2140 0    50   ~ 10
YENA+
Text Label 4700 2040 0    50   ~ 10
YDIR+
Text Label 4700 1940 0    50   ~ 10
YPUL+
Text Label 4700 1640 0    50   ~ 10
XENA+
Text Label 4700 1540 0    50   ~ 10
XDIR+
Text Label 4700 1440 0    50   ~ 10
XPUL+
$Comp
L power:VCC #PWR?
U 1 1 60DF4D78
P 5050 1140
F 0 "#PWR?" H 5050 990 50  0001 C CNN
F 1 "VCC" H 5065 1313 50  0000 C CNN
F 2 "" H 5050 1140 50  0001 C CNN
F 3 "" H 5050 1140 50  0001 C CNN
	1    5050 1140
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 1140 5050 1440
Connection ~ 5050 1440
Wire Wire Line
	4450 6900 4800 6900
Wire Wire Line
	4450 7100 4800 7100
Wire Wire Line
	4450 9600 4800 9600
$Comp
L Connector_Generic:Conn_01x02 J?
U 1 1 60F1F1D1
P 2250 2400
F 0 "J?" H 2330 2392 50  0000 L CNN
F 1 "Limit_XAxis" H 2330 2301 50  0000 L CNN
F 2 "" H 2250 2400 50  0001 C CNN
F 3 "~" H 2250 2400 50  0001 C CNN
	1    2250 2400
	1    0    0    -1  
$EndComp
Text Label 1700 2400 0    50   ~ 10
XLIM+
Text Label 1700 2500 0    50   ~ 10
XLIM-
Wire Wire Line
	1700 2400 2050 2400
Wire Wire Line
	1700 2500 2050 2500
$Comp
L Connector_Generic:Conn_01x02 J?
U 1 1 60F7D3F0
P 2250 3900
F 0 "J?" H 2330 3892 50  0000 L CNN
F 1 "Limit_YAxis" H 2330 3801 50  0000 L CNN
F 2 "" H 2250 3900 50  0001 C CNN
F 3 "~" H 2250 3900 50  0001 C CNN
	1    2250 3900
	1    0    0    -1  
$EndComp
Text Label 1700 3900 0    50   ~ 10
YLIM+
Text Label 1700 4000 0    50   ~ 10
YLIM-
Wire Wire Line
	1700 3900 2050 3900
Wire Wire Line
	1700 4000 2050 4000
$Comp
L Connector_Generic:Conn_01x02 J?
U 1 1 60F8AE1E
P 2250 5400
F 0 "J?" H 2330 5392 50  0000 L CNN
F 1 "Limit_ZAxis" H 2330 5301 50  0000 L CNN
F 2 "" H 2250 5400 50  0001 C CNN
F 3 "~" H 2250 5400 50  0001 C CNN
	1    2250 5400
	1    0    0    -1  
$EndComp
Text Label 1700 5400 0    50   ~ 10
ZLIM+
Text Label 1700 5500 0    50   ~ 10
ZLIM-
Wire Wire Line
	1700 5400 2050 5400
Wire Wire Line
	1700 5500 2050 5500
$Comp
L Connector_Generic:Conn_01x02 J?
U 1 1 60F91F8B
P 2250 6900
F 0 "J?" H 2330 6892 50  0000 L CNN
F 1 "Limit_AAxis" H 2330 6801 50  0000 L CNN
F 2 "" H 2250 6900 50  0001 C CNN
F 3 "~" H 2250 6900 50  0001 C CNN
	1    2250 6900
	1    0    0    -1  
$EndComp
Text Label 1700 6900 0    50   ~ 10
ALIM+
Text Label 1700 7000 0    50   ~ 10
ALIM-
Wire Wire Line
	1700 6900 2050 6900
Wire Wire Line
	1700 7000 2050 7000
$Comp
L Connector_Generic:Conn_01x02 J?
U 1 1 60F98D2F
P 2250 8400
F 0 "J?" H 2330 8392 50  0000 L CNN
F 1 "Limit_BAxis" H 2330 8301 50  0000 L CNN
F 2 "" H 2250 8400 50  0001 C CNN
F 3 "~" H 2250 8400 50  0001 C CNN
	1    2250 8400
	1    0    0    -1  
$EndComp
Text Label 1700 8400 0    50   ~ 10
BLIM+
Text Label 1700 8500 0    50   ~ 10
BLIM-
Wire Wire Line
	1700 8400 2050 8400
Wire Wire Line
	1700 8500 2050 8500
$Comp
L power:+3V3 #PWR?
U 1 1 60FA9E2C
P 4100 11400
F 0 "#PWR?" H 4100 11250 50  0001 C CNN
F 1 "+3V3" H 4115 11573 50  0000 C CNN
F 2 "" H 4100 11400 50  0001 C CNN
F 3 "" H 4100 11400 50  0001 C CNN
	1    4100 11400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 11400 4800 11400
$Comp
L power:+3V3 #PWR?
U 1 1 60FB1278
P 6400 11600
F 0 "#PWR?" H 6400 11450 50  0001 C CNN
F 1 "+3V3" H 6415 11773 50  0000 C CNN
F 2 "" H 6400 11600 50  0001 C CNN
F 3 "" H 6400 11600 50  0001 C CNN
	1    6400 11600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 11600 6300 11600
Wire Wire Line
	6000 11700 6300 11700
Wire Wire Line
	6300 11700 6300 11600
Connection ~ 6300 11600
Wire Wire Line
	6300 11600 6400 11600
Text Label 6400 1200 2    50   ~ 10
XLIM-
Text Label 6400 1400 2    50   ~ 10
YLIM-
Text Label 6400 1600 2    50   ~ 10
ZLIM-
Text Label 6400 1800 2    50   ~ 10
ALIM-
Text Label 6400 2000 2    50   ~ 10
BLIM-
Wire Wire Line
	6400 1200 6000 1200
Connection ~ 6000 1200
Wire Wire Line
	6400 1400 6000 1400
Wire Wire Line
	6400 1600 6000 1600
Wire Wire Line
	6400 1800 6000 1800
Wire Wire Line
	6400 2000 6000 2000
Wire Notes Line
	1400 1300 1400 2600
Wire Notes Line
	1400 2600 3000 2600
Wire Notes Line
	3000 2600 3000 1300
Wire Notes Line
	3000 1300 1400 1300
Wire Notes Line
	1400 2900 3000 2900
Wire Notes Line
	3000 2900 3000 4100
Wire Notes Line
	3000 4100 1400 4100
Wire Notes Line
	1400 4100 1400 2900
Wire Notes Line
	1400 4400 3000 4400
Wire Notes Line
	3000 4400 3000 5600
Wire Notes Line
	3000 5600 1390 5600
Wire Notes Line
	1390 5600 1390 4400
Wire Notes Line
	1400 5900 3000 5900
Wire Notes Line
	3000 5900 3000 7100
Wire Notes Line
	3000 7100 1400 7100
Wire Notes Line
	1400 7100 1400 5900
Wire Notes Line
	1400 7400 3000 7400
Wire Notes Line
	3000 7400 3000 8610
Wire Notes Line
	3000 8610 1400 8610
Wire Notes Line
	1400 8610 1400 7400
Text Label 6400 2200 2    50   ~ 10
ESTP-
Wire Wire Line
	6400 2200 6000 2200
Connection ~ 6000 2200
Wire Wire Line
	6000 2200 6000 2390
Text Label 4460 9800 0    50   ~ 10
ESTP+
Wire Wire Line
	4800 9800 4460 9800
$Comp
L Connector_Generic:Conn_01x02 J?
U 1 1 61116F30
P 3900 1500
F 0 "J?" H 3980 1492 50  0000 L CNN
F 1 "ESTP" H 3980 1401 50  0000 L CNN
F 2 "" H 3900 1500 50  0001 C CNN
F 3 "~" H 3900 1500 50  0001 C CNN
	1    3900 1500
	1    0    0    -1  
$EndComp
Text Label 3400 1600 0    50   ~ 10
ESTP-
Text Label 3400 1500 0    50   ~ 10
ESTP+
Wire Wire Line
	3400 1500 3700 1500
Wire Wire Line
	3700 1600 3400 1600
$Comp
L Connector_Generic:Conn_01x04 J?
U 1 1 61140DCB
P 10000 4000
F 0 "J?" H 10080 3992 50  0000 L CNN
F 1 "SpindleCOMM" H 10080 3901 50  0000 L CNN
F 2 "" H 10000 4000 50  0001 C CNN
F 3 "~" H 10000 4000 50  0001 C CNN
	1    10000 4000
	1    0    0    -1  
$EndComp
Text Label 9300 3900 0    50   ~ 10
SPSPD
Text Label 9300 4000 0    50   ~ 10
ACOM
Wire Wire Line
	9300 3900 9800 3900
Wire Wire Line
	9300 4000 9800 4000
Text Label 6600 6500 2    50   ~ 10
XLIM+
Text Label 6600 10000 2    50   ~ 10
YLIM+
Text Label 6600 10300 2    50   ~ 10
ZLIM+
Text Label 6600 5800 2    50   ~ 10
RESET+
Text Label 6600 6000 2    50   ~ 10
FEEDHOLD+
Text Label 6600 5900 2    50   ~ 10
CSTART+
Wire Wire Line
	6350 8000 6000 8000
Wire Wire Line
	6350 8200 6000 8200
Wire Wire Line
	6600 8400 6000 8400
Text Label 6600 6300 2    50   ~ 10
PROBE+
Wire Wire Line
	6360 8600 6000 8600
Text Label 7190 7630 2    50   ~ 10
FAN
Text Label 7250 7950 2    50   ~ 10
LIGHTS
Text Label 7250 8170 2    50   ~ 10
FLOOD
Text Label 6600 11000 2    50   ~ 10
MIST
Wire Wire Line
	6600 11000 6000 11000
Text Label 4460 10200 0    50   ~ 10
SPEN
Wire Wire Line
	4460 10200 4800 10200
Text Label 4470 10400 0    50   ~ 10
SPDIR
Wire Wire Line
	4800 10400 4470 10400
Text Label 4460 7300 0    50   ~ 10
ZPUL-
Wire Wire Line
	4460 7300 4800 7300
Wire Wire Line
	6440 9500 6000 9500
Text Label 6600 8400 2    50   ~ 10
SPKR
Text Label 4480 11300 0    50   ~ 10
ATC2
Wire Wire Line
	4480 11300 4800 11300
Text Label 4480 11100 0    50   ~ 10
ATC1
Wire Wire Line
	4480 11100 4800 11100
Wire Wire Line
	2050 6700 1700 6700
Text Label 1700 6700 0    50   ~ 10
AALM-
Text Label 1700 6600 0    50   ~ 10
AALM+
Wire Wire Line
	4450 6700 4800 6700
Wire Wire Line
	4450 6500 4800 6500
Text Label 4450 6700 0    50   ~ 10
AALM+
Text Label 6600 10500 2    50   ~ 10
BALM+
Wire Wire Line
	4800 7500 4450 7510
Wire Wire Line
	4450 7700 4800 7700
Text Label 6600 6200 2    50   ~ 10
INPUT+
Wire Wire Line
	4460 10000 4800 10000
Wire Wire Line
	6600 5900 6000 5900
Text Label 6600 6100 2    50   ~ 10
SAFETYDOOR+
Wire Wire Line
	6600 6100 6000 6100
Wire Wire Line
	6600 6500 6000 6500
Wire Wire Line
	6600 6300 6000 6300
Wire Wire Line
	6600 10000 6000 10000
Wire Wire Line
	6600 10300 6000 10300
Text Label 6600 10200 2    50   ~ 10
ALIM+
Text Label 4450 7700 0    50   ~ 10
BLIM+
$Comp
L STM32Boards:BlackBoard BLB?1
U 1 1 60628595
P 5400 5350
F 0 "BLB?1" H 5410 5130 50  0000 C CNN
F 1 "BlackBoard" H 5400 5240 50  0000 C CNN
F 2 "" H 4750 6550 50  0001 C CNN
F 3 "" H 4750 6550 50  0001 C CNN
	1    5400 5350
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6600 10500 6000 10500
Wire Wire Line
	6600 10200 6000 10200
$EndSCHEMATC
