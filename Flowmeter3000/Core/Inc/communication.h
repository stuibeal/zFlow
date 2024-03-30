/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : communication.h
 * @brief          : Defines für I2C Kommunication
 *                   Sollte für alle uC verwendbar sein
 ******************************************************************************
 * @attention
 *
 * 2022 Z.Gesellschaft / Zapfapparat 0.4
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/**
 * Grundlegend I2C Adressen
 */
#define flowi2c                 0x12   // Adresse vom Flow (in Cube linksshiften 1 Bit!: 36(dez))
#define tempi2c                 0x13   // Adresse vom Temp (in Cube linksshiften 1 Bit!: 38(dez))

/*
 *  Definitionen für Bytes
 *  FLOWMETER+TEMPERATUR
 *  										ALLES 2 Bytes -> 1 uint16_t
 *  							MASTER		SLAVE			MASTER
 *								SEND		RETURN			SEND
 *******************************************************************************/

#define ebiMode               	0xF9      //1 an, 0 aus, Temperatur auf 2°C, Hahn auf, Zapfmusik
#define beginZapf             	0xFA      //Beginn das Zapfprogramm -> PID auf aggressiv
#define endZapf              	0xFB      //zapfMillis
#define kurzBevorZapfEnde     	0xFC      //sagt das wir kurz vor Ende sind → Valve schließen -> PID auf konservativ
#define lowEnergy             	0xFD      //LEDS nicht benutzen  byte2: 0: vollgas   1: sparen
#define wachAuf               	0xFE      //mach was
#define zapfenStreich         	0xFF      //Valve Schließen, LEDs aus


/*
 *  Definitionen für Bytes
 *  FLOWMETER
 *
 *  Der Flowmeter schickt IMMER die aktuellen zapfMillis zurück.
 *  Diese werden dann vom Master durch setUserMilliLitres oder beginZapf wieder
 *  auf 0 zurückgestellt.
 *
 *  								ALLES 2 Bytes -> 1 uint16_t
 *  							MASTER		SLAVE			MASTER
 *								SEND		RETURN			SEND
 *******************************************************************************/
#define sendMilliLitres       	0x01      //zapfMillis		2 Bytes egal was
#define setUserMilliLitres    	0x21      //				userMilliLiter
#define makeFunWithLeds1      	0x22      //zapfMillis		Laufzeit (0-255), Delay (0-255)
#define makeFunWithLeds2      	0x23      //zapfMillis
#define makeFunWithLeds3      	0x24      //zapfMillis
#define makeFunWithLeds4      	0x25      //zapfMillis		byte1: Leds byte2: helligkeit

/*
 *  Definitionen für Bytes
 *  TEMPERATURREGLER						ALLES 2 Bytes -> 1 uint16_t
 *  							MASTER		SLAVE			MASTER 		MASTER
 *								SEND		RETURN			SEND		Variable
 *******************************************************************************/
#define transmitBlockTemp       0x40      //blockTemp in °C*10
#define transmitAuslaufTemp     0x41  // Data send:   hahnTemp in °C*10
#define transmitPower           0x42  // Data send:   Leistung in W (power1+power2)    
#define transmitInVoltage       0x43  // Data send:   inVoltage in V*100   
#define transmitKuehlFlow       0x44  // Data send:   Durchfluss Kühlwasser (extern) pro 10000ms

#define setHighTemperatur       0x60  // Data get: Zieltemperatur Block * 100 (2°C)
#define setMidTemperatur        0x61  // Data get: Normale Temperatur in °C * 100 (6°C)
#define setLowTemperatur        0x62  // Data get: Energiespar Temperatur * 100 (9°C)
#define setMinCurrent           0x63  // Data get: Current in mA / 10 (11 = 0,11 A), Untere Regelgrenze
#define setLowCurrent           0x64  // Data get: current in mA / 10, Obere Regelgrenze bei wenig Strom
#define setMidCurrent           0x65  // Data get: Current in mA / 10, Obere Regelgrenze bei normalem Strom
#define setHighCurrent          0x66  // Data get: Current in mA / 10, Obere Regelgrenze bei gutem Strom 

#define setNormVoltage          0x68  // Data get: norm Voltage * 100, passt normal, mehr als 9V macht wenig Sinn bei den Peltierelementen
#define setMaxVoltage           0x69  // Data get: max Voltage * 100, das wäre dann eigentlich die Batteriespannung
#define setLowBatteryVoltage    0x6A  // 11V Eingangsspannung 
#define setMidBatteryVoltage    0x6B  // 12V Eingangsspannung
#define setHighBatteryVoltage   0x6C  // 13V Eingangsspannung

#define setWasserTemp           0x6D  // Data get: kühlwasserTemp in °C*100 vom DS18B20 Sensor vom Master: Fühler neben Peltier
#define setEinlaufTemp          0x6E  // Data get: Biertemperatur in °C*100 vom DS18B20 Sensor vom Master: Bierzulauf       

#define setConsKp               0x70  // Data get: konservativer Kp
#define setConsKi               0x71  // Data get: konservativer Ki
#define setConsKd               0x72  // Data get: konservativer Kd
#define setAggKp                0x73  // Data get: aggressiver Kp
#define setAggKi                0x74  // Data get: aggressiver Ki
#define setAggKd                0x75  // Data get: aggressiver Kd
#define setUnterschiedAggPid    0x75  // mal zehn grad nehmen ab wann der aggressiv regelt
#define setSteuerZeit           0x76  // alle sekunde mal nachjustieren


/*
 I2C Adressen

 uC          Adresse (Dez) Bin
 Flowmeter   0x12    18
 Temperatur  0x13    19
 PCA9685
 DCF77       0x6D    109   1101101

 400 khz betrieb

 Der Master schickt immer ein Byte an den Slave
 Der Slave schickt dann zwei Bytes zurück
 oder der Master schickt zwei Bytes (int) an den Slave

 Also schaut das Paket so aus: i2c_addr, todobyte, highbyte, lowbyte

 Grundlegend:
 01-0F der Master will Daten vom Flow
 20-2F der Master schickt Daten an Flow
 40-4F der Master will Daten vom Temp
 60-6F der Master schickt Daten an Temp
 F0-FF mach was, für Beide

 send      send    return    return
 i2c Addr  toDo  HighByte  Lowbyte HighByte  Lowbyte

 0x12 Flowmeter
 sendMilliLitres 0x01      milliliter

 setCalibrationFactor  0x20  factor
 setUserMilliLitres  0x21  userMilliLitres
 makeFunWithLeds 0x22  zeitinMillis
 makeFunWithLeds 0x23  zeitinMillis
 setPulseProMl 0x24  Pulse pro Ml



 beginZapf 0xFA
 endZapf 0xFB      milliliter
 kurzBevorZapfEnde 0xFC          sagt das wir kurz vor Ende sind → Valve schließen
 lowEnergy 0xFD          LEDS nicht benutzen
 wachAuf 0xFE          mach was
 zapfenStreich 0xFF          Valve Schließen, LEDs aus


 send    return
 i2c Addr  toDo  HighByte  Lowbyte HighByte  Lowbyte
 0x13 Temperatur
 blockTemp 0x40      blockTemp in °C*10
 hahnTemp  0x41      hahnTemp in °C*10
 usedPower 0x42      Leistung in Ws
 inVoltage 0x43      inVoltage in mV


 setZielTemperatur 0x61  Temperatur * 10
 setNormalTemp 0x62  temp in °C*10       normale Zieltemperatur
 setMaxCurrent 0x63  Current in mA
 setMinCurrent 0x64  Current in mA
 setMaxVoltage 0x65  voltage in mV
 setShutDownVoltage  0x66  mV ab wann der abschaltet
 setPID-P  0x67          Proportionalfaktor der PID Steuerung
 setPID-I  0x68          Integralfaktor der PID
 setPID-D  0x69          Differentialfaktor der PID
 wasserTemp  0x6A  kühlwasserTemp in °C*10
 einlaufTemp 0x6B  Biertemperatur in °C*10



 beginZapf 0xFA
 endZapf 0xFB
 kurzBevorZapfEnde 0xFC          sagt das wir kurz vor Ende sind → Regelung auf normal runterfahren
 lowEnergy 0xFD          warmes Bier ihr Lumpen
 wachAuf 0xFE          Ruidengs hochfahren,
 zapfenStreich 0xFF          Ruidengs auf Minimal , Display low, Current auf Null
 */
