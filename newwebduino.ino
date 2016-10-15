#include <Time.h>
#include <TimeLib.h>

#include "wol.h"

#include <ICMPPing.h>
#include <Ethernet.h>
#include <Time.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include <avr/wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <utility/w5100.h>

void initEPROM();
int memoryFree();
void getPostdata(char *data, int maxposdata);
int parsePostdata(const char* data, const char* param, char* value);
void getStatus(char* GETparam);
float readTemperature();
float getAverageTemperature();
void readEPROM();
void writeEPROM();
boolean sendStatus();
int findIndex(const char* data, const char* target);
void showMain(boolean isPost, char* param);
void showChangeSettings(char* GETparam);
void showPower(char* GETparam);
void setRemoteTemperature(char* GETparam);
bool sendNotification(int type, int value);
bool post(char* host, int port, char* path, char* param);


#define Versione "2.12"

IPAddress pingAddr(192, 168, 1, 1); // ip address to ping

// Il terminale data del sensore � connesso
// alla porta 2 di Arduino
#define ONE_WIRE_BUS 3
// Imposta la comunicazione oneWire per comunicare
// con un dispositivo compatibile
OneWire oneWire(ONE_WIRE_BUS);
// Passaggio oneWire reference alla Dallas Temperature. 
DallasTemperature dallasSensors(&oneWire);

const int relePin = 2; // rel� pin
const bool RELE_ON = HIGH;//LOW
const bool RELE_OFF = LOW;//LOW
bool releStatus = false;
bool oldReleStatus = false;
int activeProgram = -1;
int activeTimerange = -1;


const int maxposdataChangeSetting = 150;
char databuff[maxposdataChangeSetting];

#include <avr/pgmspace.h> // for progmem
#define P(name) static const prog_uchar name[] PROGMEM // declare a static string
const byte EEPROM_ID = 0x99; // used to identify if valid data in EEPROM
const int ID_ADDR = 0; // the EEPROM address used to store the ID
const int TIMERTIME_ADDR = 1; // the EEPROM address used to store the pin

const char SWVERSION[] = "1.01";

byte mac[] = { 0x00, 0xA0, 0xB0, 0xC0, 0xD0, 0x90 };
byte ip[] = { 192, 168, 1, 95 };
byte id = 0;

byte dhcp = 1;
int localPort = 80;
const int servernamelen = 30;
char servername[servernamelen];
int serverPort = 90;


const int MAX_PAGE_NAME_LEN = 12;
const int MAX_PARAM_LEN = 12;//12;

const int maxposdata = 101; // massimo numero di caratteri della variabile posdata
const int maxvaluesize = maxposdata - 1/*10*/; // massimo numero di digit del valore di una variabile nel POS data

EthernetServer server = EthernetServer(80);
EthernetClient client, client2;

float localTemperature = 0;
float localAvTemperature = 0;
float oldLocalAvTemperature = 0;
bool statusChangeSent = true;
const int avTempsize = 10;
float avTemp[avTempsize];
unsigned int avTempCounter = 0;
float targetTemperature = 0.0;
float remoteTemperature = 0;

unsigned long lastFlash = 0;
const int flash_interval = 30;
unsigned long lastStatusChange = 0;
const int lastStatusChange_interval = 600; // timeout retry invio status change
unsigned long lastNotification = 0;
const int Notification_interval = 20;

SOCKET pingSocket = 0;
ICMPPing ping(pingSocket, (uint16_t)random(0, 255));
int offlineCounter = 0;

unsigned long last_RemoteSensor = 0;
unsigned long remoteSensorTimeout = 360; // tempo dopo il quale il programa ssi disattiva

//unsigned long last_ConsumptionCounter = 0;


// variables created by the build process when compiling the sketch
extern int __bss_end;
extern void *__brkval;

const int temperaturePin = 3;

char* boolStr[] = { "false", "true" };

char* statusStr[] = { "unused", "idle", "program", "manual", "disabled", "restarted" };
#define STATUS_IDLE            1
#define STATUS_PROGRAMACTIVE   2
#define STATUS_MANUAL           3
#define STATUS_DISABLED        4
#define STATUS_RESTARTED        5

#define			notification_statuschange		1
#define			notification_restarted			2
//#define			notification_programstart		3 // no serve, usare status change
#define			notification_programend			3
#define			notification_relestatuschange	4
#define			notification_offline			5
//#define			notification_manualend			6

#define			relestatus_off		0
#define			relestatus_on		1
#define			relestatus_disabled	2
#define			relestatus_enabled	3

int currentStatus = STATUS_IDLE;
int oldcurrentstatus = currentStatus;

bool netwokStarted = false;
time_t programStartTime = 0;
time_t programDuration = 30;

#define     sensor_local    0
#define     sensor_remote   1
int temperatureSensor = sensor_local;

int sendRestartNotification = 0;

unsigned long totalConsumptionTime = 0;
unsigned long lastConsumptionEnableTime = 0;
unsigned long ConsumptionStartTime;

void enableRele(boolean on);


void setup()
{
	Serial.begin(9600);
	delay(500);

	Serial.println(F("starting....x"));
	String str = "NETSWITCH - Version ";
	str += Versione;
	Serial.println(str);

	// rele
	pinMode(relePin, OUTPUT);
	enableRele(false);
	ConsumptionStartTime = now();


	//Serial.println(F("starting...."));
	wdt_disable();
	wdt_enable(WDTO_8S);

	initEPROM();

	Serial.println(F("MAC: "));
	char buf[50];
	sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	Serial.print(buf);

	//byte ip[] = { 192, 168, 1, 96 };

	Ethernet.begin(mac, ip);


	sprintf(buf, "%d.%d.%d.%d:%d\n", ip[0], ip[1], ip[2], ip[3], localPort);
	Serial.print(buf);

	server = EthernetServer(localPort);

	server.begin();

	netwokStarted = true;

	//Serial.print(F("----->Memory free: setup "));
	//Serial.println(memoryFree());

	currentStatus = STATUS_IDLE;


	Serial.print(F("<---Memory free: setup "));
	Serial.println(memoryFree());
}



void enableRele(boolean on) {

	if (releStatus) {
		totalConsumptionTime += (now() - lastConsumptionEnableTime);
	}

	oldReleStatus = releStatus;
	if (on) {
		Serial.println(F("rele on"));
		digitalWrite(relePin, RELE_ON/*HIGH*/);
		releStatus = true;

		lastConsumptionEnableTime = now();
	}
	else {
		Serial.println(F("rele off"));
		digitalWrite(relePin, RELE_OFF/*LOW*/);
		releStatus = false;
	}

}

void showwol(char* GETparam) {

	//Serial.println(F("showwol "));
	//int ntimer = parsePostdata(GETparam, "prg");
	//int ret = parseChangeTimer(ntimer);

	wol* w = new wol();
	w->init();
	w->wakeup();
	delete w;

	client.println(F("HTTP/1.1 200 OK\r\nContent-Type: text/html"));
	//client.println();
	client.println(F("\n\n<html><head><meta HTTP-EQUIV='REFRESH' content='0; url=/main?msg=2'><title>WOL</title></head><body></body></html>"));


}

void showRele(char* GETparam) {

	Serial.println(F("showRele "));

	getPostdata(databuff, maxposdataChangeSetting);
	char posdata[maxposdata];
	int val;
	// status
	val = parsePostdata(databuff, "status", posdata);
	Serial.print(F("status="));
	Serial.println(val, DEC);
	// duration
	long duration = parsePostdata(databuff, "duration", posdata);
	Serial.print(F("duration="));
	Serial.println(duration, DEC);
	duration = duration * 60;
	Serial.print(F("duration * 60="));
	Serial.println(duration, DEC);
	// temperature
	int res = parsePostdata(databuff, "target", posdata);
	//if (res != -1) {
	String str = "";
	str += posdata;
	targetTemperature = str.toFloat();
	Serial.print(F("targetTemperature="));
	Serial.println(targetTemperature, DEC);
	// sensor
	temperatureSensor = parsePostdata(databuff, "sensor", posdata);
	Serial.print(F("sensor="));
	Serial.println(temperatureSensor, DEC);
	if (temperatureSensor == sensor_remote) {
		last_RemoteSensor = now();
		Serial.print(F("last_RemoteSensor="));
		Serial.println(last_RemoteSensor, DEC);
	}
	// remote temperature
	res = parsePostdata(databuff, "temperature", posdata);
	if (res != -1) {
		str = "";
		str += posdata;
		remoteTemperature = str.toFloat();
		Serial.print(F("remoteTemperature="));
		Serial.println(remoteTemperature, DEC);
	}
	else {
		Serial.print(F("remoteTemperature MISSING"));
	}
	// program
	int program = parsePostdata(databuff, "program", posdata);
	Serial.print(F("program="));
	Serial.println(program, DEC);
	int timerange = parsePostdata(databuff, "timerange", posdata);
	Serial.print(F("timerange="));
	Serial.println(timerange, DEC);
	// manual
	int manual = parsePostdata(databuff, "manual", posdata);
	Serial.print(F("manual="));
	Serial.println(manual, DEC);
	// jsonRequest
	int json = parsePostdata(databuff, "json", posdata);
	Serial.print(F("json="));
	Serial.println(json, DEC);

	Serial.print(F("currentStatus="));
	Serial.println(currentStatus, DEC);
	if (val == relestatus_on && currentStatus != STATUS_DISABLED) {

		if (manual != 1) {
			Serial.print(F("rele on"));
			if (currentStatus != STATUS_MANUAL) {
				Serial.print(F("not manual"));
				enableRele(true);
				currentStatus = STATUS_PROGRAMACTIVE;
				if (duration != -1)
					programDuration = duration;
				else
					programDuration = 30;
				programStartTime = now();
				activeProgram = program;
				activeTimerange = timerange;
			}
		}
		else if (manual == 1) {
			Serial.print(F("manual"));
			enableRele(true);
			currentStatus = STATUS_MANUAL;
			if (duration != -1)
				programDuration = duration;
			else
				programDuration = 30;
			programStartTime = now();

		}
	}
	else if (val == relestatus_off) {
		Serial.print(F("rele off"));
		if (currentStatus == STATUS_MANUAL) { // il progr aut è finito ma il disp è in manual mode
			if (manual == 1) {
				Serial.print(F("manual stop"));
				enableRele(false);
				currentStatus = STATUS_IDLE;
			}
			else {
				Serial.print(F("manual mode, impossibile fermare"));
			}
		}
		else if (currentStatus == STATUS_PROGRAMACTIVE) {
			Serial.print(F("program active rele off"));
			enableRele(false);
			activeProgram = program;
			activeTimerange = timerange;
		}
		else if (currentStatus == STATUS_IDLE) {

			if (manual != 1) {
				Serial.print(F("rele off"));
				Serial.print(F("not manual"));
				enableRele(false);
				currentStatus = STATUS_PROGRAMACTIVE;
				if (duration != -1)
					programDuration = duration;
				else
					programDuration = 30;
				programStartTime = now();
				activeProgram = program;
				activeTimerange = timerange;
			}
		}
	}
	else if (val == relestatus_disabled) {
		enableRele(false);
		currentStatus = STATUS_DISABLED;
	}
	else if (val == relestatus_enabled) {
		enableRele(false);
		currentStatus = STATUS_IDLE;
	}

	if (json == 1) {
		getStatus(NULL);
	}
	else {
		client.println(F("HTTP/1.1 200 OK\r\nContent-Type: text/html"));
		client.println(F("\n\n<html><head><meta HTTP-EQUIV='REFRESH' content='0; url=/main?msg=2'><title>rele</title></head><body>"));

		char   buffer[50];
		sprintf(buffer, "relestatus=%d,status=%s", releStatus, statusStr[currentStatus]);
		client.print(buffer);

		client.println(F("</body></html>"));
	}



	/**/
}


void flash() {

	Serial.println(F(""));
	Serial.println(F("FLASH............ "));

	time_t currentTime = now();



	localTemperature = readTemperature();
	localAvTemperature = getAverageTemperature();

	Serial.print(" ConsumptionStartTime=");
	Serial.println(ConsumptionStartTime);
	Serial.print(" totalConsumptionTime=");
	Serial.println(totalConsumptionTime);
	Serial.print(" lastConsumptionEnableTime=");
	Serial.println(lastConsumptionEnableTime);

	Serial.print(" targetTemperature=");
	Serial.println(targetTemperature);
	Serial.print("localTemperature=");
	Serial.println(localTemperature, DEC);
	Serial.print("sensor_local=");
	Serial.println(sensor_local);
	Serial.print(" localAvTemperature=");
	Serial.println(localAvTemperature);
	Serial.print(" oldLocalAvTemperature=");
	Serial.println(oldLocalAvTemperature);
	Serial.print("active temperatureSensor=");
	Serial.println(temperatureSensor, DEC);
	Serial.print(" remoteTemperature=");
	Serial.println(remoteTemperature);

	if (currentStatus == STATUS_MANUAL) {

		Serial.print("STATUS_MANUAL");
		// se stato manuale accendi il relè se sensore == locale e temperatura sensore locale < temperatura target oppure
		// se sensore == remoto e temperature sensore remoto < temperatura target 
		if ((temperatureSensor == sensor_local && localTemperature < targetTemperature) ||
			(temperatureSensor == sensor_remote && remoteTemperature < targetTemperature)) {

			Serial.println(F("-LOW TEMPERATURE"));
			enableRele(true);
		}
		else {
			Serial.println(F("-HIGH TEMPERATURE"));
			enableRele(false);
		}
	}
	else if (currentStatus == STATUS_PROGRAMACTIVE) {

		Serial.print("STATUS_PROGRAMACTIVE");

		if (temperatureSensor != sensor_local) {
			Serial.print(F("-REMOTE SENSOR")); // non modificare stato relke, controllato remotamenre

		}
		else if (temperatureSensor == sensor_local) {
			Serial.print(F("-LOCAL SENSOR"));
			if (localTemperature < targetTemperature) {

				Serial.println(F("-LOW TEMPERATURE"));
				enableRele(true);
			}
			else {
				Serial.println(F("-HIGH TEMPERATURE"));
				enableRele(false);
			}
		}
	}
	else{
		Serial.println("INACTIVE-rele OFF");
		enableRele(false);
	}
}

void initEPROM()
{

	byte id = EEPROM.read(ID_ADDR); // read the first byte from the EEPROM
	if (id == EEPROM_ID)
	{
		readEPROM();
	}
	else
	{
		writeEPROM();
	}
}

void writeEPROM() {

	//Serial.println("write eprom");

	EEPROM.write(ID_ADDR, EEPROM_ID); // write the ID to indicate valid data
	byte hiByte;
	byte loByte;

	int addr = TIMERTIME_ADDR;
	//mac
	EEPROM.write(addr++, mac[0]);
	EEPROM.write(addr++, mac[1]);
	EEPROM.write(addr++, mac[2]);
	EEPROM.write(addr++, mac[3]);
	EEPROM.write(addr++, mac[4]);
	EEPROM.write(addr++, mac[5]);
	// local port
	EEPROM.write(addr++, localPort);
	// server name
	int res = EEPROM_writeAnything(addr, servername);
	addr += res;
	// server port
	hiByte = highByte(serverPort);
	loByte = lowByte(serverPort);
	EEPROM.write(addr++, hiByte);
	EEPROM.write(addr++, loByte);

	// lopcal ip
	EEPROM.write(addr++, ip[0]);
	EEPROM.write(addr++, ip[1]);
	EEPROM.write(addr++, ip[2]);
	EEPROM.write(addr++, ip[3]);

	// id
	EEPROM.write(addr++, id);



}
void readEPROM() {

	//Serial.println("read eprom");

	byte hiByte;
	byte lowByte;

	int addr = TIMERTIME_ADDR;

	// mac
	mac[0] = EEPROM.read(addr++);
	mac[1] = EEPROM.read(addr++);
	mac[2] = EEPROM.read(addr++);
	mac[3] = EEPROM.read(addr++);
	mac[4] = EEPROM.read(addr++);
	mac[5] = EEPROM.read(addr++);
	// local port
	localPort = EEPROM.read(addr++);
	//server name
	int res = EEPROM_readAnything(addr, servername);
	addr += res;
	// server port
	hiByte = EEPROM.read(addr++);
	lowByte = EEPROM.read(addr++);
	serverPort = word(hiByte, lowByte);
	// local ip
	ip[0] = EEPROM.read(addr++);
	ip[1] = EEPROM.read(addr++);
	ip[2] = EEPROM.read(addr++);
	ip[3] = EEPROM.read(addr++);

	// id
	id = EEPROM.read(addr++);

}


int memoryFree()
{
	int freeValue;

	if ((int)__brkval == 0)
		freeValue = ((int)&freeValue) - ((int)&__bss_end);
	else
		freeValue = ((int)&freeValue) - ((int)__brkval);
	return freeValue;
}



bool pingServer() {

	Serial.println(F("ping server ...."));
	for (int i = 0; i < 4; i++) {
		Serial.print(pingAddr);
		Serial.print(".");
	}
	ICMPEchoReply echoReply = ping(pingAddr, 4);
	if (echoReply.status == SUCCESS)
	{
		offlineCounter = 0;
		Serial.println("ping success");
		return true;
	}
	else
	{
		Serial.println("ping unsuccess");
		offlineCounter++;

		if (offlineCounter > 15) {
			//reset
			for (int i = 0; i >= 0; i++)
			{
				;
			}
		}
		return false;
	}
}

void loop()
{
	wdt_enable(WDTO_8S);

	if (oldReleStatus != releStatus) {
		char buf[10];
		sprintf(buf, "relestatus=%d", releStatus);
		//sendNotification(notification_relestatuschange, releStatus);
		sendStatus();
		oldReleStatus = releStatus;
		return;
	}
	//
	if (currentStatus != oldcurrentstatus) {
		if (currentStatus == STATUS_DISABLED) {
			char buf[10];
			sprintf(buf, "status=%d", 0);
			//sendNotification(notification_statuschange, currentStatus);
			sendStatus();
		}
		oldcurrentstatus = currentStatus;
		return;
	}

	unsigned long currMillis = now();


	if (currentStatus == STATUS_PROGRAMACTIVE || currentStatus == STATUS_MANUAL) {

		if (temperatureSensor != sensor_local && (currMillis - last_RemoteSensor) > remoteSensorTimeout) {
			Serial.println("REMOTE SENSOR TIMEOUT");
			//sendNotification(notification_programend, (currMillis - last_RemoteSensor));
			enableRele(false);
			// è iniutile mandare un sendstatus perchè tanto cambia lo stato dopo e verrebbe inviato due volte
			remoteTemperature = 0;
			currentStatus = STATUS_IDLE;
			return;

		}
		else if (currMillis - programStartTime > programDuration) {
			Serial.print("END PROGRAM");
			//sendNotification(notification_programend, 1);
			enableRele(false);
			// è iniutile mandare un sendstatus perchè tanto cambia lo stato dopo e verrebbe inviato due volte
			currentStatus = STATUS_IDLE;
			return;

		}
	}

	if (currMillis - lastFlash > flash_interval) {
		lastFlash = currMillis;
		flash();
		return;
	}

	if ((oldLocalAvTemperature != localAvTemperature && statusChangeSent == true) || (statusChangeSent == false && (currMillis - lastStatusChange) > lastStatusChange_interval)) {


		Serial.println("");
		Serial.println("SEND STATUS");
		Serial.print(" oldLocalAvTemperature=");
		Serial.println(oldLocalAvTemperature, DEC);
		Serial.print(" localAvTemperature=");
		Serial.println(localAvTemperature, DEC);
		Serial.print(" statusChangeSent=");
		Serial.println(statusChangeSent);

		Serial.print(" currMillis=");
		Serial.println(currMillis, DEC);
		Serial.print(" lastStatusChange=");
		Serial.println(lastStatusChange, DEC);


		if (sendStatus()) {
			statusChangeSent = true;
			oldLocalAvTemperature = localAvTemperature;
		}
		else {
			statusChangeSent = false;
			lastStatusChange = currMillis;
		}
		return;
	}


	if (currMillis - lastNotification > Notification_interval) {

		lastNotification = currMillis;
		pingServer();
		return;
	}

	char buffer[MAX_PAGE_NAME_LEN + 1]; // additional character for terminating null
	char parambuffer[MAX_PARAM_LEN];

	client = server.available();
	if (client) {

		int type = 0;
		while (client.connected()) {

			if (client.available()) {
				// GET, POST, or HEAD
				memset(buffer, 0, sizeof(buffer)); // clear the buffer
				memset(parambuffer, 0, sizeof(parambuffer));
				if (client.readBytesUntil('/', buffer, MAX_PAGE_NAME_LEN)){
					Serial.println(buffer);
					if (strcmp(buffer, "GET ") == 0)
						type = 1;
					else if (strcmp(buffer, "POST ") == 0)
						type = 2;
					// look for the page name
					memset(buffer, 0, sizeof(buffer)); // clear the buffer
					int l;

					if (l = client.readBytesUntil(' ', buffer, MAX_PAGE_NAME_LEN))
					{
						Serial.println(l, DEC);
						Serial.println(buffer);
						l = findIndex(buffer, "?");
						int i = 0;
						if (l != -1) {
							while ((l + i) < MAX_PAGE_NAME_LEN && i < MAX_PARAM_LEN) {
								parambuffer[i] = buffer[l + i];
								i++;
							}
							buffer[l] = '\0';
						}
						else {
							;
						}
						//Serial.println(l, DEC);
						//Serial.println(buffer);
						//Serial.println(parambuffer);
						//Serial.println("-");

						if (strcmp(buffer, "main") == 0)
							showMain(type == 2, parambuffer);
						else if (strcmp(buffer, "chstt") == 0)
							showChangeSettings(parambuffer);
						else if (strcmp(buffer, "wol") == 0)
							showwol(parambuffer);
						else if (strcmp(buffer, "rele") == 0)
							showRele(parambuffer);
						else if (strcmp(buffer, "power") == 0)
							showPower(parambuffer);
						else if (strcmp(buffer, "temp") == 0)
							setRemoteTemperature(parambuffer);
						else if (strcmp(buffer, "status") == 0)
							getStatus(parambuffer);
						/*else
						unknownPage(buffer);*/
					}
				}
				break;
			}
		}
		// give the web browser time to receive the data
		delay(20);
		client.stop();
	}

	if (sendRestartNotification < 10) {
		pingServer(); // per qualche motivo se non faccio prima ping la notification non funziona e non si può mettere tutto in setup()
		Serial.print(F("SEND RESTART NOTIFICATION "));
		if (sendNotification(notification_restarted, 0))
			sendRestartNotification = 10;
		else
			sendRestartNotification++;
		return;
	}
}



float readTemperature(){

	// Dallas sensor
	dallasSensors.requestTemperatures(); // Invia il comando di lettura delle temperatura
	float dallasTemperature = dallasSensors.getTempCByIndex(0);

	//int temp = 10 * (dallasTemperature + 0.05);

	if (avTempCounter > avTempsize)
		avTempCounter = 0;
	avTemp[avTempCounter++] = dallasTemperature;

	//Serial.print(F("temp = "));
	//Serial.println(dallasTemperature, DEC);

	return dallasTemperature;
}

float getAverageTemperature() { // return 10 * average temp 

	float average = 0;

	for (int i = 0; i < avTempsize; i++) {
		average += avTemp[i];
	}
	average = average / avTempsize;

	return average;
}

void showMain(boolean isPost, char* param)
{
	Serial.print(F("-->Memory free: showmain "));
	Serial.println(memoryFree());

	char   buffer[100];

	client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE html><html><head><title>Webduino</title><body>\r\n"));

	// comandi
	client.println(F("\n<font color='#53669E' face='Verdana' size='2'><b>Webduino</b></font>\r\n<table width='80%' border='1'><colgroup bgcolor='#B6C4E9' width='20%' align='left'></colgroup><colgroup bgcolor='#FFFFFF' width='30%' align='left'></colgroup>"));

	// power
	client.print(F("<tr><td>Power "));
	if (currentStatus == STATUS_DISABLED) {

		client.print(F("OFF</td><td><form action='/power' method='POST'>"));
		client.print(F("<input type='radio' name='status' value='0' >Off<input type='radio' name='status' value='1' checked>On<BR>"));
	}
	else {
		client.print(F("ON</td><td><form action='/power' method='POST'>"));
		client.print(F("<input type='radio' name='status' value='0' checked >Off<input type='radio' name='status' value='1' >On<BR>"));
	}
	client.print(F("<input type='submit' value='set'></form>"));
	client.print(F("</td></tr>"));

	// status
	sprintf(buffer, "<tr><td>Status: </td><td>%s</td></tr>", statusStr[currentStatus]);
	client.print(buffer);

	// rele
	client.print(F("<tr><td>Rele </td><td>"));
	if (releStatus) {
		client.print(F("on - "));

		time_t onStatusDuration = (now() - programStartTime);
		time_t remainingTime = programDuration - onStatusDuration;// (now() - programStartTime);

		time_t hr = remainingTime / 3600L;
		time_t mn = (remainingTime - hr * 3600L) / 60L;
		time_t sec = (remainingTime - hr * 3600L) % 60L;
		sprintf(buffer, " remaining:(%2d) ", remainingTime);
		client.print(buffer);
		sprintf(buffer, "%02d:", hr);
		client.print(buffer);
		sprintf(buffer, "%02d:", mn);
		client.print(buffer);
		sprintf(buffer, "%02d", sec);
		client.print(buffer);

		hr = onStatusDuration / 3600L;
		mn = (onStatusDuration - hr * 3600L) / 60L;
		sec = (onStatusDuration - hr * 3600L) % 60L;
		sprintf(buffer, " acceso da:(%2d) ", onStatusDuration);
		client.print(buffer);
		sprintf(buffer, "%02d:", hr);
		client.print(buffer);
		sprintf(buffer, "%02d:", mn);
		client.print(buffer);
		sprintf(buffer, "%02d", sec);
		client.print(buffer);

		client.print(F("</td></tr>"));

	}
	else {
		client.print(F("off</td></tr>"));
	}

	// program
	client.print(F("<tr><td>program </td><td>"));
	if (currentStatus == STATUS_PROGRAMACTIVE) {
		client.print(F("program "));
		client.print(activeProgram);
		client.print(F("timerange "));
		client.print(activeTimerange);

		sprintf(buffer, " Target: %d.%02d", (int)targetTemperature, (int)(targetTemperature * 100.0) % 100);
		client.print(buffer);
		client.print(F(" Sensor: "));
		if (temperatureSensor == sensor_local) {
			client.print(F("Local"));
		}
		else if (temperatureSensor == sensor_remote) {
			client.print(F(" Remote"));
		}

		client.print(F("</td></tr>"));

	}
	else if (currentStatus == STATUS_MANUAL) {
		client.print(F("manual program"));

		client.print(F("</td></tr>"));
	}


	// Manual
	client.print(F("<tr><td>Manual "));
	if (currentStatus == STATUS_MANUAL) {


		sprintf(buffer, "ON</td><td>Target: %d.%02d<BR>", (int)targetTemperature, (int)(targetTemperature * 100.0) % 100);
		client.print(buffer);

		if (temperatureSensor == sensor_local) {
			client.print(F("Sensor: Local"));
		}
		else if (temperatureSensor == sensor_remote) {
			client.print(F("Sensor: Remote"));

			//client.print(buffer);
		}
		client.print(F("<form action='/rele' method='POST'>"));
		client.print(F("<input type='hidden' name='manual' value='1'>")); ///manual
		client.print(F("<input type='hidden' name='status' value='0'>"));
		client.print(F("<input type='submit' value='stop'></form>"));
	}
	else if (currentStatus == STATUS_PROGRAMACTIVE || currentStatus == STATUS_IDLE) {
		client.print(F("OFF</td><td><form action='/rele' method='POST'><input type='hidden' name='status' value='1'>"));
		client.print(F("Minutes:<input type='num' name='duration' value='"));

		client.print(30);
		client.print(F("' size='5' maxlength='5'><BR>"));
		client.print(F("<input type='hidden' name='manual' value='1'>")); ///manual
		client.print(F("Target:<input type='number' name='target' value='22.0' step='0.10' ><BR>")); // size='2' maxlength='2' 
		client.print(F("Sensor:<input type='radio' name='sensor' value='0' checked>Local<input type='radio' name='sensor' value='1'>Remote<BR>"));
		client.print(F("<input type='submit' value='start'></form>"));

	}
	else {
		client.print(F("</td><td>--"));
	}
	client.print(F("</td></tr>"));


	// temperature
	sprintf(buffer, "<tr><td>Local Temperature: </td><td>%d.%02d</td></tr>", (int)localTemperature, (int)(localTemperature * 100.0) % 100);
	client.print(buffer);
	sprintf(buffer, "<tr><td>Local Average Temperature: </td><td>%d.%02d</td></tr>", (int)localAvTemperature, (int)(localAvTemperature * 100.0) % 100);
	client.print(buffer);
	// remote temperature
	sprintf(buffer, "<tr><td>Remote Temperature: </td><td>%d.%02d</td></tr>", (int)remoteTemperature, (int)(remoteTemperature * 100.0) % 100);
	client.print(buffer);
	// program update
	sprintf(buffer, "<tr><td>Last program or temperature update: </td><td>%d</td></tr>", (now() - last_RemoteSensor));
	client.print(buffer);

	// sendbutton
	client.print(F("<tr><td>WOL</td><td><form action='/wol' method='POST'><input type='submit' value='send'></form></td><tr>"));
	client.print(F("</table><font color='#53669E' face='Verdana' size='2'><b>Impostazioni </b></font>\r\n<form action='/chstt' method='POST'><table width='80%' border='1'><colgroup bgcolor='#B6C4E9' width='20%' align='left'></colgroup><colgroup bgcolor='#FFFFFF' width='30%' align='left'></colgroup>"));

	// mac address
	client.print(F("<tr><td>Mac Address</td><td>"));
	for (byte i = 0; i < sizeof(mac); i++) {
		int n = mac[i];
		/*sprintf(buffer, "<input type='text' name='mac%d' value='%x' size='2' maxlength='2'>:", i, mac[i]);
		client.print(buffer);*/

		client.print(F("<input type='text' name='mac"));
		client.print(i);
		client.print(F("' value='"));
		client.print(n, HEX);
		client.print(F("' size='2' maxlength='2'>"));

		client.print(" ");
	}
	client.print(F(" </td></tr>"));


	// local ip
	client.print(F("<tr><td>IP</td><td>"));
	for (byte i = 0; i < sizeof(ip); i++) {

		int n = ip[i];
		client.print(F("<input type='text' name='ip"));
		client.print(i);
		client.print(F("' value='"));
		client.print(n, DEC);
		client.print(F("' size='3' maxlength='3'>"));
		client.print(F(" "));
	}
	client.print(F(" </td></tr>"));

	// id
	client.print(F("<tr><td>ID</td><td><input type='num' name='id' value='"));
	client.print(id);
	client.print(F("' size='2' maxlength='2'> </td></tr>"));

	// local port
	client.print(F("<tr><td>Local port</td><td><input type='num' name='localport"));
	client.print(F("' value='"));
	client.print(localPort, DEC /*HEX*/);
	client.print(F("' size='4' maxlength='4'> </td></tr>"));

	// server name
	client.print(F("<tr><td>Server name</td><td><input type='num' name='servername' value='"));
	client.print(servername);
	client.print(F("' size='"));
	client.print(servernamelen - 1);
	client.print(F("' maxlength='"));
	client.print(servernamelen - 1);
	client.print(F("'> </td></tr>"));

	// local port
	client.print(F("<tr><td>Server port</td><td><input type='num' name='serverport"));
	client.print(F("' value='"));
	client.print(serverPort, DEC /*HEX*/);
	client.print(F("' size='4' maxlength='4'> </td></tr>"));

	client.print(F("</table><input type='submit' value='save'/></form>"));

	sprintf(buffer, "%s", Versione);
	client.print(buffer);

	client.print(F("</body></html>"));

	delay(100);
	client.stop();

	//Serial.print(F("----->Memory free: change timer "));
	//Serial.println(memoryFree());
}

void showChangeSettings(char* GETparam) {

	//Serial.println(F("showChangeSettings "));
	getPostdata(databuff, maxposdataChangeSetting);
	char posdata[maxposdata];

	int val;
	char name[] = "mac0";
	for (byte i = 0; i < sizeof(mac); i++) {
		name[3] = i + 48;
		val = parsePostdata(databuff, name, posdata);
		unsigned long x;
		x = strtoul(posdata, 0, 16);
		mac[i] = x;
	}
	// localport
	val = parsePostdata(databuff, "localport", posdata);
	localPort = val;
	// local ip
	char nameip[] = "ip0";
	//Serial.println("localip");
	for (byte i = 0; i < sizeof(ip); i++) {
		nameip[2] = i + 48;
		val = parsePostdata(databuff, nameip, posdata);
		ip[i] = val;
		//Serial.print("ip: ");
		//Serial.print(val);
		//Serial.print(" - ");
		//Serial.println(posdata);
	}
	// server name
	val = parsePostdata(databuff, "servername", posdata);
	memccpy(servername, posdata, '\0', servernamelen);
	// server port
	val = parsePostdata(databuff, "serverport", posdata);
	Serial.print("server port ");
	Serial.println(val);
	serverPort = val;
	// id
	val = parsePostdata(databuff, "id", posdata);
	//Serial.print("server port ");
	//Serial.println(val);
	id = val;

	client.println(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\n\n<html><head><meta HTTP-EQUIV='REFRESH' content='0; url=/main'><title>Timer</title></head><body></body></html>"));

	writeEPROM();

	client.stop();

	Serial.print(F("<---Memory free: showmain "));
	Serial.println(memoryFree());
}


bool sendNotification(int type, int value)
{
	/*Serial.println(F("sendNotification "));

	if (!netwokStarted) return false;

	char   buffer[100];
	sprintf(buffer, "{\"id\":%d,\"type\":%d,\"value\":%d}", id, type, value);

	return post(servername, serverPort, "/webduino/notification", buffer);*/

	return true;
}

boolean sendStatus()
{
	Serial.println(F("sendStatus "));

	if (!netwokStarted) return false;

	time_t remaining = programDuration - (now() - programStartTime);

	char   buffer[300];

	Serial.println(statusStr[currentStatus]);
	Serial.println(boolStr[releStatus]);
	sprintf(buffer, "{\"command\":\"status\",\"id\":%d,\"avtemperature\":%d.%02d,\"status\":\"%s\",\"relestatus\":\"%s\" ,\"remaining\":\"%d\"   }",
		(int)id,
		(int)localAvTemperature, (int)(localAvTemperature * 100.0) % 100,
		statusStr[currentStatus],
		(releStatus) ? boolStr[1] : boolStr[0]),
		remaining;

	Serial.println(buffer);


	/*client.print(F(",\"localsensor\":"));
	if (temperatureSensor == sensor_local)
	client.print(F("true"));
	else
	client.print(F("false"));

	client.print(F(",\"target\":"));
	client.print(targetTemperature);

	client.print(F(",\"program\":"));
	client.print(activeProgram, DEC);

	client.print(F(",\"timerange\":"));
	client.print(activeTimerange, DEC);*/


	Serial.print("servername=");
	Serial.println(servername);
	return post(servername, serverPort, "/webduino/actuator", buffer);

	//return true;
}





bool post(char* host, int port, char* path, char* param)
{
	wdt_enable(WDTO_8S);

	W5100.setRetransmissionCount(2);

	String data;
	data += "";
	data += param;

	Serial.println(F("post notification"));
	Serial.println(host);
	Serial.println(port);
	Serial.println(path);
	Serial.println(param);

	//client2 = server.available();

	if (client2.connect(host, port)) {
		Serial.println("connected");
		client2.print("POST ");
		client2.print(path);
		client2.println(" HTTP/1.1\nHost: ");
		client2.println(host);
		client2.println("Content-Type: application/x-www-form-urlencoded");
		client2.println("Connection: close");
		client2.print("Content-Length: ");
		client2.println(data.length());
		client2.println();
		client2.print(data);
		client2.println();

		int counter = 0;
		while (client2.connected()) {

			if (client2.find("result")){
				if (client2.find("=")){
					int result = client2.parseInt();
					Serial.print(F("result= "));
					Serial.println(result);
					break;
				}
			}
			else {
				Serial.println(F("not found"));
				if (counter++ > 3)
					break;
			}

			delay(500); // check again in 1 seconds 
		}

	}
	else {
		Serial.println(F("-NON CONNESSO-"));
		return false;
	}
	client2.stop();
	return true;
}

/*void unknownPage(const char *page)
{
//sendHeader("unknown");
client.println(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<html><head><title>Webduino</title><body>Unknown</body></html>"));
}*/

int len(const char* data) {

	int l = 0;
	while (data[l] != '\0')
		l++;
	return l;
}

int parsePostdata(const char* data, const char* param, char* value) {


	int pos = findIndex(data, param);

	if (pos != -1) {

		int i = 0;
		char c;
		while (i < len(data) && i < maxvaluesize) {
			if (data[i + pos + len(param) + 1] == '&' || data[i + pos + len(param) + 1] == '\0')
				break;
			value[i] = data[i + pos + len(param) + 1];
			i++;
		}
		//if (i < maxvaluesize) value[i] = '\0';
		/*if (i < maxvaluesize)*/ value[i] = '\0';
		int val = 0;

		for (int k = 0; k < i; k++) {

			val = (val * 10) + value[k] - 48;
		}
		return val;
	}
	return -1;
}

void getPostdata(char *data, int maxposdata) {

	Serial.print(F("getPostdata"));
	//Serial.print(F("+++->Memory free: "));
	//Serial.println(memoryFree());

	int datalen = 0;

	if (client.findUntil("Content-Length:", "\n\r"))
	{
		datalen = client.parseInt();
	}


	delay(400);
	if (client.findUntil("\n\r", "\n\r"))
	{
		;
	}
	delay(400);
	client.read();

	if (datalen >= maxposdata) {
		Serial.println(F("maxposdat to long"));
	}


	int i = 0;
	while (i < datalen && i < maxposdata) {
		data[i] = client.read();
		Serial.print(data[i]); // ailitare questa riga per vedere il contenuto della post
		delay(2);
		i++;
	}

	Serial.println("");
	Serial.print("datalen ");
	Serial.print(datalen);
	if (i < maxposdata)
		data[i] = '\0';
}
int findIndex(const char* data, const char* target) {

	boolean found = false;
	int i = 0;
	while (data[i] != '\0') {
		i++;
	}
	i = 0;
	int k = 0;
	while (data[i] != '\0') {

		if (data[i] == target[0]) {
			found = true;
			k = 0;
			while (target[k] != '\0') {

				if (data[i + k] == '\0')
					return -1;
				if (data[i + k] != target[k]) {
					found = false;
					break;
				}
				k++;
			}
			if (found == true)
				return i;
		}
		i++;
	}
	return -1;
}

void getStatus(char* GETparam)
{
	Serial.print(F("getStatus start"));
	Serial.print(F("+++>Memory free: "));
	Serial.println(memoryFree());

	client.print(F("HTTP/1.0 200 OK\r\nContent-Type: application/json; charset=utf-8\r\nPragma: no-cache\r\n\r\n"));

	client.print(F("{\"id\":"));
	client.print(id, DEC);

	client.print(F(",\"temperature\":"));
	float temp = localTemperature;
	client.print(temp, DEC);

	client.print(F(",\"avtemperature\":"));
	temp = localAvTemperature;
	client.print(temp, DEC);

	client.print(F(",\"remotetemperature\":"));
	temp = remoteTemperature;
	client.print(temp, DEC);

	char buf[50];
	sprintf(buf, ",\"status\":\"%s\"", statusStr[currentStatus]);
	client.print(buf);

	client.print(F(",\"relestatus\":"));
	if (releStatus)
		client.print(F("true"));
	else
		client.print(F("false"));

	if (currentStatus == STATUS_PROGRAMACTIVE || currentStatus == STATUS_MANUAL) {

		client.print(F(",\"duration\":"));
		client.print(programDuration, DEC);

		int remainingTime = programDuration - (now() - programStartTime);
		client.print(F(",\"remaining\":"));
		client.print(remainingTime, DEC);

		client.print(F(",\"localsensor\":"));
		if (temperatureSensor == sensor_local)
			client.print(F("true"));
		else
			client.print(F("false"));

		client.print(F(",\"target\":"));
		client.print(targetTemperature);

		client.print(F(",\"program\":"));
		client.print(activeProgram, DEC);

		client.print(F(",\"timerange\":"));
		client.print(activeTimerange, DEC);

	}
	client.print(F("}"));
	delay(300);
	client.stop();

	Serial.print(F("getStatus end"));
	Serial.print(F("<---Memory free: setup "));
	Serial.println(memoryFree());

}

void setRemoteTemperature(char* GETparam) {

	Serial.println(F("setRemoteTemperature "));

	getPostdata(databuff, maxposdataChangeSetting);
	char posdata[maxposdata];

	int val;
	// status
	parsePostdata(databuff, "temperature", posdata);
	String str = "";
	str += posdata;
	remoteTemperature = str.toFloat();

	last_RemoteSensor = now();

	/*Serial.print(F("remoteTemperature="));
	Serial.println(remoteTemperature);

	client.println(F("HTTP/1.1 200 OK\r\nContent-Type: text/html"));

	client.println(F("\n\n<html><head><meta HTTP-EQUIV='REFRESH' content='2; url=/main?msg=2'><title>WOL</title></head><body>"));

	char   buffer[50];
	sprintf(buffer, "remote=%s", posdata);
	client.print(buffer);

	client.println(F("</body></html>"));*/
	getStatus(NULL);

}

void showPower(char* GETparam) {

	Serial.println(F("showPower "));

	getPostdata(databuff, maxposdataChangeSetting);
	char posdata[maxposdata];

	int val;
	// status
	val = parsePostdata(databuff, "status", posdata);
	Serial.print(F("status="));
	Serial.println(val, DEC);


	if (val == 1 && currentStatus == STATUS_DISABLED) {
		currentStatus = STATUS_IDLE;
		enableRele(false);

	}
	else if (val == 0) {
		currentStatus = STATUS_DISABLED;
		enableRele(false);
	}
	client.println(F("HTTP/1.1 200 OK\r\nContent-Type: text/html"));

	client.println(F("\n\n<html><head><meta HTTP-EQUIV='REFRESH' content='2; url=/main?msg=2'><title>WOL</title></head><body>"));

	char   buffer[50];
	sprintf(buffer, "relestatus=%d,status=%s", releStatus, statusStr[currentStatus]);
	client.print(buffer);

	client.println(F("</body></html>"));
}
