#include <Arduino.h>

// Minimal Prologix + TSP3222 simulator for quick host-side tests.
// Serial settings match common Prologix defaults used by Python drivers.

static const long SERIAL_BAUD = 9600;
static const int TSP3222_GPIB_ADDR = 15;

static String rxLine;
static String pendingReply;

// Prologix-like state
static int gpibAddress = TSP3222_GPIB_ADDR;
static bool autoRead = false;

// TSP3222-like state (2 channels)
static float tspVSet[2] = {5.000f, 12.000f};
static float tspISet[2] = {1.000f, 0.500f};
static bool tspOutputOn[2] = {false, false};

// PL303QMD-like state (2 channels)
static float plVSet[2] = {3.300f, 15.000f};
static float plISet[2] = {0.300f, 0.200f};
static bool plOutputOn[2] = {false, false};

// EL302-like state (single channel)
static float elVSet = 5.000f;
static float elISet = 1.000f;
static bool elOutputOn = false;

static String f4(float v) {
	return String(v, 4);
}

static float pseudoJitter() {
	long t = millis() / 100;
	int step = (int)(t % 5) - 2;   // -2..2
	return 0.001f * step;          // +/-2 mV/A-ish
}

static bool isTSP3222Command(const String &up) {
	return (
		up == "*IDN?" ||
		up == "*RST" ||
		up == "V1R?" || up == "V2R?" || up == "I1R?" || up == "I2R?" ||
		up == "V1?" || up == "V2?" || up == "I1?" || up == "I2?" ||
		up == "OP1?" || up == "OP2?" ||
		up.startsWith("V1 ") || up.startsWith("V2 ") ||
		up.startsWith("I1 ") || up.startsWith("I2 ") ||
		up.startsWith("OP1 ") || up.startsWith("OP2 ") || up.startsWith("OPALL ")
	);
}

static bool isPL303QMDCommand(const String &up) {
	return (
		up == "*IDN?" ||
		up == "*RST" ||
		up == "V1O?" || up == "V2O?" || up == "I1O?" || up == "I2O?" ||
		up == "V1?" || up == "V2?" || up == "I1?" || up == "I2?" ||
		up == "OP1?" || up == "OP2?" ||
		up.startsWith("V1 ") || up.startsWith("V2 ") ||
		up.startsWith("I1 ") || up.startsWith("I2 ") ||
		up.startsWith("OP1 ") || up.startsWith("OP2 ") || up.startsWith("OPALL ")
	);
}

static bool isEL302Command(const String &up) {
	return (
		up == "*IDN?" ||
		up == "*RST" ||
		up == "VO?" || up == "IO?" || up == "V?" || up == "I?" || up == "OUT?" ||
		up == "ON" || up == "OFF" ||
		up.startsWith("V ") || up.startsWith("I ")
	);
}

static void handleTSP3222Command(const String &cmdRaw) {
	String cmd = cmdRaw;
	cmd.trim();
	if (cmd.length() == 0) return;

	String up = cmd;
	up.toUpperCase();

	// TSP3222 is simulated on GPIB address 15 only.
	if (gpibAddress != TSP3222_GPIB_ADDR) {
		pendingReply = "";
		return;
	}

	// Queries
	if (up == "*IDN?") {
		pendingReply = "TTi,TSP3222,SIM0001,1.00";
	} else if (up == "V1R?") {
		pendingReply = f4(tspOutputOn[0] ? (tspVSet[0] + pseudoJitter()) : 0.0f);
	} else if (up == "V2R?") {
		pendingReply = f4(tspOutputOn[1] ? (tspVSet[1] + pseudoJitter()) : 0.0f);
	} else if (up == "I1R?") {
		pendingReply = f4(tspOutputOn[0] ? min(tspISet[0], 0.1200f) + pseudoJitter() : 0.0f);
	} else if (up == "I2R?") {
		pendingReply = f4(tspOutputOn[1] ? min(tspISet[1], 0.0800f) + pseudoJitter() : 0.0f);
	} else if (up == "V1?") {
		pendingReply = f4(tspVSet[0]);
	} else if (up == "V2?") {
		pendingReply = f4(tspVSet[1]);
	} else if (up == "I1?") {
		pendingReply = f4(tspISet[0]);
	} else if (up == "I2?") {
		pendingReply = f4(tspISet[1]);
	} else if (up == "OP1?") {
		pendingReply = tspOutputOn[0] ? "1" : "0";
	} else if (up == "OP2?") {
		pendingReply = tspOutputOn[1] ? "1" : "0";
	} else {
		// Set commands / actions
		if (up.startsWith("V1 ")) {
			tspVSet[0] = cmd.substring(3).toFloat();
		} else if (up.startsWith("V2 ")) {
			tspVSet[1] = cmd.substring(3).toFloat();
		} else if (up.startsWith("I1 ")) {
			tspISet[0] = cmd.substring(3).toFloat();
		} else if (up.startsWith("I2 ")) {
			tspISet[1] = cmd.substring(3).toFloat();
		} else if (up.startsWith("OP1 ")) {
			tspOutputOn[0] = (cmd.substring(4).toInt() != 0);
		} else if (up.startsWith("OP2 ")) {
			tspOutputOn[1] = (cmd.substring(4).toInt() != 0);
		} else if (up.startsWith("OPALL ")) {
			bool on = (cmd.substring(6).toInt() != 0);
			tspOutputOn[0] = on;
			tspOutputOn[1] = on;
		} else if (up == "*RST") {
			tspVSet[0] = 5.000f;
			tspVSet[1] = 12.000f;
			tspISet[0] = 1.000f;
			tspISet[1] = 0.500f;
			tspOutputOn[0] = false;
			tspOutputOn[1] = false;
		}
		pendingReply = "";
	}

	if (autoRead && pendingReply.length() > 0) {
		Serial.println(pendingReply);
		pendingReply = "";
	}
}

static void handlePL303QMDCommand(const String &cmdRaw) {
	String cmd = cmdRaw;
	cmd.trim();
	if (cmd.length() == 0) return;

	String up = cmd;
	up.toUpperCase();

	if (up == "*IDN?") {
		pendingReply = "Aim-TTi,PL303QMD,SIM0002,1.00";
	} else if (up == "V1O?") {
		pendingReply = f4(plOutputOn[0] ? (plVSet[0] + pseudoJitter()) : 0.0f);
	} else if (up == "V2O?") {
		pendingReply = f4(plOutputOn[1] ? (plVSet[1] + pseudoJitter()) : 0.0f);
	} else if (up == "I1O?") {
		pendingReply = f4(plOutputOn[0] ? min(plISet[0], 0.0900f) + pseudoJitter() : 0.0f);
	} else if (up == "I2O?") {
		pendingReply = f4(plOutputOn[1] ? min(plISet[1], 0.0600f) + pseudoJitter() : 0.0f);
	} else if (up == "V1?") {
		pendingReply = f4(plVSet[0]);
	} else if (up == "V2?") {
		pendingReply = f4(plVSet[1]);
	} else if (up == "I1?") {
		pendingReply = f4(plISet[0]);
	} else if (up == "I2?") {
		pendingReply = f4(plISet[1]);
	} else if (up == "OP1?") {
		pendingReply = plOutputOn[0] ? "1" : "0";
	} else if (up == "OP2?") {
		pendingReply = plOutputOn[1] ? "1" : "0";
	} else {
		if (up.startsWith("V1 ")) {
			plVSet[0] = cmd.substring(3).toFloat();
		} else if (up.startsWith("V2 ")) {
			plVSet[1] = cmd.substring(3).toFloat();
		} else if (up.startsWith("I1 ")) {
			plISet[0] = cmd.substring(3).toFloat();
		} else if (up.startsWith("I2 ")) {
			plISet[1] = cmd.substring(3).toFloat();
		} else if (up.startsWith("OP1 ")) {
			plOutputOn[0] = (cmd.substring(4).toInt() != 0);
		} else if (up.startsWith("OP2 ")) {
			plOutputOn[1] = (cmd.substring(4).toInt() != 0);
		} else if (up.startsWith("OPALL ")) {
			bool on = (cmd.substring(6).toInt() != 0);
			plOutputOn[0] = on;
			plOutputOn[1] = on;
		} else if (up == "*RST") {
			plVSet[0] = 3.300f;
			plVSet[1] = 15.000f;
			plISet[0] = 0.300f;
			plISet[1] = 0.200f;
			plOutputOn[0] = false;
			plOutputOn[1] = false;
		}
		pendingReply = "";
	}

	if (autoRead && pendingReply.length() > 0) {
		Serial.println(pendingReply);
		pendingReply = "";
	}
}

static void handleEL302Command(const String &cmdRaw) {
	String cmd = cmdRaw;
	cmd.trim();
	if (cmd.length() == 0) return;

	String up = cmd;
	up.toUpperCase();

	if (up == "*IDN?") {
		pendingReply = "Aim-TTi,EL302,SIM0003,1.00";
	} else if (up == "VO?") {
		pendingReply = f4(elOutputOn ? (elVSet + pseudoJitter()) : 0.0f);
	} else if (up == "IO?") {
		pendingReply = f4(elOutputOn ? min(elISet, 0.1100f) + pseudoJitter() : 0.0f);
	} else if (up == "V?") {
		pendingReply = f4(elVSet);
	} else if (up == "I?") {
		pendingReply = f4(elISet);
	} else if (up == "OUT?") {
		pendingReply = elOutputOn ? "ON" : "OFF";
	} else {
		if (up.startsWith("V ")) {
			elVSet = cmd.substring(2).toFloat();
		} else if (up.startsWith("I ")) {
			elISet = cmd.substring(2).toFloat();
		} else if (up == "ON") {
			elOutputOn = true;
		} else if (up == "OFF") {
			elOutputOn = false;
		} else if (up == "*RST") {
			elVSet = 5.000f;
			elISet = 1.000f;
			elOutputOn = false;
		}
		pendingReply = "";
	}

	if (autoRead && pendingReply.length() > 0) {
		Serial.println(pendingReply);
		pendingReply = "";
	}
}

static void handlePrologixCommand(const String &raw) {
	String cmd = raw;
	cmd.trim();
	if (cmd.length() == 0) return;

	String up = cmd;
	up.toUpperCase();

	if (up == "++VER") {
		Serial.println("Prologix GPIB-USB Controller version 6.0");
		return;
	}

	if (up.startsWith("++MODE ")) {
		// Accepted, ignored in this simulator.
		return;
	}

	if (up.startsWith("++AUTO ")) {
		autoRead = (cmd.substring(7).toInt() != 0);
		return;
	}

	if (up.startsWith("++ADDR ")) {
		gpibAddress = cmd.substring(7).toInt();
		return;
	}

	if (up == "++READ EOI") {
		Serial.println(pendingReply);
		pendingReply = "";
		return;
	}
}

static void processLine(const String &line) {
	if (line.startsWith("++")) {
		handlePrologixCommand(line);
	} else {
		String up = line;
		up.trim();
		up.toUpperCase();

		// 1) TSP3222: only when selected GPIB address matches.
		if (gpibAddress == TSP3222_GPIB_ADDR && isTSP3222Command(up)) {
			handleTSP3222Command(line);
			return;
		}

		// 2) USB instruments by protocol, independent of GPIB address.
		// Shared commands (e.g. *IDN?) are first-match wins (PL before EL).
		if (isPL303QMDCommand(up)) {
			handlePL303QMDCommand(line);
			return;
		}
		if (isEL302Command(up)) {
			handleEL302Command(line);
			return;
		}

		pendingReply = "";
	}
}

void setup() {
	Serial.begin(SERIAL_BAUD);
	rxLine.reserve(96);
	pendingReply.reserve(64);
}

void loop() {
	while (Serial.available() > 0) {
		char c = (char)Serial.read();
		if (c == '\n' || c == '\r') {
			if (rxLine.length() > 0) {
				processLine(rxLine);
				rxLine = "";
			}
		} else {
			rxLine += c;
			if (rxLine.length() > 120) {
				rxLine = "";
			}
		}
	}
}
