#include <Arduino.h>

// Minimal Prologix + TSP3222 simulator for quick host-side tests.
// Serial settings match common Prologix defaults used by Python drivers.

static const long SERIAL_BAUD = 9600;
static const int TSP3222_GPIB_ADDR = 15;
static const int ZUP_DEVICE_ADDR = 1;

static String rxLine;
static String pendingReply;

// Prologix-like state
static int gpibAddress = 0;
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

// ZUP-like state (single channel, direct RS232)
static int zupSelectedAddr = ZUP_DEVICE_ADDR;
static float zupVSet = 5.000f;
static float zupISet = 1.000f;
static bool zupOutputOn = false;
static bool zupAutoRestart = false;
static float zupOVP = 6.500f;
static float zupUVP = 0.000f;

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
		up == String("*IDN?") ||
		up == String("*RST") ||
		up == String("V1R?") || up == String("V2R?") || up == String("I1R?") || up == String("I2R?") ||
		up == String("V1?") || up == String("V2?") || up == String("I1?") || up == String("I2?") ||
		up == String("OP1?") || up == String("OP2?") ||
		up.startsWith("V1 ") || up.startsWith("V2 ") ||
		up.startsWith("I1 ") || up.startsWith("I2 ") ||
		up.startsWith("OP1 ") || up.startsWith("OP2 ") || up.startsWith("OPALL ")
	);
}

static bool isPL303QMDCommand(const String &up) {
	return (
		up == String("*IDN?") ||
		up == String("*RST") ||
		up == String("V1O?") || up == String("V2O?") || up == String("I1O?") || up == String("I2O?") ||
		up == String("V1?") || up == String("V2?") || up == String("I1?") || up == String("I2?") ||
		up == String("OP1?") || up == String("OP2?") ||
		up.startsWith("V1 ") || up.startsWith("V2 ") ||
		up.startsWith("I1 ") || up.startsWith("I2 ") ||
		up.startsWith("OP1 ") || up.startsWith("OP2 ") || up.startsWith("OPALL ")
	);
}

static bool isEL302Command(const String &up) {
	return (
		up == String("*IDN?") ||
		up == String("*RST") ||
		up == "VO?" || up == "IO?" || up == "V?" || up == "I?" || up == "OUT?" ||
		up == "ON" || up == "OFF" ||
		up.startsWith("V ") || up.startsWith("I ")
	);
}

static bool isZUPCommand(const String &up) {
	return (
		up.indexOf(":ADR") >= 0 ||
		up.startsWith(":VOL") || up.startsWith(":CUR") ||
		up.startsWith(":OUT") || up.startsWith(":MDL?") || up.startsWith(":REV?") ||
		up.startsWith(":AST") || up.startsWith(":OVP") || up.startsWith(":UVP") ||
		up.startsWith(":STA?") || up.startsWith(":ALM?") || up.startsWith(":STP?") ||
		up.startsWith(":STT?")
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

	if (pendingReply.length() > 0) {
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

	if (pendingReply.length() > 0) {
		Serial.println(pendingReply);
		pendingReply = "";
	}
}

static void handleZUPCommand(const String &cmdRaw) {
	String cmd = cmdRaw;
	cmd.trim();
	if (cmd.length() == 0) return;

	int targetAddr = zupSelectedAddr;
	String payload = "";

	int from = 0;
	while (from <= cmd.length()) {
		int sep = cmd.indexOf(';', from);
		if (sep < 0) sep = cmd.length();

		String token = cmd.substring(from, sep);
		token.trim();
		if (token.length() > 0) {
			if (token.startsWith(":")) token = token.substring(1);

			String tokenUp = token;
			tokenUp.toUpperCase();

			if (tokenUp.startsWith("ADR")) {
				int addr = token.substring(3).toInt();
				if (addr > 0) {
					targetAddr = addr;
					zupSelectedAddr = addr;
				}
			} else if (payload.length() == 0) {
				payload = token;
			}
		}

		if (sep >= cmd.length()) break;
		from = sep + 1;
	}

	if (payload.length() == 0) {
		pendingReply = "";
		return;
	}

	// Simulate one ZUP unit at address 1. Other addresses are silent.
	if (targetAddr != ZUP_DEVICE_ADDR) {
		pendingReply = "";
		return;
	}

	String up = payload;
	up.toUpperCase();

	if (up == "MDL?") {
		pendingReply = "Nemic-Lambda<6V-33A>";
	} else if (up == "REV?") {
		pendingReply = "Ver 1.00";
	} else if (up == "VOL?") {
		pendingReply = f4(zupOutputOn ? (zupVSet + pseudoJitter()) : 0.0f);
	} else if (up == "CUR?") {
		pendingReply = f4(zupOutputOn ? (min(zupISet, 0.1500f) + pseudoJitter()) : 0.0f);
	} else if (up == "OUT?") {
		pendingReply = zupOutputOn ? "OUT1" : "OUT0";
	} else if (up == "AST?") {
		pendingReply = zupAutoRestart ? "AST1" : "AST0";
	} else if (up == "OVP?") {
		pendingReply = f4(zupOVP);
	} else if (up == "UVP?") {
		pendingReply = f4(zupUVP);
	} else if (up == "STA?") {
		pendingReply = "STA0000";
	} else if (up == "ALM?") {
		pendingReply = "ALM0000";
	} else if (up == "STP?") {
		pendingReply = "STP0000";
	} else if (up == "STT?") {
		pendingReply = String("MV") +
			f4(zupOutputOn ? (zupVSet + pseudoJitter()) : 0.0f) +
			",MC" +
			f4(zupOutputOn ? (min(zupISet, 0.1500f) + pseudoJitter()) : 0.0f) +
			",SR" + (zupOutputOn ? "1" : "0");
	} else {
		if (up.startsWith("VOL")) {
			zupVSet = payload.substring(3).toFloat();
		} else if (up.startsWith("CUR")) {
			zupISet = payload.substring(3).toFloat();
		} else if (up == "OUT1") {
			zupOutputOn = true;
		} else if (up == "OUT0") {
			zupOutputOn = false;
		} else if (up.startsWith("AST")) {
			zupAutoRestart = (payload.substring(3).toInt() != 0);
		} else if (up.startsWith("OVP")) {
			zupOVP = payload.substring(3).toFloat();
		} else if (up.startsWith("UVP")) {
			zupUVP = payload.substring(3).toFloat();
		}
		pendingReply = "";
	}

	// ZUP is direct serial in this project: answer queries immediately.
	if (pendingReply.length() > 0) {
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
		// Serial.println(up);

		// 1) ZUP: direct RS232 command syntax (e.g. :ADR01;:VOL?;).
		if (isZUPCommand(up)) {
			// Serial.println("ZUP command detected");
			handleZUPCommand(line);
			return;
		}

		// 2) TSP3222: only when selected GPIB address matches.
		if (gpibAddress == TSP3222_GPIB_ADDR && isTSP3222Command(up)) {
			handleTSP3222Command(line);
			return;
		}

		// 3) USB instruments by protocol, independent of GPIB address.
		// Shared commands (e.g. *IDN?) are first-match wins (PL before EL).
		if (isPL303QMDCommand(up)) {
			// Serial.println("PL303QMD command detected");
			handlePL303QMDCommand(line);
			return;
		}
		if (isEL302Command(up)) {
			// Serial.println("EL302 command detected");
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
