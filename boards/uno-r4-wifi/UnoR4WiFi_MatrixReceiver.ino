// Uno R4 WiFi matrix receiver for Mega exit gate status.
// Data arrives from the Mega on hardware RX0 via Serial1.
// Expects newline-delimited messages like:
// O5..O1 = opening countdown, O0 = open
// C5..C1 = closing countdown, C0 = closed
// W? = entry opening/open hold (wait)
// W5..W1 = entry closing/countdown (wait)

#include <Arduino_LED_Matrix.h>

ArduinoLEDMatrix matrix;

char lineBuf[16];
uint8_t linePos = 0;
unsigned long lastRxMs = 0;
unsigned long happyUntilMs = 0;
char currentLeft = 'C';
char currentRight = '0';

void drawFace(bool happy) {
	uint8_t frame[8][12] = {
		{0,0,0,0,0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0,0,0,0,0},
		{0,0,0,0,0,1,0,1,0,0,0,0},
		{0,0,0,0,0,1,0,1,0,0,0,0},
		{0,0,0,0,0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0,0,0,0,0}
	};

	// Distinct mouth shape for connection state.
	if (happy) {
		frame[6][3] = 1;
		frame[7][4] = 1;
		frame[7][5] = 1;
		frame[7][6] = 1;
		frame[7][7] = 1;
		frame[6][8] = 1;
	} else {
		frame[6][4] = 1;
		frame[5][5] = 1;
		frame[5][6] = 1;
		frame[5][7] = 1;
		frame[6][8] = 1;
	}

	matrix.renderBitmap(frame, 8, 12);
}

void pushCharPattern(char ch, uint8_t out[8][12]) {
	const uint8_t blank[8]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	const uint8_t Opat[8]   = {0x00, 0x3C, 0x42, 0x42, 0x42, 0x42, 0x3C, 0x00};
	const uint8_t Cpat[8]   = {0x00, 0x3C, 0x40, 0x40, 0x40, 0x40, 0x3C, 0x00};
	const uint8_t Wpat[8]   = {0x00, 0x42, 0x42, 0x42, 0x5A, 0x5A, 0x24, 0x00};
	const uint8_t qpat[8]   = {0x00, 0x3C, 0x42, 0x04, 0x08, 0x00, 0x08, 0x00};
	const uint8_t n0[8]     = {0x00, 0x3C, 0x46, 0x4A, 0x52, 0x62, 0x3C, 0x00};
	const uint8_t n1[8]     = {0x00, 0x08, 0x18, 0x08, 0x08, 0x08, 0x1C, 0x00};
	const uint8_t n2[8]     = {0x00, 0x3C, 0x42, 0x04, 0x18, 0x20, 0x7E, 0x00};
	const uint8_t n3[8]     = {0x00, 0x3C, 0x42, 0x0C, 0x02, 0x42, 0x3C, 0x00};
	const uint8_t n4[8]     = {0x00, 0x04, 0x0C, 0x14, 0x24, 0x7E, 0x04, 0x00};
	const uint8_t n5[8]     = {0x00, 0x7E, 0x40, 0x7C, 0x02, 0x42, 0x3C, 0x00};
	const uint8_t n6[8]     = {0x00, 0x1C, 0x20, 0x7C, 0x42, 0x42, 0x3C, 0x00};
	const uint8_t n7[8]     = {0x00, 0x7E, 0x42, 0x04, 0x08, 0x10, 0x10, 0x00};
	const uint8_t n8[8]     = {0x00, 0x3C, 0x42, 0x3C, 0x42, 0x42, 0x3C, 0x00};
	const uint8_t n9[8]     = {0x00, 0x3C, 0x42, 0x42, 0x3E, 0x02, 0x1C, 0x00};

	const uint8_t* src = blank;
	if (ch == 'O') src = Opat;
	else if (ch == 'C') src = Cpat;
	else if (ch == 'W') src = Wpat;
	else if (ch == '?') src = qpat;
	else if (ch == '0') src = n0;
	else if (ch == '1') src = n1;
	else if (ch == '2') src = n2;
	else if (ch == '3') src = n3;
	else if (ch == '4') src = n4;
	else if (ch == '5') src = n5;
	else if (ch == '6') src = n6;
	else if (ch == '7') src = n7;
	else if (ch == '8') src = n8;
	else if (ch == '9') src = n9;

	for (uint8_t r = 0; r < 8; r++) {
		for (uint8_t c = 0; c < 12; c++) {
			out[r][c] = ((src[r] >> (7 - c)) & 0x01) ? 1 : 0;
		}
	}
}

void drawPair(char left, char right) {
	uint8_t leftGlyph[8][12];
	uint8_t rightGlyph[8][12];
	uint8_t frame[8][12];

	pushCharPattern(left, leftGlyph);
	pushCharPattern(right, rightGlyph);

	for (uint8_t r = 0; r < 8; r++) {
		for (uint8_t c = 0; c < 12; c++) {
			frame[r][c] = 0;
		}
	}

	// Left glyph columns 0..5
	for (uint8_t r = 0; r < 8; r++) {
		for (uint8_t c = 1; c <= 6; c++) {
			frame[r][c - 1] = leftGlyph[r][c];
		}
	}

	// Right glyph columns 0..5 shifted to 6..11
	for (uint8_t r = 0; r < 8; r++) {
		for (uint8_t c = 1; c <= 6; c++) {
			frame[r][c + 5] = rightGlyph[r][c];
		}
	}

	matrix.renderBitmap(frame, 8, 12);
}

void showNoLink() {
	drawFace(false);
}

void applyMessage(const char* msg) {
	bool parsed = false;
	char nextLeft = currentLeft;
	char nextRight = currentRight;

	// Accept formats: O5, C3, O0, C0, W?, W1..W5
	if (msg[0] == 'O' && msg[1] >= '0' && msg[1] <= '9') {
		nextLeft = 'O';
		nextRight = msg[1];
		parsed = true;
	}
	else if (msg[0] == 'C' && msg[1] >= '0' && msg[1] <= '9') {
		nextLeft = 'C';
		nextRight = msg[1];
		parsed = true;
	}
	else if (msg[0] == 'W' && ((msg[1] >= '0' && msg[1] <= '9') || msg[1] == '?')) {
		nextLeft = 'W';
		nextRight = msg[1];
		parsed = true;
	}
	if (!parsed) {
		return;
	}

	currentLeft = nextLeft;
	currentRight = nextRight;
	lastRxMs = millis();
	drawPair(currentLeft, currentRight);
}

void setup() {
	Serial.begin(9600);
	Serial1.begin(9600);
	matrix.begin();
	showNoLink();
	Serial.println("Uno R4 exit matrix ready");
}

void loop() {
	while (Serial1.available() > 0) {
		char ch = (char)Serial1.read();
		if (ch == '\r') {
			continue;
		}
		if (ch == '\n') {
			if (linePos > 0) {
				lineBuf[linePos] = '\0';
				Serial.print("RX: ");
				Serial.println(lineBuf);
				applyMessage(lineBuf);
				linePos = 0;
			}
			continue;
		}

		if (linePos < (sizeof(lineBuf) - 1)) {
			lineBuf[linePos++] = ch;
		} else {
			linePos = 0;
		}
	}

	// Show sad face only on real communication loss.
	if (lastRxMs != 0 && (millis() - lastRxMs) > 10000) {
		showNoLink();
		happyUntilMs = 0;
		lastRxMs = 0;
	} else if (lastRxMs != 0) {
		drawPair(currentLeft, currentRight);
	} else {
		drawPair('C', '0');
	}
}
