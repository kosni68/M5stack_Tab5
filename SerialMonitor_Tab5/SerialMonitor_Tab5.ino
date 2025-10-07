// M5Core Tab5 - Serial Monitor with On-Screen Keyboard + Baud Control + Auto-Rotate (4 orientations + clear)
// RX=GPIO38, TX=GPIO37 for external UART
// Requires: M5Unified
//
// Ajouts :
// - Effacement écran à chaque rotation.
// - 4 orientations : PortraitUp, PortraitDown (180°), LandscapeRight, LandscapeLeft (180°).
// - Choix via boussole (N/S/E/O) + fallback accéléromètre (signe sur l’axe dominant).
// - Hystérésis + délai anti-flip.
// - Relayout + redraw complets après rotation.

#include <M5Unified.h>
#include <cmath>

// ---------- Serial config ----------
HardwareSerial SerialExt(1);  // use UART #1
static constexpr int RX_PIN    = 38;       // external RX (GPIO38)
static constexpr int TX_PIN    = 37;       // external TX (GPIO37)
static constexpr uint32_t BAUD_USB = 115200;

uint32_t baudRates[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
int baudIdx = 4; // start at 115200
bool sendCRLF = true;

// ---------- Monitor buffer ----------
static constexpr int MAX_BUF_LINES = 300;
String lines[MAX_BUF_LINES];
int lineCount = 0;
String currentIncomingLine;

// ---------- Input buffer ----------
String inputBuffer;

// ---------- UI geometry ----------
struct Rect {
  int x, y, w, h;
  bool contains(int px, int py) const {
    return (px >= x && px < x + w && py >= y && py < y + h);
  }
};

// Will be set in layout()
Rect topBar, btnClear, btnBaud, btnCRLF;
Rect monitorRect;
Rect inputRect, btnSend;
Rect keyboardRect;

int W = 0, H = 0;           // screen size
int margin = 8;

// ---------- Keyboard ----------
enum KeyType { KT_CHAR, KT_SHIFT, KT_MODE, KT_BKSP, KT_SPACE, KT_NONE };
bool upperCase = false;
bool numMode   = false;

String row1_alpha = "qwertyuiop";
String row2_alpha = "asdfghjkl";
String row3_alpha = "zxcvbnm";
String row1_num   = "1234567890";
String row2_num   = "-/:;()$&@";
String row3_num   = ".,?!'\"";

// ---------- Orientation / Compass ----------
enum class UiOrient { PortraitUp, PortraitDown, LandscapeRight, LandscapeLeft };

UiOrient currentOrient = UiOrient::LandscapeRight; // défaut
UiOrient targetOrient  = UiOrient::LandscapeRight;

static constexpr uint32_t ORIENT_CHECK_MS = 200;      // fréquence de vérif
static constexpr uint32_t ORIENT_APPLY_DELAY_MS = 600; // délai anti-flip
uint32_t lastOrientPoll = 0;
uint32_t orientStableSince = 0;

// Hystérésis des secteurs (en degrés) pour éviter les bascules sur la frontière
static constexpr float SECTOR_HALF_CORE = 45.f - 12.f; // cœur du secteur (±45° rétréci de 12°)
bool haveMagEver = false;

// ---------- Utils ----------
void pushLineToMonitor(const String& s) {
  lines[lineCount % MAX_BUF_LINES] = s;
  ++lineCount;
}

void clearMonitor() {
  for (int i = 0; i < MAX_BUF_LINES; ++i) lines[i] = "";
  lineCount = 0;
  currentIncomingLine = "";
}

// ---------- Drawing helpers ----------
void drawTopBar() {
  M5.Display.fillRect(topBar.x, topBar.y, topBar.w, topBar.h, TFT_DARKGREY);
  M5.Display.drawRect(topBar.x, topBar.y, topBar.w, topBar.h, TFT_BLACK);

  // Clear
  M5.Display.fillRoundRect(btnClear.x, btnClear.y, btnClear.w, btnClear.h, 8, TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextDatum(MC_DATUM);
  M5.Display.drawString("Clear", btnClear.x + btnClear.w/2, btnClear.y + btnClear.h/2);

  // Baud
  M5.Display.fillRoundRect(btnBaud.x, btnBaud.y, btnBaud.w, btnBaud.h, 8, TFT_BLACK);
  String btxt = String("Baud: ") + String(baudRates[baudIdx]);
  M5.Display.drawString(btxt, btnBaud.x + btnBaud.w/2, btnBaud.y + btnBaud.h/2);

  // CRLF
  M5.Display.fillRoundRect(btnCRLF.x, btnCRLF.y, btnCRLF.w, btnCRLF.h, 8, sendCRLF ? TFT_BLACK : TFT_NAVY);
  String ctxt = sendCRLF ? "CRLF: ON" : "CRLF: OFF";
  M5.Display.drawString(ctxt, btnCRLF.x + btnCRLF.w/2, btnCRLF.y + btnCRLF.h/2);
}

void drawMonitor() {
  // background
  M5.Display.fillRect(monitorRect.x, monitorRect.y, monitorRect.w, monitorRect.h, TFT_BLACK);
  M5.Display.drawRect(monitorRect.x, monitorRect.y, monitorRect.w, monitorRect.h, TFT_DARKGREY);

  // compute how many lines fit
  int lineH = M5.Display.fontHeight();
  int usableH = monitorRect.h - 2*margin;
  int maxLinesOnScreen = usableH / lineH;
  if (maxLinesOnScreen < 1) maxLinesOnScreen = 1;

  int start = lineCount - maxLinesOnScreen;
  if (start < 0) start = 0;

  int y = monitorRect.y + margin;
  M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Display.setTextDatum(TL_DATUM);
  for (int i = start; i < lineCount; ++i) {
    M5.Display.setCursor(monitorRect.x + margin, y);
    M5.Display.println(lines[i % MAX_BUF_LINES]);
    y += lineH;
    if (y > monitorRect.y + monitorRect.h - lineH) break;
  }
}

void drawInputBar() {
  // input rectangle
  M5.Display.fillRect(inputRect.x, inputRect.y, inputRect.w, inputRect.h, TFT_BLACK);
  M5.Display.drawRect(inputRect.x, inputRect.y, inputRect.w, inputRect.h, TFT_DARKGREY);

  // text inside input
  int pad = 6;
  M5.Display.setTextDatum(TL_DATUM);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setCursor(inputRect.x + pad, inputRect.y + (inputRect.h - M5.Display.fontHeight())/2);
  String show = inputBuffer;
  if (show.length() > 80) show = "..." + show.substring(show.length() - 77);
  M5.Display.print(show);

  // Send button
  M5.Display.fillRoundRect(btnSend.x, btnSend.y, btnSend.w, btnSend.h, 8, TFT_BLUE);
  M5.Display.setTextDatum(MC_DATUM);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLUE);
  M5.Display.drawString("Send", btnSend.x + btnSend.w/2, btnSend.y + btnSend.h/2);
}

void drawKey(const Rect& r, const String& label, bool dark=false) {
  uint16_t bg = dark ? TFT_DARKGREY : TFT_NAVY;
  M5.Display.fillRoundRect(r.x, r.y, r.w, r.h, 8, bg);
  M5.Display.drawRoundRect(r.x, r.y, r.w, r.h, 8, TFT_BLACK);
  M5.Display.setTextDatum(MC_DATUM);
  M5.Display.setTextColor(TFT_WHITE, bg);
  M5.Display.drawString(label, r.x + r.w/2, r.y + r.h/2);
}

void drawKeyboard() {
  M5.Display.fillRect(keyboardRect.x, keyboardRect.y, keyboardRect.w, keyboardRect.h, TFT_DARKGREY);
  M5.Display.drawRect(keyboardRect.x, keyboardRect.y, keyboardRect.w, keyboardRect.h, TFT_BLACK);

  int rows = 4; // 3 rows of keys + space row
  int rowH = keyboardRect.h / rows;

  // Row strings depending on mode
  String r1 = numMode ? row1_num : row1_alpha;
  String r2 = numMode ? row2_num : row2_alpha;
  String r3 = numMode ? row3_num : row3_alpha;

  // Row 1
  {
    int n = r1.length();
    int keyW = (keyboardRect.w - 2*margin) / n;
    int y = keyboardRect.y + 0*rowH + margin/2;
    for (int i = 0; i < n; ++i) {
      Rect k{ keyboardRect.x + margin + i*keyW, y, keyW - 2, rowH - margin };
      String label = String(r1[i]);
      if (!numMode && upperCase) label[0] = (char)toupper(label[0]);
      drawKey(k, label);
    }
  }

  // Row 2
  {
    int n = r2.length();
    int keyW = (keyboardRect.w - 2*margin) / n;
    int y = keyboardRect.y + 1*rowH + margin/2;
    for (int i = 0; i < n; ++i) {
      Rect k{ keyboardRect.x + margin + i*keyW, y, keyW - 2, rowH - margin };
      String label = String(r2[i]);
      if (!numMode && upperCase) label[0] = (char)toupper(label[0]);
      drawKey(k, label);
    }
  }

  // Row 3 with special keys: left [Shift]/[ABC], right [Bksp]
  {
    int y = keyboardRect.y + 2*rowH + margin/2;

    String mid = r3;
    int midN = mid.length();
    int totalW = keyboardRect.w - 2*margin;
    int specialW = (totalW / (midN + 4)); // wide-ish specials
    if (specialW < 40) specialW = 40;
    int keyW = (totalW - 2*specialW) / midN;

    // Left special
    Rect left{ keyboardRect.x + margin, y, specialW - 2, rowH - margin };
    drawKey(left, numMode ? "ABC" : (upperCase ? "Shift▲" : "Shift"), true);

    // Middle chars
    int x = left.x + left.w + 2;
    for (int i = 0; i < midN; ++i) {
      Rect k{ x + i*keyW, y, keyW - 2, rowH - margin };
      String label = String(mid[i]);
      if (!numMode && upperCase) label[0] = (char)toupper(label[0]);
      drawKey(k, label);
    }

    // Right special
    Rect right{ keyboardRect.x + keyboardRect.w - margin - specialW, y, specialW - 2, rowH - margin };
    drawKey(right, "Bksp", true);
  }

  // Row 4: Space
  {
    int y = keyboardRect.y + 3*rowH + margin/2;
    int totalW = keyboardRect.w - 2*margin;
    int spaceW = totalW * 70 / 100;
    int x = keyboardRect.x + margin + (totalW - spaceW)/2;
    Rect space{ x, y, spaceW, rowH - margin };
    drawKey(space, "Space");
  }
}

// ---------- Layout / Redraw ----------
void layout() {
  W = M5.Display.width();
  H = M5.Display.height();

  int topH = 48;
  int inputH = 52;
  int kbH   = H / 3; // bottom third for keyboard

  topBar   = {0, 0, W, topH};
  int btnW = 140, btnH = topH - 10;
  btnClear = {margin, topBar.y + 5, btnW, btnH};
  btnBaud  = {W/2 - btnW/2, topBar.y + 5, btnW, btnH};
  btnCRLF  = {W - margin - btnW, topBar.y + 5, btnW, btnH};

  monitorRect = {margin, topBar.y + topBar.h + margin, W - 2*margin, H - topH - inputH - kbH - 3*margin};

  inputRect = {margin, monitorRect.y + monitorRect.h + margin, W - 2*margin - 130, inputH};
  btnSend   = {inputRect.x + inputRect.w + margin, inputRect.y, 130 - margin, inputH};

  keyboardRect = {margin, inputRect.y + inputRect.h + margin, W - 2*margin, kbH};
}

void redrawUI() {
  drawTopBar();
  drawMonitor();
  drawInputBar();
  drawKeyboard();
}

// ---------- Serial setup ----------
void applyBaud() {
  uint32_t b = baudRates[baudIdx];
  SerialExt.flush();
  SerialExt.end();
  delay(10);
  SerialExt.begin(b, SERIAL_8N1, RX_PIN, TX_PIN);
}

// ---------- Orientation helpers ----------
static inline float rad2deg(float r) { return r * 57.2957795f; }

static inline float angdist(float a, float b) {
  // distance angulaire minimale (0..180)
  float d = fmodf(fabsf(a - b), 360.f);
  return (d > 180.f) ? 360.f - d : d;
}

// Map heading -> 4 orientations (N/S/E/O)
UiOrient orientationFromHeading4(float deg) {
  // Normalise 0..360
  while (deg < 0) deg += 360.f;
  while (deg >= 360.f) deg -= 360.f;

  // Cœurs des secteurs (±SECTOR_HALF_CORE)
  float dN = angdist(deg,   0.f);
  float dE = angdist(deg,  90.f);
  float dS = angdist(deg, 180.f);
  float dW = angdist(deg, 270.f);

  if (dN <= SECTOR_HALF_CORE) return UiOrient::PortraitUp;
  if (dS <= SECTOR_HALF_CORE) return UiOrient::PortraitDown;
  if (dE <= SECTOR_HALF_CORE) return UiOrient::LandscapeRight;
  if (dW <= SECTOR_HALF_CORE) return UiOrient::LandscapeLeft;

  // Zone grise : garder courant
  return currentOrient;
}

// Lecture du magnétomètre si dispo (retourne false si pas fiable)
bool readHeadingDeg(float &outDeg) {
  float mx=0, my=0, mz=0;
  M5.Imu.getMag(&mx, &my, &mz);
  float norm = sqrtf(mx*mx + my*my + mz*mz);
  if (!isfinite(norm) || norm < 5.f) return false; // seuil empirique
  haveMagEver = true;

  float heading = atan2f(my, mx); // -pi..pi
  outDeg = rad2deg(heading);
  if (outDeg < 0) outDeg += 360.f;
  return true;
}

// Fallback par accélération -> 4 orientations (axe dominant + signe)
// NOTE: selon le montage, il peut être nécessaire d’inverser un signe.
// Si comportement inversé, échangez les branches '>' / '<' ci-dessous.
UiOrient orientationFromAccel4() {
  float ax=0, ay=0, az=0;
  M5.Imu.getAccel(&ax, &ay, &az);
  if (!isfinite(ax) || !isfinite(ay)) return currentOrient;

  if (fabsf(ax) > fabsf(ay)) {
    // Paysage (axe X dominant)
    return (ax >= 0) ? UiOrient::LandscapeLeft : UiOrient::LandscapeRight;
  } else {
    // Portrait (axe Y dominant)
    return (ay >= 0) ? UiOrient::PortraitUp : UiOrient::PortraitDown;
  }
}

// Conversion orientation -> setRotation()
uint8_t rotationFromUi(UiOrient o) {
  // TFT_eSPI/M5 : 0=PortraitUp, 1=LandscapeRight, 2=PortraitDown, 3=LandscapeLeft (en général)
  switch (o) {
    case UiOrient::PortraitUp:     return 0;
    case UiOrient::LandscapeRight: return 1;
    case UiOrient::PortraitDown:   return 2;
    case UiOrient::LandscapeLeft:  return 3;
  }
  return 1;
}

// Applique la rotation écran + effacement + relayout/redraw
void applyRotation(UiOrient o) {
  if (o == currentOrient) return;

  uint8_t rot = rotationFromUi(o);
  M5.Display.setRotation(rot);

  // *** Effacement complet demandé ***
  M5.Display.fillScreen(TFT_BLACK);

  currentOrient = o;

  layout();
  redrawUI();

  // Trace utilisateur
  const char* oname =
    (o == UiOrient::PortraitUp) ? "Portrait Up" :
    (o == UiOrient::PortraitDown) ? "Portrait Down (180°)" :
    (o == UiOrient::LandscapeRight) ? "Landscape Right" :
    "Landscape Left (180°)";
  pushLineToMonitor(String("[Orientation -> ") + oname + "]");
  drawMonitor();
}

// Vérifie régulièrement et décide d’appliquer après stabilisation
void pollAndMaybeRotate() {
  uint32_t now = millis();
  if (now - lastOrientPoll < ORIENT_CHECK_MS) return;
  lastOrientPoll = now;

  float deg = NAN;
  bool magOk = readHeadingDeg(deg);

  if (magOk) {
    targetOrient = orientationFromHeading4(deg);
  } else {
    targetOrient = orientationFromAccel4();
  }

  // anti-flip : on attend que la cible reste identique pendant ORIENT_APPLY_DELAY_MS
  static UiOrient lastSeen = currentOrient;
  if (targetOrient != lastSeen) {
    lastSeen = targetOrient;
    orientStableSince = now;
  }
  if (now - orientStableSince >= ORIENT_APPLY_DELAY_MS && targetOrient != currentOrient) {
    applyRotation(targetOrient);
  }
}

// ---------- Touch/key hit-testing ----------
KeyType hitTestKeyboard(int tx, int ty, char &outChar) {
  if (!keyboardRect.contains(tx, ty)) return KT_NONE;

  int rows = 4;
  int rowH = keyboardRect.h / rows;
  int localY = ty - keyboardRect.y;
  int rowIdx = localY / rowH;

  // row strings for current mode
  String r1 = numMode ? row1_num : row1_alpha;
  String r2 = numMode ? row2_num : row2_alpha;
  String r3 = numMode ? row3_num : row3_alpha;

  auto applyCase = [&](char c) -> char {
    if (!numMode && upperCase && c >= 'a' && c <= 'z') return (char)toupper(c);
    return c;
  };

  // Row 1
  if (rowIdx == 0) {
    int n = r1.length();
    int keyW = (keyboardRect.w - 2*margin) / n;
    int idx = (tx - (keyboardRect.x + margin)) / keyW;
    if (idx >= 0 && idx < n) {
      outChar = applyCase(r1[idx]);
      return KT_CHAR;
    }
    return KT_NONE;
  }

  // Row 2
  if (rowIdx == 1) {
    int n = r2.length();
    int keyW = (keyboardRect.w - 2*margin) / n;
    int idx = (tx - (keyboardRect.x + margin)) / keyW;
    if (idx >= 0 && idx < n) {
      outChar = applyCase(r2[idx]);
      return KT_CHAR;
    }
    return KT_NONE;
  }

  // Row 3: left special, middle chars, right bksp
  if (rowIdx == 2) {
    String mid = r3;
    int midN = mid.length();
    int totalW = keyboardRect.w - 2*margin;
    int specialW = (totalW / (midN + 4));
    if (specialW < 40) specialW = 40;
    int keyW = (totalW - 2*specialW) / midN;

    int xL = keyboardRect.x + margin;
    int xMid = xL + specialW;
    int xR = xMid + midN * keyW;
    int xRBeg = keyboardRect.x + keyboardRect.w - margin - specialW;

    if (tx >= xL && tx < xL + specialW) {
      // left special: Shift or ABC
      return numMode ? KT_MODE : KT_SHIFT;
    } else if (tx >= xMid && tx < xR) {
      int idx = (tx - xMid) / keyW;
      if (idx >= 0 && idx < midN) {
        outChar = applyCase(mid[idx]);
        return KT_CHAR;
      }
    } else if (tx >= xRBeg && tx < xRBeg + specialW) {
      return KT_BKSP;
    }
    return KT_NONE;
  }

  // Row 4: Space
  if (rowIdx == 3) {
    int totalW = keyboardRect.w - 2*margin;
    int spaceW = totalW * 70 / 100;
    int x = keyboardRect.x + margin + (totalW - spaceW)/2;
    if (tx >= x && tx < x + spaceW) {
      return KT_SPACE;
    }
    return KT_NONE;
  }

  return KT_NONE;
}

void handleTouch(int tx, int ty) {
  // Top bar buttons
  if (btnClear.contains(tx, ty)) {
    clearMonitor();
    drawMonitor();
    return;
  }
  if (btnBaud.contains(tx, ty)) {
    baudIdx = (baudIdx + 1) % (int)(sizeof(baudRates)/sizeof(baudRates[0]));
    applyBaud();
    drawTopBar();
    pushLineToMonitor(String("[Baud changed to ") + baudRates[baudIdx] + "]");
    drawMonitor();
    return;
  }
  if (btnCRLF.contains(tx, ty)) {
    sendCRLF = !sendCRLF;
    drawTopBar();
    return;
  }

  // Send button
  if (btnSend.contains(tx, ty)) {
    if (inputBuffer.length() > 0) {
      String payload = inputBuffer;
      if (sendCRLF) payload += "\r\n";
      SerialExt.print(payload);
      Serial.print(">> "); Serial.println(inputBuffer);
      pushLineToMonitor(">> " + inputBuffer);
      inputBuffer = "";
      drawInputBar();
      drawMonitor();
    }
    return;
  }

  // Keyboard
  char ch = 0;
  KeyType kt = hitTestKeyboard(tx, ty, ch);
  switch (kt) {
    case KT_CHAR:
      inputBuffer += ch;
      drawInputBar();
      break;
    case KT_SPACE:
      inputBuffer += ' ';
      drawInputBar();
      break;
    case KT_BKSP:
      if (inputBuffer.length() > 0) inputBuffer.remove(inputBuffer.length() - 1);
      drawInputBar();
      break;
    case KT_SHIFT:
      upperCase = !upperCase;
      drawKeyboard();
      break;
    case KT_MODE:
      numMode = !numMode;
      if (numMode) upperCase = false;
      drawKeyboard();
      break;
    default:
      break;
  }
}

// ---------- Setup / Loop ----------
void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = BAUD_USB;
  cfg.clear_display = true;
  M5.begin(cfg);

  M5.Display.setTextSize(2);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.fillScreen(TFT_BLACK);

  // Orientation initiale via accéléro (robuste au boot) -> 4 orientations
  currentOrient = orientationFromAccel4();
  applyRotation(currentOrient); // setRotation + clear + layout + redraw

  Serial.begin(BAUD_USB);
  SerialExt.begin(baudRates[baudIdx], SERIAL_8N1, RX_PIN, TX_PIN);

  pushLineToMonitor("Ready. USB <-> UART bridge on GPIO38/GPIO37.");
  pushLineToMonitor("Tap 'Baud' to change speed. Use the keyboard to type.");
  pushLineToMonitor("[Auto-rotate 4-way enabled: Compass>Accel fallback]");
  redrawUI();

  orientStableSince = millis();
}

void loop() {
  // UART ext -> USB + monitor
  while (SerialExt.available()) {
    int c = SerialExt.read();
    Serial.write(c);

    if (c == '\r') {
      // ignore CR here; we add lines on LF
    } else if (c == '\n') {
      pushLineToMonitor(currentIncomingLine);
      currentIncomingLine = "";
      drawMonitor();
    } else if (c == '\b') {
      if (!currentIncomingLine.isEmpty())
        currentIncomingLine.remove(currentIncomingLine.length() - 1);
    } else {
      currentIncomingLine += (char)c;
      if (currentIncomingLine.length() > 240) {
        pushLineToMonitor(currentIncomingLine);
        currentIncomingLine = "";
        drawMonitor();
      }
    }
  }

  // USB -> UART ext (PC monitor can talk to target)
  while (Serial.available()) {
    SerialExt.write(Serial.read());
  }

  // Touch handling
  M5.update();
  uint16_t tx, ty;
  if (M5.Display.getTouch(&tx, &ty)) {
    handleTouch(tx, ty);
    delay(120); // debounce
  }

  // Auto-rotation (boussole / fallback)
  pollAndMaybeRotate();
}
