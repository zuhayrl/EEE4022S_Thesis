#include <WiFi.h>
#include <HardwareSerial.h>
#include <SD.h>
#include <SPI.h>

// SD Card pins
#define SD_CS   5
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK  18

// SD Card speed (lower = more reliable, higher = faster)
#define SD_SPEED 400000  // 400kHz (safe), can try 4000000 (4MHz) if stable


#define RE_DE 27
HardwareSerial RS485(1);

String latestMessage = "No data yet";

// AP credentials
const char *apSSID = "Salinity";
const char *apPassword = "salinity";

WiFiServer server(80);

void setup() {
  Serial.begin(115200);

  // Initialize SD card
  if (!SD_init()) {
    Serial.println("Cannot continue without SD card!");
    //while(1) delay(1000);
  }

  // RS485 setup
  RS485.begin(9600, SERIAL_8N1, 26, 25); // RX=26, TX=25
  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, LOW); // Start in receive mode

  // Wi-Fi Access Point setup
  //Serial.println("Starting Access Point...");
  //WiFi.softAP(apSSID, apPassword);

  //IPAddress IP = WiFi.softAPIP();
  //Serial.print("AP IP Address: ");
  //Serial.println(IP);

  //server.begin();
  Serial.println("Setup complete. Type messages below to send via RS485.");
}

void loop() {
  // ---------- Handle Serial Input ----------
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      Serial.print("Sending via RS485: ");
      Serial.println(cmd);

      // Enable transmit mode
      digitalWrite(RE_DE, HIGH);
      delay(2); // Allow line to stabilize

      RS485.println(cmd);
      delay(50);

      // Switch back to receive mode
      digitalWrite(RE_DE, LOW);
      Serial.println("Message sent, waiting for response...");
    }
  }

  // ---------- Handle RS485 Incoming Data ----------
  if (RS485.available()) {
    latestMessage = RS485.readStringUntil('\n');
    latestMessage.trim();
    Serial.print("Received from RS485: ");
    Serial.println(latestMessage);
    if (latestMessage.startsWith("msg: ")){
      //SD_appendString("/data.csv",latestMessage);
    }
  }

  // ---------- Handle Web Server ----------
  /*WiFiClient client = server.available();
  if (client) {
    //Serial.println("New Client Connected.");
    String req = client.readStringUntil('\r');
    client.flush();

    // Simple HTML page with auto-refresh
    String html = "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='2'/>"
                  "<title>RS485 Data</title></head><body>"
                  "<h1>Latest RS485 Message</h1>"
                  "<p>" + latestMessage + "</p>"
                  "</body></html>";

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println(html);
    client.stop();
    //Serial.println("Client Disconnected.");
  }*/
}

/*
 * Initialize SD card
 * Returns: true if successful, false if failed
 */
bool SD_init() {
  Serial.println("Initializing SD card...");
  
  // Initialize SPI with custom pins
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  
  // Try to begin SD card
  if (!SD.begin(SD_CS, SPI, SD_SPEED)) {
    Serial.println("SD Card initialization FAILED!");
    return false;
  }
  
  Serial.println("SD Card initialized successfully!");
  Serial.printf("Card Type: %d\n", SD.cardType());
  Serial.printf("Card Size: %llu MB\n", SD.cardSize() / (1024 * 1024));
  
  return true;
}


/*
 * Write string to file (overwrites existing content)
 * filepath: Must start with "/" e.g. "/data.txt"
 * content: String to write
 * Returns: true if successful, false if failed
 */
bool SD_writeString(const char* filepath, String content) {
  File file = SD.open(filepath, FILE_WRITE);
  
  if (!file) {
    Serial.print("Failed to open file for writing: ");
    Serial.println(filepath);
    return false;
  }
  
  // Truncate file (clear existing content)
  file.seek(0);
  
  // Write string
  size_t bytesWritten = file.print(content);
  file.close();
  
  if (bytesWritten > 0) {
    Serial.print("Written to ");
    Serial.print(filepath);
    Serial.print(": ");
    Serial.print(bytesWritten);
    Serial.println(" bytes");
    return true;
  } else {
    Serial.print("Failed to write to: ");
    Serial.println(filepath);
    return false;
  }
}

/*
 * Append string to file (adds to end without erasing)
 * filepath: Must start with "/" e.g. "/data.txt"
 * content: String to append
 * Returns: true if successful, false if failed
 */
bool SD_appendString(const char* filepath, String content) {
  File file = SD.open(filepath, FILE_APPEND);
  
  if (!file) {
    Serial.print("Failed to open file for appending: ");
    Serial.println(filepath);
    return false;
  }
  
  // Write string at end of file
  size_t bytesWritten = file.print(content);
  file.close();
  
  if (bytesWritten > 0) {
    Serial.print("Appended to ");
    Serial.print(filepath);
    Serial.print(": ");
    Serial.print(bytesWritten);
    Serial.println(" bytes");
    return true;
  } else {
    Serial.print("Failed to append to: ");
    Serial.println(filepath);
    return false;
  }
}


