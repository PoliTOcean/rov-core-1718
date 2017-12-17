/* Create a WiFi access point and provide a web server on it. */

#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>

/* Set these to your desired credentials. */
const char *ssid = "wifi";
const char *password = "password";

ESP8266WebServer server(80);

/* Just a little test message.  Go to http://192.168.4.1 in a web browser
 * connected to this access point to see it.
 */
void handleRoot() {
	server.send(200, "text/html", "<h1>You are connected</h1>");
}

void handleTest() {
  char message[2];
  sprintf(message, "%d", random(100));
  server.send(200, "text/plain", message);
}

void handleTestNew() {
  char message[2];
  server.send(200, "text/plain", "0 0\n1.1 0\n1.3 4.4\n1.4 -4.3\n1.5 0\n2 0\n2.2 6.8\n2.4 -7\n2.6 0\n3 0\n3.1 1.6\n3.2 -1.6\n3.3 0\n3.9 0");
}

void setup() {
	delay(1000);
	Serial.begin(115200);
	Serial.println();
	Serial.print("Configuring access point...");
	/* You can remove the password parameter if you want the AP to be open. */
	WiFi.softAP(ssid, password);

	IPAddress myIP = WiFi.softAPIP();
	Serial.print("AP IP address: ");
	Serial.println(myIP);
	server.on("/", handleRoot);
  server.on("/test", handleTest);
  server.on("/test_new", handleTestNew);
	server.begin();
	Serial.println("HTTP server started");
}

void loop() {
	server.handleClient();
}
