// ===============================================
// SenML Builder
// ===============================================

String senml = "";

bool setupSenML() {
  senml.reserve(4096);
}

// Clears the current SenML message and begins a new one.
void beginSenML() {
  senml = "{'e':[";
}

void appendIntSenML(String name, int value) {
  senml = senml + constructSenMLFieldStart(name);
  senml = senml + value;
  senml = senml + "},";
}

// The last measurement...
void appendLastIntSenML(String name, int value) {
    senml = senml + constructSenMLFieldStart(name);
    senml = senml + value;
    senml = senml + "}";
}

void appendFloatSenML(String name, float value) {
    senml = senml + constructSenMLFieldStart(name);
    senml = senml + value;
    senml = senml + "},";
}

void appendUInt8SenML(String name, uint8_t value) {
    senml = senml + constructSenMLFieldStart(name);
    senml = senml + value;
    senml = senml + "},";
}

void appendUInt16SenML(String name, uint16_t value) {
    senml = senml + constructSenMLFieldStart(name);
    senml = senml + value;
    senml = senml + "},";
}

void appendStringSenML(String name, String value) {
  senml = senml + "{'n':'" + name + "', 'sv':'";
  senml = senml + value;
  senml = senml + "'},";
}

String constructSenMLFieldStart(String name) {
  return "{'n':'" + name + "', 'v':";
}

// Terminate the SenML json string.
String terminateSenML() {
  // Dummy final to avoid trailing ,
  senml = senml + "{'n':'end', 'sv':''}";
  // Terminate the json and send the senml to Tinamous
  senml = senml +  "]}";

  return senml;
}
