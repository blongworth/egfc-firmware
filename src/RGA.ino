
float Read_Status_RGA (char *x, int a, int b) { // function declaration {
  RGA_SERIAL.write(x);
  delay(50);
  // for loop from a to b
  const int BUFFER_SIZE = 30;
  char Var1[BUFFER_SIZE];
  char VarOut[6];
  //int rlen
  RGA_SERIAL.readBytesUntil(13, Var1, BUFFER_SIZE);
  for ( int i = a; i < b; ++i ) {
    VarOut[i - a] = Var1[ i ];
  }
  float VarNum = atof(VarOut);
  //  String s = String(Var1);
  //  int VarNum = roundf(s.toFloat());
  return VarNum; // return the value
}


/// Set noise floor
void setNF(int NF)
{
  char s[10];
  sprintf(s, "NF%d\r", NF);
  RGA_SERIAL.write(s);
  delay(25);
}

// Measure a single mass
void RGA_ScanO(int MR)
{
  // Clear the receiving buffer
  rga_serial_flush();
    
  // send scan command to instrument
  char s[10];
  sprintf(s, "MR%d\r", MR);
  RGA_SERIAL.write(s);
  delay(25);
}

// read data from RGA
int RGA_ScanI() {
  char RGA[4];
  RGA_SERIAL.readBytes(RGA, 4);
  int num = RGA[0] + RGA[1] * 256 + RGA[2] * 65536 + RGA[3] * 16777216;
  return num;
}

// Get total pressure
//void RGA_TP() {
//  char s[10];
//    RGA_SERIAL.write("TP?\r");
//    delay(25);
//}
