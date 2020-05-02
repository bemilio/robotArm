//velocità espressa in mm / s
//il robot si può usare solo in un semipiano, diciamo per convenzione solo x positivo, y qualsiasi segno, z solo positivo. da aggiungere i filtri per controllare questo
//da aggiungere: handshake per la com. seriale
#include <math.h>
#include <Servo.h>
#define MINPWMBASE 810   //servi digitali standard
#define MAXPWMBASE 2180 
#define PWM_ON_ANGLE_SERVO_0  9.8 //9.8
#define PWM_90_SERVO_0 1485 //1650
#define PWM_ON_ANGLE_SERVO_1  9.8 //9.8
#define PWM_90_SERVO_1 1837
#define PWM_ON_ANGLE_SERVO_2  10.38 //prima era 11.111
#define PWM_90_SERVO_2 1220  //PWM_180 è 1820, ed è più facile da misurare
#define PWM_ON_ANGLE_SERVO_3  10.38 //prima era 11.111
#define PWM_90_SERVO_3 950
#define MINPWMARM 550 
#define MAXPWMARM 2500
#define NUMSERVO 4 //numero dei servi
#define N_PARAMETERS 5
#define TEMPOLIMITE 1000
#define BUFLEN 20
#define NUMJOINTS 4
#define LEN1 124  //lunghezza braccio 1 in millimetri
#define LEN2 117
#define LEN3 60
#define MAXDIST LEN1+LEN2
#define pi 3.14159265
#define CTS_PIN 3
#define DIM_QUEUE 200
//correzioni

//correzioni sul piano (ossia considerando J0 in posizione. Sono dovute a offset sull' end eff. e sul punto d' origine)
#define PLANE_CORR_X 7 //ossia il punto è da considerare più lontano di tot
#define PLANE_CORR_Y -65 //ossia il punto è da considerare più in alto di tot. da integrare con LEN3

float coeff[NUMJOINTS];
float q[NUMJOINTS]; 
float corr_angle[NUMJOINTS] = {0, 0, 0, 0}; //correzione angoli dei motori. I valori verranno sommati a quelli ottenuti dall' algoritmo
float maxangle[NUMJOINTS] = {180, 150, 180, 220}; //massimo valore degli angoli dei 4 joints
Servo motor[NUMJOINTS];
int servopin[NUMJOINTS] = {6, 9, 10, 11};
float param[N_PARAMETERS]; //via seriale arrivano degli int, ma li converto in float tramite conversione esplicita in GetParameters
int ext = 0;
int time = 0;
char chr = NULL;
String s = "";
int i = 0, j = 0;
float t_in, t_fin, t_delta;
float posit_in[3] = {60, 0, 80};
float posit_fin[3];
float vel;
double angles[NUMJOINTS];
float X_now, Y_now, Z_now;
int pinEM = 13;
//5th grade interpolation coefficients
float a[3], b[3], c[3], d[3], e[3], f[3];
int queue[DIM_QUEUE];
int head = -1;
int tail = -1;
int par_available = 0;
boolean glitch_flag = 1; //raspi manda uno zero di troppo all' inizio, quando arduino lo scarta abbasso il flag

//////////////////////INIZIO PROGRAMMA//////////////////////
void setup() {
  for (i = 0; i < NUMJOINTS; i++) {
    motor[i].attach(servopin[i]); //Vedi reference per i parametri che impostano valori min e max per la PWM
  }
  pinMode(CTS_PIN, OUTPUT);
  digitalWrite(CTS_PIN, LOW);
  Serial.begin(9600);
  pinMode(pinEM, OUTPUT);
  while (Serial.available())
    Serial.read();
  SetupServos(); //ricava la PWM (teorica) a 0 gradi e il rapporto variazionePWM/var. angolo per ogni motore
  GetAngles(posit_in[0], posit_in[1], posit_in[2], angles); //li mette alla posizione iniziale
  SendPWM(angles);
  WaitForRaspi(); //aspetta il segnale della raspberry pi (tramite seriale, "here")
}
void SetupServos(){
  coeff[0] = PWM_ON_ANGLE_SERVO_0;
  q[0] = PWM_90_SERVO_0 - coeff[0] * 90;//sapendo il valore di pwm a 90° e l' incremento della pwm in funzione dell' angolo calcola la pwm associata a 0
  coeff[1] = PWM_ON_ANGLE_SERVO_1;
  q[1] = PWM_90_SERVO_1 - coeff[1] * 90;
  coeff[2] = PWM_ON_ANGLE_SERVO_2;
  q[2] = PWM_90_SERVO_2 - coeff[2] * 90;
  coeff[3] = PWM_ON_ANGLE_SERVO_3;
  q[3] = PWM_90_SERVO_3 - coeff[3] * 90;
  //for(int i=0; i<NUMJOINTS; i++){
  //  double value=((90*coeff[i])+q[i]);
  //  motor[i].writeMicroseconds(value);
  //}
}
void WaitForRaspi(){
  int here=0;
  while(!here){
    if(Serial.available()){
      delay(30);
      //ReadString();
      //if (s.equals("here")){
      //  s="";
      Serial.println("raspi e' arrivata");
      here = 1;
      delay(70);
      while(Serial.available())
        Serial.read();
    }
  }
}

//INIZIO LOOP//
//implem.coda//
void EnQueue (int elem) {
  tail = (tail + 1) % DIM_QUEUE;
  queue[tail] = elem;
}

int DeQueue () {
  head = (head + 1) % DIM_QUEUE;
  return (queue[head]);
}
boolean empty() {
  return (head == tail);
}
boolean full() {
  int numel;
  numel = (tail >= head) ? (tail - head) : (tail + DIM_QUEUE - head); /*3*/
  if (numel == DIM_QUEUE - 1) /*4*/
    return 1;
  else
    return 0;
}
//fine implem. coda//
//funzione che stampa un array di parametri
void printParameters(int num, double par[], char name[], int spec) { 
  //spec: inserire un numero per specificare i decimali di un float,
  //inserire DEC, ecc, per altre specifiche, altrimenti mettere 0
  Serial.println();
  Serial.println("////");
  for (int k = 0; k < num; k++) {
    Serial.print(name);
    Serial.print(" ");
    Serial.print(k + 1);
    Serial.print(": ");
    if (spec)
      Serial.print(par[k], spec);
    else
      Serial.print(par[k]);
    Serial.print("  ;  ");
  }
}
//funzione che ritorna secondi con precisione al millisecondo
float sec() {
  unsigned long ms = millis();
  float s = ((float)ms / 1000);
  return (s);
}
void bufferClean(){
  while(Serial.peek()=='\n' || Serial.peek()=='\0') //sembra necessario pulire il buffer il più spesso possibile
    Serial.read();
}


int spacchettaStringa(){
  float par = (float) s.toFloat();
  Serial.println(par);
  //delay(20);
  //ReadString();
  //if (s.equals("err")){
  //}
  //else{
  chr = NULL;
  //Serial.print("il parametro e' : ");
  //Serial.println(par);
  s = "";
  if(glitch_flag && par == 0){ //il raspi manda zeri di troppo all' inizio, li scarto e bonocchiù
    Serial.println("dato scartato");
  }
  else{
    if(glitch_flag){
      glitch_flag = 0;
      Serial.println("glitch risolto");
    }
    if (par == 10000){
      Serial.println("ricevuto dato di termine");
      return (1);
    }
    Serial.println("Enque");
    EnQueue(par);
    par_available++;
    return(0);
  }
}

void ReadSerial() { 
  digitalWrite(CTS_PIN, HIGH); //indica che è pronto a ricevere
  if (Serial.available()){
    delay(40);
    chr = (char)Serial.read();
    s += chr;
    if (chr == '\n'){
      digitalWrite(CTS_PIN, LOW);
      spacchettaStringa();
      bufferClean();
    }
  }
}

boolean GetParameters() {
  if (par_available >= N_PARAMETERS) {
    for (int i = 0; i < N_PARAMETERS; i++) {
      param[i] = DeQueue();
      Serial.print("in coda ho: ");
      Serial.println(param[i]);
    }

    par_available = par_available - N_PARAMETERS;
    return (1);
  }
  else
    return (0);
}

float GetTfin(float fin[], float init[], float v) {
  if (v <= 10)
    v = 10;
  //t_fin=sqrt((x_fin-x_in)^2+(y_fin....)/vel
  float module ;
  float space = ((fin[0] - init[0]) * (fin[0] - init[0])) + ((fin[1] - init[1]) * (fin[1] - init[1])) + ((fin[2] - init[2]) * (fin[2] - init[2]));
  module = (sqrt(space) / v);
  //Serial.print ("distanza: ");
  //Serial.println(space);
  return (module);
}

void GetInterp(float fin[], float init[], float t) {
  for (i = 0; i < 3; i++) {
    a[i] = (float)((6 * (fin[i] - init[i])) / (pow(t, 5)));
    b[i] = ((float)(-15)) * ((fin[i] - init[i]) / (pow(t, 4)));
    c[i] = (10) * ((fin[i] - init[i]) / (pow(t, 3)));
    d[i] = 0;
    e[i] = 0;
    f[i] = init[i];
    float tempint[6] = {a[i], b[i], c[i], d[i], e[i], f[i]};
    //printParameters(6, tempint , "interp",5);
  }
}
float GetX(float t, float ap, float bp, float cp, float dp, float ep, float fp) {
  float x;
  x = ((ap * pow(t, 5)) + (bp * pow(t, 4)) + (cp * pow(t, 3)) + (dp * pow(t, 2)) + (ep * pow(t, 1)) + fp);
  return x;
}
//opera la cinematica inversa
double GetAngles(float x, float y, float z, double angles[]) {
  float L1 = LEN1;
  float L2 = LEN2;
  float x_square = x*x;
  float y_square = y*y;
  float r = sqrt(x_square + y_square);
  angles[0] = (atan (y / x)) + ((float)pi / 2);
  //ora che sono sul piano applico le correzioni
  r = r+PLANE_CORR_X;
  z = z + PLANE_CORR_Y + LEN3; //ci ho messo y perchè nel piano la z dventa y
  angles[2] = pi - acos(((r * r) + z*z - L1*L1 - L2*L2) / ((float)2 * LEN1 * LEN2)); //quando è 180 il braccio è dritto
  double C = cos(pi - angles[2]);
  double S = sin(pi - angles[2]);
  angles[1] = acos(((r * (L1 + L2 * C)) - (L2 * S * z)) / (pow((L2 * S), 2) + pow((L1 + L2 * C), 2)));
  angles[3] = (pi / 2) - angles[1] + (pi-angles[2]); //quando è pi/2 il braccio è dritto
  float fltangles[3];
  for (i = 0; i < 4; i++) {
    angles[i] = ((angles[i] * 180) / pi) + corr_angle[i];
    if (angles[i] > maxangle[i]) {
      Serial.println();
      Serial.print("ERRORE: ANGOLO TROPPO GRANDE al motore ");
      Serial.print(i);
      Serial.print(" l' angolo e' ");
      Serial.println(angles[i]);
      angles[i] = maxangle[i];
    }
  }
  //Serial.println();
  //Serial.print(" angle 3 : ");
  //Serial.print ((angles[2]));
  //Serial.print(" angle 2 : ");
  //Serial.print (angles[1]);
}
void SendPWM(double ang[]) {
  double value[NUMJOINTS];
  int intvalue[NUMJOINTS];
  int mx, mn;
  for (i = 0; i < NUMJOINTS; i++) {
    if (i < 2) {
      mx = MAXPWMBASE;
      mn = MINPWMBASE;
    }
    else
    {
      mx = MAXPWMARM;
      mn = MINPWMARM;
    }
    value[i] = ((ang[i]*coeff[i])+q[i]);
    intvalue[i] = (int)value[i];
    if (intvalue[i] <= mx && intvalue[i] >= mn)
      motor[i].writeMicroseconds(value[i]);
    else {
      Serial.println("ERRORE MANDANDO PWM: ");
      Serial.print("motore ");
      Serial.print(i);
      Serial.print("  :");
      Serial.println(ang[i]);
    }
  }
}
void LessPrecisePWM(double ang[]) {
  for (i = 0; i < NUMJOINTS; i++) {
    if (ang[i] >= 0 && ang[i] <= 180)
      motor[i].write((int)ang[i]);
    else {
      Serial.println("ERRORE MANDANDO PWM: ");
      Serial.print("angolo ");
      Serial.print(i);
      Serial.print("  :");
      Serial.println(ang[i]);
    }
  }
}
boolean TooDistant(float newx, float newy, float newz) {
  float module;
  if (newz<0)
    return(1);
  newz = newz + PLANE_CORR_Y + LEN3; //serve perchè lo sto impostando per prendere le cose sempre dall' alto
  module = sqrt((newx*newx)+(newy*newy));
  module = sqrt(module*module + (newz*newz));
  if (module > (LEN1 + LEN2))
    return (1);
  return (0);
}

void loop() {
  ReadSerial();
  boolean newpar = GetParameters();
  if (newpar) {
    if (!TooDistant(param[0], param[1], param[2])) {
      for (i = 0; i < 3; i++)
        posit_fin[i] = param[i];
      //posit_fin[2]= posit_fin[2] + LEN 3; //visto che l' ultimo joint è fatto per essere perpendicolare al terreno, lavoro sulla z sommata la lunghezza del 3° pezzo
      vel = param[3];
      if (param[4])
        digitalWrite(pinEM, HIGH);
      else
        digitalWrite(pinEM, LOW);
      //parameters are: [x_fin, y_fin, z_fin, vel, straight(boolean)]
      t_fin = GetTfin(posit_fin, posit_in, vel); //tempo finale in secondi
      Serial.println("tempo da impiegare: ");
      Serial.println(t_fin);
      GetInterp(posit_fin, posit_in, t_fin); //ricava i parametri per l' interpolazione a polinomio di grado 5
      t_in = sec();
      t_delta = (sec() - t_in);
      while (t_delta < t_fin) {
        X_now = GetX(t_delta, a[0], b[0], c[0], d[0], e[0], f[0]); //ricava la posizione al tempo attuale
        Y_now = GetX(t_delta, a[1], b[1], c[1], d[1], e[1], f[1]);
        Z_now = GetX(t_delta, a[2], b[2], c[2], d[2], e[2], f[2]);
        GetAngles(X_now, Y_now, Z_now, angles);
        // float temppos[3] = {X_now, Y_now, Z_now};
        //printParameters(3, temppos, "Pos_asse", 7);
        SendPWM(angles);
        t_delta = ((sec()) - t_in);
      }
      GetAngles(posit_fin[0], posit_fin[1], posit_fin[2], angles);
      printParameters(4, angles, "angolo", 0);
      SendPWM(angles);
      for (i = 0; i < 3; i++)
        posit_in[i] = posit_fin[i];
      newpar = 0;
      Serial.println("movimento finito");
    }
    else {
      Serial.println("too distant");
      newpar = 0;
    }
    //sendXON();
  }
}
