//velocità espressa in mm / s
//il robot si può usare solo in un semipiano, diciamo per convenzione solo x positivo, y qualsiasi segno, z solo positivo. da aggiungere i filtri per controllare questo
//da aggiungere: handshake per la com. seriale
//g16 switcha a coordinate cilindriche, g15 torna a coord cartesiane
//g91: coord relative g90: coord assolute
//g1: velocità controllata g0: velocità non controllata (OCCHIO: USALA SOLO CON UN PID A BASSO GAIN)
#include <math.h>
#include <Servo.h>
#define TAU 1 //costante di tempo interp esponenz
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
#define BUFLEN 40
#define NUMJOINTS 4
#define LEN1 124  //lunghezza braccio 1 in millimetri
#define LEN2 117
#define LEN3 60
#define MAXDIST LEN1+LEN2
#define pi 3.14159265
#define half_pi 1.5707963
#define IDLE_PIN 3
#define DIM_QUEUE 10
#define VEL_MAX -1
#define DIM_BUFFER_S 50
#define DIM_BUFFER_SX 12
#define DIM_BUFFER_SY 12
#define DIM_BUFFER_SZ 12
//correzioni

//correzioni sul piano (ossia considerando J0 in posizione. Sono dovute a offset sull' end eff. e sul punto d' origine)
#define PLANE_CORR_X 7 //ossia il punto è da considerare più lontano di tot
#define PLANE_CORR_Y -65 //ossia il punto è da considerare più in alto di tot. da integrare con LEN3
//definisco i software Limits
int annidate = 0;
float max_limit[3] = {300, 180, 300};
float min_limit[3] = {20, -180, 5};
//fine soft limits


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
char s[DIM_BUFFER_S] = "";
char sx[DIM_BUFFER_SX]="";
char sy[DIM_BUFFER_SY]="";
char sz[DIM_BUFFER_SZ]="";
char sf[4]="";
boolean coord_rel=0;
boolean coord_cilind=0;
int indx;
int indy;
int indz;
int indf;
boolean status_em = 0;
float feedvel = 80;
int i = 0, j = 0;
float t_in, t_fin, t_delta;
float posit_in[3] = {60, 0, 80}; //posit_in salva l' ultima posizione assunta
float posit_in_cilin[3];
float posit_fin[3];
float posit_fin_cilin[3];
float vel;
double angles[NUMJOINTS];
//float X_now, Y_now, Z_now;
float coord_now[3];
float coord_now_cilin[3];
int pinEM = 13;
//5th grade interpolation coefficients
float a[3], b[3], c[3], d[3], e[3], f[3];
int queue[DIM_QUEUE];
int head = -1;
int tail = -1;
int par_available = 0;
boolean glitch_flag = 1; //raspi manda uno zero di troppo all' inizio, quando arduino lo scarta abbasso il flag
int idle = 0;
float t_iter= 0.001;
//////////////////////INIZIO PROGRAMMA//////////////////////
void setup() {
  for (i = 0; i < NUMJOINTS; i++) {
    motor[i].attach(servopin[i]); //Vedi reference per i parametri che impostano valori min e max per la PWM
  }
  pinMode(IDLE_PIN, OUTPUT);
  digitalWrite(IDLE_PIN, LOW);
  Serial.begin(115200);
  pinMode(pinEM, OUTPUT);
  while (Serial.available())
    Serial.read();
  SetupServos(); //ricava la PWM (teorica) a 0 gradi e il rapporto variazionePWM/var. angolo per ogni motore
  ToCilindric(posit_in, posit_in_cilin);
  GetAngles(posit_in_cilin, angles); //li mette alla posizione iniziale
  for (int i=0; i<3; i++)
    coord_now[i]=posit_in[i];
  ToCilindric(coord_now, coord_now_cilin);
  SendPWM(angles);
  idle=1;
  //Serial.println("Grbl v1.09 ['$' for help]");
  WaitForRaspi(); //aspetta il segnale della raspberry pi (un bit qualsiasi)
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
      //delay(70);
      //while(Serial.available())
        //Serial.read();
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
  if (numel == DIM_QUEUE - 1){ /*4*/
    Serial.println("codapiena");
    return 1;
  }
  else
    return 0;
}
//fine implem. coda//
//funzione che stampa un array di parametri
void printParameters(int num, float par[], char name[], int spec) { 
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

int ii = 0;
boolean ReadSerial() { 
  //digitalWrite(CTS_PIN, HIGH); //indica che è pronto a ricevere
  //Serial.println("angolo");
  //Serial.println(angles[0]);
  if (Serial.available()){ 
    //delay(50);
    chr='E'; //qualsiasi cosa tanto per fare partire il ciclo while
    while(chr!='\n' && chr!='\0'){
      if(Serial.available()){
        chr = Serial.read();
        //Serial.print(chr);
        if(ii>DIM_BUFFER_S){
          Serial.println("s e' andato in overflow");
          break;
      }
      s[ii] = chr;
      ii++;
      }
    }
    if (chr == '\n' || chr=='\0'){
      //digitalWrite(CTS_PIN, LOW);
      boolean block = parse(); //parse torna 1 se il comando è tale da interrompere l' interpolazione e iniziarne una nuova
      chr=NULL;
      bufferClean();
      ii=0;
      return(block);
    }
  }
  return(0);
}
//non più utilizzata
boolean GetParameters() {
  if (par_available >= N_PARAMETERS) {
    for (int i = 0; i < N_PARAMETERS; i++) {
      param[i] = DeQueue();
      //Serial.print("in coda ho: ");
      ////Serial.println(param[i]);
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

float GetCoord (float dt, float init[], float fin[], float coordinates[], float* k) 
{
   //float k;
   //k=3*(dt*dt) - 2*(dt*dt*dt); //interpolazione a polinomio di 3 grado
   //k=dt; //interpolazione lineare
   *k=((1-(*k))/TAU)*t_iter + (*k);
   for (int i=0; i<3; i++)
    coordinates[i] = ((*k)*(fin[i] - init[i])) + init[i];
}
//opera la cinematica inversa PARTENDO DA COORDINATE CILINDRICHE
double GetAngles(float coor[], double angles[]) {
  float z=coor[2];
  float r;
  float L1 = LEN1;
  float L2 = LEN2;
//  if (!coord_cilind){
//    float x= coor[0];
//    float y=coor[1];
//    float x_square = x*x;
//    float y_square = y*y;
//    r = sqrt(x_square + y_square);
//    angles[0] = (atan2 (y, x)) + (half_pi);
//  }
//  else{
  angles[0] = ((coor[0] + 90)*pi/180); //deve variare tra 0 e 180
  r =  coor[1];
  //ora che sono sul piano applico le correzioni
  r = r+PLANE_CORR_X;
  z = z + PLANE_CORR_Y + LEN3; //ci ho messo y perchè nel piano la z dventa y
  angles[2] = pi - acos(((r * r) + z*z - L1*L1 - L2*L2) / (2 * L1 * L2)); //quando è 180 il braccio è dritto
  double C = cos(pi - angles[2]);
  double S = sin(pi - angles[2]);
  angles[1] = acos(((r * (L1 + L2 * C)) - (L2 * S * z)) / (((L2 * S)*(L2 * S)) + ((L1 + L2 * C)*(L1 + L2 * C))));
  angles[3] = (half_pi) - angles[1] + (pi-angles[2]); //quando è pi/2 il braccio è dritto
  for (i = 0; i < 4; i++) {
    angles[i] = ((angles[i] * 180) / pi) + corr_angle[i];
    if (angles[i] > maxangle[i]) {
      //Serial.println();
//      Serial.print("ERRORE: ANGOLO TROPPO GRANDE al motore ");
//      Serial.print(i);
//      Serial.print(" l' angolo e' ");
//      Serial.println(angles[i]);
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
//      Serial.println("ERRORE MANDANDO PWM: ");
//      Serial.print("motore ");
//      Serial.print(i);
//      Serial.print("  :");
//      Serial.println(ang[i]);
    }
  }
}

boolean TooDistant(float coord[]) {
  float newx=coord[0];
  float newy=coord[1];
  float newz=coord[2];
  float module;
  if (newz<0){
    Serial.println("z negativa");
    return(1);
  }
  newz = newz + PLANE_CORR_Y + LEN3; //serve perchè lo sto impostando per prendere le cose sempre dall' alto
  module = sqrt((newx*newx)+(newy*newy));
  module = sqrt(module*module + (newz*newz));
  if (module > (LEN1 + LEN2))
    return (1);
  return (0);
}
boolean softwareLimits(float *par){
  boolean reached=0;
  for(int i=0; i<3; i++){
    if(par[i]>max_limit[i]){
      par[i]=max_limit[i];
      reached=1;
    }
    if(par[i]<min_limit[i]){
      par[i]=min_limit[i];
      reached=1;
      //Serial.println("WARNING: soft limits reached");
    }
  }
  return(reached);
}
void ToCilindric(float car[], float cil[]){
  float x_square=car[0]*car[0];
  float y_square=car[1]*car[1];
  cil[1] = sqrt(x_square + y_square);
  cil[0] = (atan2(car[1], car[0]))*180/pi;
  cil[2] = car[2];
}
void ToCartesian(float cil[], float car[]){
  car[0]=cil[1] * cos((cil[0])*pi/180);
  car[1]=cil[1] * sin((cil[0])*pi/180);
  car[2]=cil[2];
}
int MakeMovement(float *par){
  if(!coord_cilind){
    for (i = 0; i < 3; i++)
      posit_fin[i] = par[i];
    ToCilindric(posit_fin, posit_fin_cilin);
  }
  else{
    for (i = 0; i < 3; i++)
        posit_fin_cilin[i] = par[i];
    ToCartesian(posit_fin_cilin, posit_fin);
  }
//  if (!TooDistant(posit_fin)) {    //non uso più la toodistant, bastano i softwarelimits
    if(1){
      float old_posit_in[3];
      float old_posit_in_cilin[3];
      digitalWrite(IDLE_PIN, 0);
      idle=0;
      if(softwareLimits(posit_fin))
        ToCilindric(posit_fin, posit_fin_cilin);
      //posit_fin[2]= posit_fin[2] + LEN 3; //visto che l' ultimo joint è fatto per essere perpendicolare al terreno, lavoro sulla z sommata la lunghezza del 3° pezzo
      vel = par[3]; //-1 vuol dire niente interpolazione
//      if (par[4])
//        digitalWrite(pinEM, HIGH);
//      else
//        digitalWrite(pinEM, LOW);
      //parameters are: [x_fin, y_fin, z_fin, vel, em_on?]
      if(vel>=0){
        //t_fin = GetTfin(posit_fin, posit_in, vel); //tempo finale in secondi
        t_fin = 4*TAU; //in interpolazione esponenziale uso il tempo di assestamento
        //Serial.println("tempo da impiegare: ");
        //Serial.println(t_fin);
        //GetInterp(posit_fin, posit_in, t_fin); //ricava i parametri per l' interpolazione a polinomio di grado 5
        t_in = sec();
        t_delta = (sec() - t_in);
        float k_dist_segmento = 0;
        while (t_delta < t_fin) {
          float t_start = sec();
          GetCoord((t_delta/t_fin), posit_in, posit_fin, coord_now, &k_dist_segmento);
          ToCilindric(coord_now, coord_now_cilin);
          //X_now = GetX(t_delta, a[0], b[0], c[0], d[0], e[0], f[0]); 
          //Y_now = GetX(t_delta, a[1], b[1], c[1], d[1], e[1], f[1]);
          //Z_now = GetX(t_delta, a[2], b[2], c[2], d[2], e[2], f[2]);
          if(!TooDistant(coord_now)){
            //Serial.println("voglio fare getangles");
            GetAngles(coord_now_cilin, angles);
            //Serial.println("fatta getangles");
          // float temppos[3] = {X_now, Y_now, Z_now};
          //printParameters(3, temppos, "Pos_asse", 7);
            SendPWM(angles);
          }
          t_iter = (sec()) - t_start;
          //quando non sono in coordinate assolute permetto di interrompere il movimento e iniziarne uno nuovo
            //questo perchè uso le coordinate assolute per raggiungere nodi di riferimento e voglio andare al nuovo nodo
            //solo dopo aver raggiunto quello prima
            //se sono in coord assolute registro la richiesta di movimento e la faccio dopo. problema: dovrei metterle in una coda o 
            //le nuove sovrascrivono le precedenti
          if(Serial.available()){ 
            for(int i=0; i<3; i++){
              old_posit_in[i]= posit_in[i];
              old_posit_in_cilin[i]=posit_in_cilin[i];
              posit_in[i]=coord_now[i];
              posit_in_cilin[i]=coord_now_cilin[i];
            }
            if (ReadSerial()){
                return(1);
//              MakeMovement(param); //param sono le variabili globali modificate da readserial, se ho qualche comando nuovo rifaccio tutto annidato
//              break;
            }
            else{
              for(int i=0; i<3; i++){
                posit_in[i]=old_posit_in[i];
                posit_in_cilin[i]=old_posit_in_cilin[i];
              }
            }
            digitalWrite(pinEM, status_em);
          }
          t_delta = ((sec()) - t_in);
        }
        GetAngles(posit_fin_cilin, angles);
        SendPWM(angles);
        for (i = 0; i < 3; i++){
          posit_in[i] = coord_now[i] = posit_fin[i];
          coord_now_cilin[i] = posit_fin_cilin[i];
        }
        digitalWrite(IDLE_PIN, 1);
        idle=1;
      }
      else{
        GetAngles(posit_fin_cilin, angles);
        SendPWM(angles);
        for (i = 0; i < 3; i++){
          posit_in[i] = coord_now[i] = posit_fin[i];
          coord_now_cilin[i] = posit_fin_cilin[i];
        }
      }
  }
  else {
    Serial.println("too distant");
  }
  digitalWrite(pinEM, status_em);
  return(0);
}
void loop() {
  boolean newpar = ReadSerial();
  digitalWrite(pinEM, status_em);
  //boolean newpar = GetParameters();
  if (newpar) {
    newpar=0;
    while(MakeMovement(param))
      ; //lascia questo ";"  perchè makemovement torna 1 finchè trova nuovi comandi, è un modo per evitare le chiamate annidate: troppe lo fanno impazzire
    annidate=0;
    Serial.print("tempo iterazione:");
    Serial.println(t_iter);
    //printParameters(4, angles, "angolo", 0);
    //newpar = 0;
    //Serial.println("movimento finito");
  }
  newpar=0;
}
