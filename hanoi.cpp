/*follower centrato sulla camera*/
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h> //controllare
#include <fstream>
#include <queue>
#include <stack>

#include <wiringPi.h>
#include <wiringSerial.h>
#define START 0
#define END 1
#define AUX 2
#define PICK 1
#define RELEASE 0
#define FPS 13
#define PRECISION 3 //distanza a cui si stabilizza prima di effettuare 
il pick
#define PICKHEIGHT 5
#define FRAMEONOBJECT 5 //devo stare 5 frame sull' oggetto con una precisione PRECISION  prima di prenderlo
#define FOLLOWVEL 30 //troppo veloce -> immagini sfocate
#define ANGLE_THRESHOLD 3
#define DIST_THRESHOLD 10
#define Z_THRESHOLD 5
#define ANGLE_REDUCER 100 //lunghezza media braccio
#define pi 3.14159265
#define SCAN 0
#define FOLLOW 1
#define TRYFOLLOW 2
#define TRYFOLLOW_MULTIPLIER 0.5 //di quanto moltiplica l' ultimo errore conosciuto quando perde l' oggetto
#define MAX_ATTEMPTS 5 //tentativi di ricerca prima di andare in scan
#define Z_ONOBJECT 120 //altezza a cui si ferma prima di raccogliere
#define N_SCAN_NODES 6 //numero di punti raggiunti durante la scansione
#define IDLE_PIN 12
using namespace std;
using namespace cv;
int okframe=0;
int scan_iteration=0;
int attempt = 0;
float offset[3] = {-8, -37, 9}; //offset della camera in coordinate 
della camera
float cam_rotation = (((float)-90-0.6)*pi/180); //rotazione robot -> 
camera
float scan_nodes[N_SCAN_NODES][3] = {
    {60, 0, 180},
    {60, 70, 180},
    {130, 70, 180},
    {130, 0, 180},
    {130, -70, 180},
    {60, -70, 180}
};
float initialPosition[3]= {100, 0, 150};
int status=SCAN;
VideoCapture inputVideo;
std::queue<int> moveStartQueue;
std::queue<int> moveEndQueue;
std::stack<int> startStack;
std::stack<int> endStack;
std::stack<int> auxStack;
namespace {
const char* about = "Basic marker detection";
const char* keys  =
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, 
DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, 
DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, 
DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, 
DICT_ARUCO_ORIGINAL = 16}"
        "{v        |       | Input from video file, if ommited, input 
comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video 
(-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for 
camera pose }"
        "{l        | 0.1   | Marker side lenght (in meters). Needed for 
correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{s        |       | show image stream w/ axis }"
        "{r        |       | show rejected candidates too }"
        "{kprop    | 0.2   | set proportional control constant}"
        "{kint     | 0.1   | set integral control constant}"
        "{kder     | 0     | set derivative control constant}";
}


static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["doCornerRefinement"] >> params->doCornerRefinement;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

int controlFunction(float x, float y, float z, float *vout, float ki, float kp, float kd, float lastin[3][2], float lastout[], float Tcamp){
    float vin[3];
    vin[0]=x;
    vin[1]=y;
    vin[2]=z;
    for (int i=0; i<3; i++){
        //vout[i]=kp * vin[i];
        float P = kp;
        float I = ki*Tcamp;
        float D = kd/Tcamp;
        vout[i] = lastout[i] + (P * (vin[i] - lastin[i][0])) + I * vin[i] + D * (vin[i] - (2 * lastin[i][0]) + lastin[i][1]);
        lastout[i] = vout[i];
        lastin[i][1] = lastin[i][0];
        lastin[i][0] = vin[i];
        //P=k I=ki*T D=kd/T
        //Yn=Yn_1 + P (Xn-Xn_1) + I Xn + D(Xn - 2*Xn_1+Xn_2)
    }
    return 0;
}

int min(int val1, int val2){
    if (val1<val2)
        return val1;
    else
        return val2;
}

int gcodeSendFast(float *tosend, int serial){
       //char str[] <<"g0x"<<tosend[0]<<"y"<<tosend[1]<<"z"<<tosend[2]<<"\n";
    //printf("tosend : %.2f  -  %.2f  -  %.2f\n", tosend[0], tosend[1], tosend[2]);
    serialPrintf(serial,"g0x%.2fy%.1fz%.1f\n", tosend[0], tosend[1], tosend[2]);
    //serialPutchar(serial, '\0');
    return 0;
}

void robotCoord(float* vect_before, float* vect_after, float ang, float* offs, float rot){
//    printf("robotcoord\n");
    float xcam = vect_before[0];
	float ycam = vect_before[1];
	/* EDIT: HO COMMENTATO LA CORREZIONE DEGLI OFFSET, 
	ORA LA FACCIO IN FASE DI PICK*/
	//traslation:
	//xcam = xcam - offs[0];
	//ycam = ycam - offs[1];
	float camrot= ang + rot;
	//rotation of -camrot
    vect_after[0]=xcam*cos(camrot)-ycam*sin(camrot);
    vect_after[1]=xcam*sin(camrot)+ycam*cos(camrot);
    vect_after[2]=(-vect_before[2]);//+offs[2];
}

int gcodeSendControlled(float *tosend, int serial, int vel){
    //char str[] <<"g0x"<<tosend[0]<<"y"<<tosend[1]<<"z"<<tosend[2]<<"\n";
    //printf("tosend : %.2f  -  %.2f  -  %.2f\n", tosend[0], tosend[1], tosend[2]);
    serialPrintf(serial,"g1x%.2fy%.1fz%.1ff%d\n\0", tosend[0], tosend[1], tosend[2], vel);
    return 0;
}
void setAbsCoord(int serial){
    serialPrintf(serial, "g90\n\0");    
}
void setRelCoord(int serial){
    serialPrintf(serial, "g91\n\0");    
}
void setCartesianCoord(int serial){
    serialPrintf(serial, "g15\n\0");    
}
void setCilindricCoord(int serial){
    serialPrintf(serial, "g16\n\0");    
}
void setEMOn(int serial){
    serialPrintf(serial, "m03\n\0");
}
void setEMOff(int serial){
    serialPrintf(serial, "m30\n\0");
}

void calculateOffset(float *d, float *beta, float *z){
    //printf("calculate offset\n");
    *d=sqrt((offset[0]*offset[0])+(offset[1]*offset[1]));
    *beta=atan2(offset[1],offset[0]);
    *z=offset[2]; //l' ho messo ma ancora non lo gestisco
}

int stringFromSerial(char* received, int  serial){
    int finito;
    finito=0;
    char income=' ';
    int i=0;
    int started=0;
//    printf("aspetto\n");
    while(finito==0){
//	printf("e ci sono rientrato!\n");
        if(serialDataAvail(serial)){
//	    printf("roba in arrivo:\n");
            income=serialGetchar (serial);
            if (!started && (income!='\n' || income!='\r' || income!='\0')) //flush dei caratteri di termine rimasti inizio stringa
                started=1;
            if(started){
                received[i] = income;
//	            printf("%c \n", income);
                if (income=='\n' || income =='\0' || income == '\r'){
                    received[i]='\0';
                    finito=1;
//                  printf("finito di ricevere \n");
		            break;
                }
            i++;
            i=i%15;
	        }
        }
//	printf("ancora sono in serialstringecc !fin = %d\n",!finito);
    }
//  printf("i: %d", i);
    return 0;
}

/*******ROBOT FUNCTIONS***************/
void askForIdle(int serial){
//	printf("askforidle\n");
	serialFlush(serial);
	serialPrintf(serial, "I\n\0");
}
int robotIsIdle(int serial){
    //int idle = digitalRead(IDLE_PIN);
    //printf("idle : %d", idle);
	char buf[10];
	askForIdle(serial);
	while(!serialDataAvail(serial));
	int rec = serialGetchar(serial);
//  printf("ricevuto\n");
	int idle = rec-'0';
//	printf("idle %d", idle);
    return(idle);
}
int askForAngle(int serial){
//	printf("askforangle \n");
    serialPrintf(serial, "A\n\0");
}
float getRobotAngle(int serial){
    char buf[15];
    float ang;
    askForAngle(serial);
    stringFromSerial(buf, serial);
//    printf("non mi sono impiccato dopo lo stringfromserial \n");
    ang=atof(buf);
//    printf("e nemmeno dopo atof %f \n", ang);
    return (ang);
}
void waitForIdle(int serial){
    delay(40);
    while(!robotIsIdle(serial));
}
void initializeRobot(int serial){
    //printf("initialize robot \n");
    //serialPrintf(serial, "start \n\0");
    setCartesianCoord(serial);
    setAbsCoord(serial);
    gcodeSendControlled(initialPosition, serial, 100);
    //setRelCoord(serial);
    //setCilindricCoord(serial);
    waitForIdle(serial);
    setEMOff(serial);
    status=SCAN;
    printf("end initialization\n:");
}
void robotPickRelease(int serial, float dist[], int action){
   // printf("pick\n");
    float traslation1[3];
    float traslation2[3];
    float negtraslation2[3];
    traslation1[0]=dist[0];
    traslation1[1]=dist[1];
    traslation1[2]=0; //dist[2]+PICKHEIGHT;
    setRelCoord(serial);
    gcodeSendControlled(traslation1, serial, 60);
    waitForIdle(serial);
    traslation2[0]=0;
    traslation2[1]=0;
    traslation2[2]=dist[2] + (PICKHEIGHT*(action==RELEASE));
    gcodeSendControlled(traslation2,serial,60);
    waitForIdle(serial);
	if(action == PICK)
		setEMOn(serial);
	else
		setEMOff(serial);
    delay(500);
    for(int i=0; i<3; i++)
        negtraslation2[i]= (-1)*(traslation1[i] + traslation2[i]);
    gcodeSendControlled(negtraslation2, serial, 80);
    setAbsCoord(serial);
    waitForIdle(serial);
}

//robotFollow handles the control function and the serial communication of the position
//this function returns 1 if I finished a PICK - RELEASE movement, otherwise returns 0
int robotFollow(int serial, float errvec[], float contrvec[], float kint, float kprop, 
    float kder, float lastin[3][2], float lastout[], float Tcamp, int pick, float angle_now){
	float dist_err = sqrt((errvec[0] * errvec[0]) + (errvec[1] * errvec[1]) + ((errvec[2]+Z_ONOBJECT) * (errvec[2]+Z_ONOBJECT)));
	if(dist_err < PRECISION){
		okframe++;
		if(okframe == FRAMEONOBJECT){
			float offst_rob[3];
			robotCoord(offset, offst_rob, angle_now, offset, cam_rotation);
			for (int i=0; i<3 ; i++){
				errvec[i]-=offst_rob[i];
			}
			robotPickRelease(serial, errvec, pick);
			okframe=0;
			return(1);
		}
	}
	else{
		okframe=0;
	}
	//printf("okframe %d\n",okframe);
    controlFunction(errvec[0],errvec[1], errvec[2] + Z_ONOBJECT,contrvec, kint, kprop, kder, lastin, lastout, Tcamp); 
//implementa il controllo, in=errorvec out=controlvec
    //printf("controlvec %f %f %f\n", contrvec[0], contrvec[1], contrvec[2]);
    gcodeSendControlled(contrvec, serial, FOLLOWVEL);
    //waitForIdle(serial);
    return(0);
}
void robotScan(int serial){
    if(robotIsIdle(serial)){
    	//printf("Scan iteration %d \n", scan_iteration%N_SCAN_NODES);
    	setAbsCoord(serial);
    	setCartesianCoord(serial);
    	gcodeSendControlled(scan_nodes[scan_iteration%N_SCAN_NODES], serial, 50);
    	scan_iteration++;
    	//setRelCoord(serial);
    	//setCilindricCoord(serial);
    }
}
void robotTryFollow(int serial, float lastknownerr[], float kp){
    //printf("TryFollow attempt %d\n", attempt);
    float tryVec[3];
	//control system is bypassed, 
	//so error signal must be given to robot with
	//respect to the current position
	setRelCoord(serial);
    tryVec[0] = lastknownerr[0] * TRYFOLLOW_MULTIPLIER;
    tryVec[1] = lastknownerr[1] * TRYFOLLOW_MULTIPLIER;
    tryVec[2] = 0; //aumento sempre la z
	gcodeSendControlled(tryVec, serial, FOLLOWVEL);
	setAbsCoord(serial);
    attempt++;
}


/******END ROBOT FUNCTIONS***************/
void fileWrite(double t_grab, double t_it, int frames, ofstream &log){
	double t_tot = t_grab + t_it;
    log << t_grab << ";"<<t_it<<";"<<t_tot<<";"<<frames<<"\n";
}



/**
 */

 int analyzeFrame(int fd, Ptr<aruco::Dictionary> &dictionary, const Ptr<aruco::DetectorParameters> &detectorParams, 
					float markerLength, Mat camMatrix, Mat distCoeffs, int* foundmarker, float errvec[], int 
                    showStream, vector<int>& ids, int searchedID, float *angle_now){
	inputVideo.grab();
	serialFlush(fd); //occhio a non metterla subito dopo un serial send o si mangia byte in uscita
	Mat image, imageCopy;
	*foundmarker=0;
	int indexID = -1;
	*angle_now = getRobotAngle(fd);
	*angle_now=(*angle_now-90)*pi/180;
	inputVideo.retrieve(image);
	vector< vector< Point2f > > corners, rejected;
	vector< Vec3d > rvecs, tvecs;
	aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
	if(ids.size() > 0){
		aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,tvecs);
//		printf("ho fatto estimatepose, ids size = %d\n", 
        ids.size();
    	for (int i=0; i<ids.size(); i++){
    		if (ids[i]==searchedID){
    			*foundmarker=1;
    			indexID=i;
    		}
    	}
    }
//	 for(int i=0; i<tvecs.size(); i++){
//	     	cout << "Traslation vector"<<i<<endl
//	     	<< "size = "<<tvecs.size()<<endl
//             	<< "x = "<<tvecs[i][0] <<endl
//             	<< "y = "<<tvecs[i][1]<<endl
//            	<< "z = "<<tvecs[i][2]<<endl ;
//		 }
	if(*foundmarker){
		float camera_err [3] = { tvecs[indexID][0], 
            tvecs[indexID][1], tvecs[indexID][2] };
		robotCoord(camera_err, errvec, *angle_now, offset, cam_rotation);
		//printf("err rob coor : %f %f %f\n", errvec[0], errvec[1], errvec[2]);
	}
	// draw results
	if(showStream)
		image.copyTo(imageCopy);
	if(ids.size() > 0 && showStream) {
		aruco::drawDetectedMarkers(imageCopy, corners, ids);
		for(unsigned int i = 0; i < ids.size(); i++) {
			aruco::drawAxis(imageCopy, camMatrix, 
                distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
        }
	}
	char key = 0;
	if(showStream){
		imshow("out", imageCopy);
		key= (char)waitKey(1);
	}
}
int robotControl(int serial, int inview, float errvec[], float 
contrvec[], float ki, float kp, float kd, float lastin[3][2], float 
lastout[], float Tcamp, int pick, float angle){
    //FINITE STATE MACHINE DEFINITION
    switch (status){
        case SCAN:
            if(inview){
                status = FOLLOW;
                robotFollow(serial, errvec, contrvec, ki, kp, kd, 
                    lastin, lastout, Tcamp, pick, angle);
				scan_iteration = 0;
            }
            else
                robotScan(serial);
            break;
        case FOLLOW:
            if(!inview){
                status=TRYFOLLOW;
                robotTryFollow(serial, errvec, kp);
                break;
            }
            else
                //if I see something I have to follow...
                if(robotFollow(serial, errvec, contrvec, ki, kp, kd, lastin, lastout, Tcamp, pick, angle)){
					//printf("Pick - release movement completed\n");
					for(int i=0; i<3; i++){
						lastin[i][0] = 0;
						lastin[i][1]=0;
						lastout[i]=0;
					}
					status=SCAN;
					return(1);
                }
		else
            break;
        case TRYFOLLOW:
            if(inview){
                status = FOLLOW;
                robotFollow(serial, errvec, contrvec, ki, kp, kd, lastin, lastout, Tcamp, pick, angle);
				attempt = 0;
            }
            else{
                if(attempt>MAX_ATTEMPTS){
                    status=SCAN;
                    robotScan(serial);
		    attempt=0;
                }
                else
                    robotTryFollow(serial, errvec, kp);
            }
            break;
    }
	return (0);
}
void updateIds(vector<int>& found, vector<int>& saved){
	int alreadyfound=0;
	int n=saved.size();
	int m=found.size();
	printf("savedsize %d foundsize %d\n",n ,m);
	//se trova un nuovo id lo mette in saved_ids
		for(int k=0; k<m; k++){
			for(int i=0; i<n; i++){
				if (found[k]==saved[i]){
					alreadyfound=1;
				}
			}
			if(!alreadyfound){
				saved.push_back(found[k]);
				printf("salvo id %d",found[k]);
			}
			alreadyfound=0;
		}
}
int hanoiPreparation(int serial, vector<int>& ids, vector<int>& 
saved_ids){
	robotScan(serial);
//	printf("sono in hanoipreparation\n");
	if(ids.size())
		updateIds(ids, saved_ids);
	if (saved_ids.size()==3){
		return (1);
	}
	else	
		return (0);
}
void hanoiSolution(int num, char frompeg, char topeg, char auxpeg)
{
    if (num == 1)
    {
		moveStartQueue.push(frompeg);
		moveEndQueue.push(topeg);
        return;
    }
    hanoiSolution(num - 1, frompeg, auxpeg, topeg);
	moveStartQueue.push(frompeg);
	moveEndQueue.push(topeg);
    hanoiSolution(num - 1, auxpeg, topeg, frompeg);
}
void createIdsStack(vector<int>& saved_ids, int* num_discs){
	int first=1;
	int baseids_sum=0;
	int withtower;
	for(int i=0; i<3; i++){
		if (saved_ids[i]>10){
			*num_discs=saved_ids[i]-9;
			withtower=i;
		}
		if(saved_ids[i]<10){
			baseids_sum+=saved_ids[i];
			if(first){
				endStack.push(saved_ids[i]);
				printf("endStack push %d\n", saved_ids[i]);
				first=0;
			}
			else{
				auxStack.push(saved_ids[i]);
				printf("auxStack push %d\n", saved_ids[i]);
			}
		}
	}
	startStack.push(3-baseids_sum);
	printf("startStack push %d\n", 3-baseids_sum);
	int pushed=10;
	while(pushed<=saved_ids[withtower]){
		startStack.push(pushed);
		printf("startStack push %d\n", pushed);
		pushed++;
	}
}

void bufferClean(double currentTime_tot){
	int framesinbuf = min(floor(currentTime_tot*FPS), 5);
	if(framesinbuf){
		for(int i=0; i<framesinbuf ; i++){
			inputVideo.grab();
			//printf("frame scartato %d\n",i);
		}
	}
}

int main(int argc, char *argv[]) {
    int fd;
    float angle=0;
    if (!(fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
        cerr << "impossibile aprire la seriale";
    if (wiringPiSetupPhys() == -1)
        cerr << "wiringPi dà problemi";
    pinMode(IDLE_PIN, INPUT);
    float errvec[3] = {0, 0, 0};
    float contrvec[3] = {0, 0, 0};
    int foundmarker=0;
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);
    if(argc < 2) {
        parser.printMessage();
        return 0;
    }
    int dictionaryId = parser.get<int>("d");
    bool showStream = parser.has("s");
    bool showRejected = parser.has("r");
    //bool estimatePose = parser.has("c");
    //voglio sempre stimare la posa
    bool estimatePose = 1;
    float markerLength = parser.get<float>("l");
    float kprop = parser.get<float>("kprop");
    float kint = parser.get<float>("kint");
    float kder = parser.get<float>("kder");
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }
    detectorParams->doCornerRefinement = true; // do corner refinement in markers
    int camId = parser.get<int>("ci");
    String video;
    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    Ptr<aruco::Dictionary> dictionary =
        
aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    Mat camMatrix, distCoeffs;
    if(estimatePose) {
        bool readOk = readCameraParameters(parser.get<string>("c"), 
camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = 1;
    }
    inputVideo.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    inputVideo.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    //inputVideo.set(CV_CAP_PROP_BUFFERSIZE, 0);
    inputVideo.set(CAP_PROP_FPS, FPS);
    double totalTime = 0;
    double totalTime_tot = 0;
    int totalIterations = 0;
    float dist_offset, angle_offset, z_offset;
    calculateOffset(&dist_offset, &angle_offset, &z_offset);
    printf("offsets: dist = %f, angle = %f, z= %f\n", dist_offset, angle_offset, z_offset);
    initializeRobot(fd);
    float lastin[3][2] = { (0,0) , (0,0), (0,0)};
    float lastout[3] = {0, 0, 0};
    for(int i=0; i<5 ; i++){
        inputVideo.grab();
    }
    double tick_grabbing = (double)getTickCount();
    int framesinbuf = 0;
	vector <int> saved_ids;
	vector< int > ids;
	float angle_now;
	do{
//		printf("in preparation\n");
		double tick_tot = (double)getTickCount();
		analyzeFrame(fd, dictionary, detectorParams, 
            markerLength, camMatrix, distCoeffs, &foundmarker, errvec, showStream, 
            ids, 0, &angle_now);
		double currentTime_tot = ((double)getTickCount() - 
            tick_tot) / getTickFrequency();
		bufferClean(currentTime_tot);
//		printf("hio finito analyzeframe\n");
	}while(!hanoiPreparation(fd, ids, saved_ids));
	printf("preparation done\n");
	serialFlush(fd); //occhio a non metterla subito dopo un serial send o si mangia byte in uscita
	int num_discs=0;
	createIdsStack(saved_ids, &num_discs);
	printf("num discs= %d",num_discs);
	printf("gli ID salvati sono %d, %d, %d \n", saved_ids[0], saved_ids[1], saved_ids[2]);
	hanoiSolution(num_discs, START, END, AUX);
	while(!moveStartQueue.empty()){
		int currentStartMove = moveStartQueue.front();
		int currentEndMove = moveEndQueue.front();
		printf("from pile %d to pile %d", currentStartMove, currentEndMove);
		moveStartQueue.pop();
		moveEndQueue.pop();
		int startID;
		int endID;
		switch (currentStartMove){
			case START:
				startID = startStack.top();
				startStack.pop();
				break;
			case END:
				startID = endStack.top();
				endStack.pop();
				break;
			case AUX:
				startID = auxStack.top();
				auxStack.pop();
				break;
		}
		switch (currentEndMove){
			case START:
				endID = startStack.top();
				startStack.push(startID);
				break;
			case END:
				endID = endStack.top();
				endStack.push(startID);
				break;
			case AUX:
				endID = auxStack.top();
				auxStack.push(startID);
				break;
		}
		printf("startID: %d, endID: %d \n", startID, endID);
		int picked = 0;
		int released = 0;
		int notReallyPicked;
		int pickAttempts=0;
		float offst_rob[3];
		printf("look for %d/n", startID);
		do{
			double tick_tot = (double)getTickCount();
			analyzeFrame(fd, dictionary, detectorParams, markerLength, camMatrix, distCoeffs, 
                &foundmarker, errvec, showStream, ids, startID, &angle_now);
			double currentTime = ((double)getTickCount() - tick_tot) / getTickFrequency(); //da posizionare meglio
			picked = robotControl(fd, foundmarker, errvec, 
                contrvec, kint, kprop, kder, lastin, lastout, currentTime, PICK, angle_now);
			foundmarker=0;
			double currentTime_tot = ((double)getTickCount() - tick_tot) / getTickFrequency();
			bufferClean(currentTime_tot);
			if(picked){
				do{
					analyzeFrame(fd, dictionary, 
                        detectorParams, markerLength, camMatrix, distCoeffs, &notReallyPicked, 
                        errvec, showStream, ids, startID, &angle_now); //se lo vedo ancora non l' ho preso
					printf("not really picked = %d",notReallyPicked);
					if(notReallyPicked){
						pickAttempts++;
						robotCoord(offset, offst_rob, angle_now, offset, cam_rotation);
						for (int i=0; i<3 ; i++){
                            errvec[i]-=offst_rob[i];
//printf("traslazione %d = %f\n", i, offst_rob[i]);
						}
                    errvec[2]-=(5*pickAttempts);
					robotPickRelease(fd, errvec, PICK);
					}
					bufferClean(1000);
				}while(notReallyPicked);
				pickAttempts=0;
			}
		}while(!picked);
		printf("Looking for %d ... \n", endID);
		do{
			double tick_tot = (double)getTickCount();
			analyzeFrame(fd, dictionary, detectorParams, 
                markerLength, camMatrix, distCoeffs, &foundmarker, errvec, showStream, 
                ids, endID, &angle_now);
			double currentTime = ((double)getTickCount() - tick_tot) / getTickFrequency(); //da posizionare meglio
			released = robotControl(fd, foundmarker, errvec, 
                contrvec, kint, kprop, kder, lastin, lastout, currentTime, RELEASE, angle_now);
			foundmarker=0;
			double currentTime_tot = ((double)getTickCount() - tick_tot) / getTickFrequency();
			framesinbuf = min(floor(currentTime_tot*FPS), 5);
			if(framesinbuf){
				for(int i=0; i<framesinbuf ; i++){
					inputVideo.grab();
					//printf("frame scartato %d\n",i);
				}
			}
		}while(!released);
		printf("Move completed \n");
	}
    printf("Exit from while...\n");
    return 0;
}

