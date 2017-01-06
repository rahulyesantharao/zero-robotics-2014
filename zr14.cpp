#include "ZRGame.h"
#include "ZR_API.h"
#include "ZRUser.hpp"

#include 

static ZeroRoboticsGame &game = ZeroRoboticsGame::instance();
static ZeroRoboticsAPI &api = ZeroRoboticsAPI::instance();

//Implement your simulation code in init() and loop()
class ZRUser03 : public ZRUser
{

//Begin page main
//Begin page main
//Begin page 0_Vars
float myPos[3];
int timeToFlare;
float earth[3];
float zero[3];
int activePOI; // select a POI based on the sign of the y coord of the POI
float initY;
float currTarget[3];
float targ[3][3];
float poi[3][3];
float pic_rad;
float side_rad;
int timeElapsed;
int currSign;
bool on;
int numPics;
int tasd;
bool numFlares;

//////// MODIFICATIONS
// t1: time from start to outer POI
// t2: time from shadow to outer POI
// t3: time from shadow to central POI
// all scaled by .1 to account for .1rad/sec
/////////

//End page 0_Vars
//Begin page 1_Standard_Activity

//End page 1_Standard_Activity
//Begin page 2_Movement

// wrapper for movement
void moveToMdpt(float t[3], bool uploadFlag) {
    float Vtmp[3];
    float myD = dist(myPos, t);
    float dan_rad = 0.40f; // dist(myPos, zero) < dan_rad
    if(myD >= 0.10f) { // get a mdpt in path myD >= 0.10f
        float R = dist(t, zero); // radius of target
        if(R < dan_rad) R = dan_rad;
        float mdpt[3];
//        float normMyPos[3];
        for(int i=0; i<3; i++) {
            mdpt[i]=t[i];
            Vtmp[i]=myPos[i];
        }
        mathVecNormalize(Vtmp,3);
        mathVecNormalize(mdpt,3);
        float x = 2; // originally 2
        for(int i=0; i<3; i++) mdpt[i] = (mdpt[i]+Vtmp[i])/x;
        mathVecNormalize(mdpt,3);
        for(int i=0; i<3; i++) mdpt[i]*=R;
        
        moveToNaren(mdpt);
    }
    else {
        moveToNaren(t);
    }
    // attitude
    float ti[3];
    if(uploadFlag) {
        for(int i = 0; i < 3; i++ )
            ti[i] = earth[i];
        // mathVecSubtract(Vtmp, earth, myPos, 3);
    } else {
        ti[0] = ti[1] = ti[2] = 0.0f;
        // mathVecSubtract(Vtmp, zero, myPos, 3); // currPOI
    }
    mathVecSubtract(Vtmp, ti, myPos, 3);
    mathVecNormalize(Vtmp, 3);
    api.setAttitudeTarget(Vtmp);
    if(uploadFlag) {
        game.uploadPic();
    }
}

void move2(float t[3], bool uploadFlag) {
    moveToNaren(t);
    float Vtmp[3];
    if(uploadFlag) {
        mathVecSubtract(Vtmp, earth, myPos, 3);
    } else {
        mathVecSubtract(Vtmp, zero, myPos, 3); // currPOI
    }
    mathVecNormalize(Vtmp, 3);
    api.setAttitudeTarget(Vtmp);
    if(uploadFlag) {
        game.uploadPic();
    }
}

void moveToNaren(float t[3]) {
float Vtmp[3];
    float factor = 0.127f;
    if(dist(myPos,t) > 0.55f) {
        game.takePic(0);
        factor = 0.180f;
    }
    else if(dist(myPos,t) > 0.25f) {
        game.takePic(0);
        factor = 0.173f;
    }
    else if(dist(myPos,t) > 0.04f) {
        factor = 0.171f; // originally 0.150f then 0.165f then 0.170f
    }
    // else {
    //     factor = 0.060;
    // }
    mathVecSubtract(Vtmp, t, myPos, 3);
    for(int i = 0; i < 3; i++ ) {
        Vtmp[i]*=factor;
    }
    api.setVelocityTarget(Vtmp);
}
//End page 2_Movement
//Begin page 3_Funcs
float dist(float a[3], float b[3]) {
    return sqrtf((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1])+(a[2]-b[2])*(a[2]-b[2]));
}

float ang(float vec1[3], float vec2[3]) {
    float dot = 0;
    for(int i = 0; i < 3; i++ ) {
        dot += vec1[i]*vec2[i];
    }
    dot/=mathVecMagnitude(vec1, 3);
    dot/=mathVecMagnitude(vec2, 3);
    return acosf(dot);
}

//End page 3_Funcs
//Begin page main

void init(){
    
    tasd = 0;
    numFlares=false;
    currSign = 1;
    on = true;
    pic_rad = 0.45f; // originally 0.45f
    
    for(int i  = 0; i < 3; i++ ) {
        zero[i] = 0.0f;
        myPos[i] = 0.0f;
    }
    earth[0] = 0.64f;
    earth[1] = earth[2] = 0.0f;
    timeToFlare = -1;
    activePOI = 2;
    
    numPics = 0;
    //////////////INTELLIGENCE +0.75//////////////
    // t1 = 2.025f;
    // t2 = 3.395f;
    // t3 = 3.365f;
    // t1 = 1.975f;
    // t2 = 3.2345f;
    // t3 = 3.215f;
    //////////////////////////////////////////////
}

void loop(){
    float score = game.getScore();
    // obtaining data
    DEBUG(("[%d]: numPics %d; activePOI: %d; (%f, %f, %f)", timeElapsed, numPics, activePOI, poi[activePOI][0], poi[activePOI][1], poi[activePOI][2]));
    float myState[12];
    timeToFlare = game.getNextFlare();
    if(timeToFlare == -1) timeToFlare = 35;
    float Vtmp[3];
    api.getMyZRState(myState);
    
    for(int i = 0; i < 3; i++ ) {
        myPos[i] = myState[i];
    }
    
    for(int i = 0; i < 3; i++) {
        game.getPOILoc(targ[i], i);
        game.getPOILoc(poi[i], i);
    }
    timeElapsed = api.getTime();
    // end getting data
    if(!timeElapsed) { // initial setting of POI; this is our first target
        initY = myPos[1];
        activePOI = -1;
        do {
            activePOI++;
        } while(poi[activePOI][1]/myPos[1] <= 0.0f);
        // currSign
        // if(((poi[activePOI][2]*(-0.393f)+poi[activePOI][0]*0.919f) < 0.0f) || 
        //     ((poi[activePOI][0]*(-0.393f)-poi[activePOI][2]*0.919f)>0)) currSign = -1;
        // else currSign = 1;
        if(poi[activePOI][2]>0.0f) currSign = 1;
        else currSign = -1;
    } // end initial POI setting

    for(int i = 0; i < 3; i++ ) { // begin setting targ
        if(targ[i][1] != 0) { // one of the POIS on either end
            side_rad = sqrtf(0.04f - targ[i][1]*targ[i][1]);
            Vtmp[0] = 0.0f;
            Vtmp[1] = targ[i][1];
            Vtmp[2] = side_rad*currSign;
            mathVecNormalize(Vtmp, 3);
            for(int j = 0; j < 3; j++ ) {
                targ[i][j] = pic_rad * Vtmp[j];
            }
        } 
        else
        {
            targ[i][0] = targ[i][1] = 0.0f;
            targ[i][2] = pic_rad*currSign;
        }
    } // end setting targ
    if(!on) { // begin startup
        game.turnOn();
        on = true;
    } // end startup
    if(timeElapsed > 180) { // begin endgame
        game.takePic(activePOI);
        move2(zero, true);
    } // end endgame
    ///////////////////// WHAT IS THIS, I THINK IT'S UNNECESSARY+HARMFUL - RAHUL /////
    // if(poi[activePOI][2] > 0 && asd > 0) { // add one time condition
    //     currSign = -1;
    //     tasd++;
    // } else if(poi[activePOI][2] < 0 && asd > 0) {
    //     currSign = 1;
    //     tasd++;
    // }
    //////////////////////////////////////////////////////////////////////////////////
    if(/*asd>0 || */tasd>0) tasd++;
    if(tasd==5) { // begin setting active POI and currSign
        activePOI=-1;
        do {
            activePOI++;
        } while(targ[activePOI][1]!=0);
        if(poi[activePOI][1]==0) { // t3
            // if((((poi[activePOI][2]*(-0.997f)+poi[activePOI][0]*(-0.073f)) < 0.0f) || 
            // ((poi[activePOI][0]*(-0.997f)-poi[activePOI][2]*(-0.073f))>0))) currSign=-1;
            // else currSign=1;
            if(poi[activePOI][2] > 0.0f) currSign = 1;
            else currSign = -1;
        }
        // else { // t2
        //     if((((poi[activePOI][2]*cosf(t2)+poi[activePOI][0]*sinf(t2)) < 0.0f) || 
        //     ((poi[activePOI][0]*cosf(t2)-poi[activePOI][2]*sinf(t2))>0))) currSign=-1;
        //     else currSign=1;
        // }
        tasd=0;
        DEBUG(("[%d]*****TASD SELECTION******", timeElapsed));
    } // end setting activePOI and currSign
    if(game.getFuelRemaining() == 0) {
        game.takePic(activePOI);
        if(timeToFlare == 1) {
            game.turnOff();
        }
    }
    bool insideShadow = ((myPos[0] < 0.62f) && (myPos[0] > 0.0f) && 
                         (myPos[1] < 0.18f) && (myPos[1] > -0.18f) &&
                         (myPos[2] < 0.18f) && (myPos[2] > -0.18f));

    if(insideShadow) game.takePic(activePOI);
    if(timeToFlare < 23 && (((score>19 || (score>16 && game.getMemoryFilled()>0)) && numFlares)||!numFlares)) { // begin flare response
        // traveling to shadow zone
        Vtmp[0] = 0.33f;
        Vtmp[1] = sqrtf(0.02f);
        if(myPos[1] < 0) Vtmp[1]*=-1;
        Vtmp[2] = sqrtf(0.02f)*currSign;
        game.takePic(0); // ffpicture
        if(myPos[0] < 0) {
            moveToMdpt(Vtmp, true);
        }
        else {
            move2(Vtmp, true);
        }
        
        // moveToMdpt(s, UL);
        if(timeToFlare < 2 && !insideShadow) {
            game.turnOff();
            on = false;
        }
        if(timeToFlare == 1) {
            tasd++;
            numFlares=true;
        }
        return;
    } // end flare response
    // if(timeToFlare < 2 && !insideShadow) {
    //         game.turnOff();
    //         on = false;
    // }
    // if(asd > 0) {
    //     asd = 0;
    // }
    if(myPos[0] < 0) { // begin movement commands
        move2(targ[activePOI], false);
    } else {
        moveToMdpt(targ[activePOI], false);
    } // end movement commands
    bool pic = false;
//    float v[3];
    mathVecSubtract(Vtmp, myPos, poi[activePOI], 3);
    //DEBUG(("\n***%03d: current angle: %f   myPos: %f %f %f POI %d: %f %f %f v: %f %f %f\n", timeElapsed, ang(poi[activePOI], Vtmp),
  //  myPos[0], myPos[1], myPos[2],
  //  activePOI, poi[activePOI][0], poi[activePOI][1], poi[activePOI][2],
  //  Vtmp[0], Vtmp[1], Vtmp[2]
  //  ));
    if(ang(poi[activePOI], Vtmp) < 0.40f && game.alignLine(activePOI)) { // begin picture taking
        if(currSign==-1) DEBUG(("||TAKING PIC||"));
        int mi = game.getMemoryFilled();
        game.takePic(activePOI);
        int mf = game.getMemoryFilled();
        pic = (mf > mi);
    } // end picture taking
    if(timeElapsed%60==0) { // begin reset numPics
        numPics=0;
    } // end reset numPics

    if(pic) { // begin setting POI after picture taken
        numPics++;
        
        DEBUG(("\n**;*%03d: PIC TAKEN...\n", timeElapsed));        
        
        
        if(poi[activePOI][1] == 0.0f || (numPics==2 && timeElapsed>60)) {
            activePOI = -1;
            do {
                activePOI++;
            } while(poi[activePOI][1]/initY<=0.0f);
        } else {
            activePOI = -1;
            do {
                activePOI++;
            } while(poi[activePOI][1]!=0.0f);
        }
        if(timeElapsed>120) {
            if(numPics==2) {
                activePOI=-1;
                DEBUG(("***4***"));
                do {
                    activePOI++;
                } while(poi[activePOI][1]/initY>= 0.0f);
            }
        }
    } // end setting POI after picture taken

    if((game.getMemoryFilled() == game.getMemorySize()) ||
    (game.getMemoryFilled() > 0 && game.getFuelRemaining() < 20) ||
    (game.getMemoryFilled() > 0 && timeToFlare < 10)) { // begin upload condition
        for(int i = 0; i < 3; i++ ) {
            Vtmp[i] = myPos[i];
        } mathVecNormalize(Vtmp, 3);
        for(int i = 0; i < 3; i++ ) {
            Vtmp[i]*=0.53f;
        }
        moveToMdpt(Vtmp, true);
        if(game.getMemoryFilled() == 0) {
            moveToMdpt(targ[activePOI], false);
            // numPics=0;
        }
    } // end upload condition
  DEBUG(("next flare: %d", timeToFlare));
}

//End page main

//End page main
//End page main


};

ZRUser *zruser786 = new ZRUser03;
