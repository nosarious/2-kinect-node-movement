#include "ofApp.h"

/*
    If you are struggling to get the device to connect ( especially Windows Users )
    please look at the ReadMe: in addons/ofxKinect/README.md
*/

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
    
    gui.setup(); // most of the time you don't need a name
    gui.add(bTopVP.setup("top view", false));
    gui.add(bSideVP.setup("side view", false));
    gui.add(b3DVP.setup("3D view", false));
    
    gui.add(bUseColor.setup("use kinect color", false));
    
    gui.add(k2Dolly.setup("kinect2 dolly (in/out)", 0.0,-2.0,2.0));
    gui.add(k2Truck.setup("kinect2 truck (side-to-side)", 0.0,-2.0,2.0));
    gui.add(k2Boom.setup("kinect2 boom (up-down)", 0.0,-2.0,2.0));
    gui.add(k2Rotate.setup("kinect2 rotate", 0.0,-1.0,1.0));
    gui.add(k2Tilt.setup("kinect2 tilt", 0.0,-1.0,1.0));
    
    gui.add(k2FreezeMotion.setup("freeze Motion", false));
    gui.add(k2RotatePoint.setup("kinect2 rotate point", 0,0,5000));
    
}

//--------------------------------------------------------------
void ofApp::update() {
	
    if (bTopVP){
        bSideVP=false;
        b3DVP = false;
    }
    if (bSideVP){
        bTopVP=false;
        b3DVP = false;
    }
    
    if(k2FreezeMotion){
        k2Dolly = 0.0;
        k2Truck = 0.0;
        k2Boom = 0.0;
        k2Rotate = 0.0;
        k2Tilt = 0.0;
        k2FreezeMotion = false;
    }
    
    
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void ofApp::draw() {
	
	ofSetColor(255, 255, 255);
	
    easyCam.begin();
        if (bTopVP){
            //easyCam.enableOrtho();
            ofRotateX(90);
        }
        if (bSideVP){
            //easyCam.enableOrtho();
            ofRotateY(90);
        }
        ofSetColor(100,200,100);
        drawThePointCloud1();
        #ifdef USE_TWO_KINECTS
            kinect2cam.dolly(k2Dolly);
            kinect2cam.truck(k2Truck);
            kinect2cam.boom(k2Boom);
            kinect2cam.rotate(k2Rotate,ofPoint(0,k2RotatePoint,0));
            kinect2cam.tilt(k2Tilt);
            kinect2cam.transformGL();
        
            drawThePointCloud2(); // this is now done in local space relative to the node
            kinect2cam.restoreTransformGL();
        #endif
    easyCam.end();
	

	
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
        
    if(kinect.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
		<< "motor / led / accel controls are not currently supported" << endl << endl;
    }
    
	reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl;

    if(kinect.hasCamTiltControl()) {
    	reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
    
	ofDrawBitmapString(reportStream.str(), 20, 652);
    gui.draw();
}

void ofApp::drawThePointCloud1() {
    int w = 640;
    int h = 480;
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    int step = 2;
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            if(kinect.getDistanceAt(x, y) > 0) {
                if (bUseColor){
                    mesh.addColor(ofColor(150,150,150));
                }else {
                    mesh.addColor(kinect.getColorAt(x,y));
                }
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
            }
        }
    }
    glPointSize(3);
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000); // center the points a bit
    
    ofLine(ofPoint(0,0,0), ofPoint(0,0,5000));
    ofDrawBox(0,0,0,100,20,20);
    ofEnableDepthTest();
    mesh.drawVertices();
    ofDisableDepthTest();
    ofPopMatrix();
}

void ofApp::drawThePointCloud2() {
    int w = 640;
    int h = 480;
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    int step = 2;
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            if(kinect2.getDistanceAt(x, y) > 0) {
                if (bUseColor){
                    mesh.addColor(ofColor(200,200,50));
                }else {
                    mesh.addColor(kinect2.getColorAt(x,y));
                }
                
                mesh.addVertex(kinect2.getWorldCoordinateAt(x, y));
            }
        }
    }
    glPointSize(3);
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000); // center the points a bit
    ofEnableDepthTest();
    ofSetColor(200,200,0);
    ofLine(ofPoint(0,0,0), ofPoint(0,0,5000));
    ofDrawSphere(ofPoint(0,0,k2RotatePoint), 10);
    ofDrawBox(0,0,0,100,20,20);
    mesh.drawVertices();
    ofDisableDepthTest();
    ofPopMatrix();
}


//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
            
            case 's':
            //if(key == 's'){
                gui.saveToFile("settings.xml");
                ofxSaveCamera(kinect2cam, "kinect 2 settings");
            break;
            
            //}
            //else if(key == 'l'){
            case 'l':
                gui.loadFromFile("settings.xml");
                ofxLoadCamera(kinect2cam, "kinect 2 settings");
            break;
            //}
        /*
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
         */
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
	
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{

}