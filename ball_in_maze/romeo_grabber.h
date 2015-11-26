#ifndef ROMEOGRABBER_H_
#define ROMEOGRABBER_H_

#include <iostream>
#include <string>

// ViSP includes.
#include <visp3/core/vpDisplayX.h>
#include <visp3/core/vpImage.h>
#include <visp/vpPoseVector.h>
#include <visp/vpTime.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>
#include <visp/vpImageConvert.h>
#include <visp/vpMeterPixelConversion.h>

#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiConfig.h>

#include <vpServoHead.h>
#include <vpServoArm.h>
#include <vpCartesianDisplacement.h>
#include <vpRomeoTkConfig.h>
#include <vpBlobsTargetTracker.h>
#include <vpTemplateLocatization.h>

#include "world.h"

typedef enum {
  NoControll,
  LearnPoseTemplate,
  FindTemplate,
  CalibrateRigthArm,
  CalibrateLeftArm,
  WaitPreGrasp,
  PreGraps,
  VSBox,
  End
} State_t;


struct arg_holder {
  int argc;
  char ** argv;
};

extern cv::Mat m_cvI;
extern unsigned int m_count_img;
extern vpPoseVector cMt;
extern pthread_mutex_t m_mutex;
extern pthread_cond_t  condition_var;
extern pthread_mutex_t m_mutex_rg;
extern pthread_mutex_t m_mutex_img;
extern bool m_restart_game;
extern pthread_mutex_t m_mutex_com;
extern World::R_command m_command;

/*!
  Check the validity of the pose of the box.
*/

bool checkValiditycMo(vpHomogeneousMatrix cMo)
{
  
  double x = cMo[0][3];
  double y = cMo[1][3];
  double z = cMo[2][3];
  
  //std::cout << "x: " << x <<". Limits: -0.40 > y > 0.40" << std::endl;
  //std::cout << "y: " << y <<". Limits: -0.50 > y > 0.50" << std::endl;
  //std::cout << "z: " << z <<". Limits: 0.10 > y > 0.40" << std::endl;
  if (z < 0.10 || z > 0.40
      || x < - 0.20 || x > 0.10
      || y < -0.10 || y > 0.10 )
    return false;
  else
    return true;
}

void printPose(const std::string &text, const vpHomogeneousMatrix &cMo)
{
  vpTranslationVector t;
  cMo.extract(t);
  vpRotationMatrix R;
  cMo.extract(R);
  vpThetaUVector tu(R);
  
  std::cout << text;
  for (unsigned int i=0; i < 3; i++)
    std::cout << t[i] << " ";
  for (unsigned int i=0; i < 3; i++)
    std::cout << vpMath::deg(tu[i]) << " ";
  std::cout << std::endl;
}


void *grab_compute_pose(void * arg)
{
  
  struct arg_holder arg_struct = *(struct arg_holder *)arg;
  
  try
  {
    std::string opt_ip = "198.18.0.1";
    int opt_cam = 0;
    std::string opt_box_name = "robots_pic";
    std::string opt_data_folder = std::string(ROMEOTK_DATA_FOLDER);
    bool opt_learn_pose = false;
    bool opt_no_controll = false;
    
    for (unsigned int i=0; i<arg_struct.argc; i++) {
      if (std::string(arg_struct.argv[i]) == "--pic-name")
        opt_box_name = std::string(arg_struct.argv[i+1]);
      else if (std::string(arg_struct.argv[i]) == "--data-folder")
        opt_data_folder = std::string(arg_struct.argv[i+1]);
      else if (std::string(arg_struct.argv[i]) == "--init-pose")
        opt_learn_pose = true;
      else if (std::string(arg_struct.argv[i]) == "--no-controll")
        opt_no_controll = true;
      else if (std::string(arg_struct.argv[i]) == "--help") {
        std::cout << "Usage: " << arg_struct.argv[0] << "[--box-name] [--data-folder] [--no-controll]" << std::endl;

        pthread_exit(NULL);
      }
    }
    //******************************************* Grabber initialization ********************************************************

    vpNaoqiGrabber g;
    g.setCamera(opt_cam); // left camera
    if (! opt_ip.empty()) {
      std::cout << "Connect to robot with ip address: " << opt_ip << std::endl;
      g.setRobotIp(opt_ip);
    }
    
    g.open();
    std::cout << "Open camera parameters: " << g.getCameraParameters() << std::endl;
    std::cout << "Dimension image: " << g.getHeight() <<"x" << g.getWidth() << std::endl;
    
    vpCameraParameters cam = g.getCameraParameters(vpCameraParameters::perspectiveProjWithoutDistortion);

    cam.computeFov(g.getWidth(),g.getHeight());
    std::cout << "is Fov Computed: " << cam.isFovComputed() << std::endl;
    std::cout << "Horizontal Fov Angle: " << vpMath::deg( cam.getHorizontalFovAngle()) << std::endl;
    std::cout << "Vertical Fov Angle: " << vpMath::deg(cam.getVerticalFovAngle()) << std::endl;
    
    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");
    std::cout << "Extrinsic Camera parameters: " << g.get_eMc()<< std::endl;
    
    //Initialize opencv color image
    cv::Mat cvI = cv::Mat(cv::Size(g.getWidth(), g.getHeight()), CV_8UC3);

    //************************************ Path and transformation initialization ************************************************

    std::string objects_folder = "objects/" + opt_box_name  + "/";
    std::string box_folder = opt_data_folder +"/" + objects_folder;
    std::string config_detection_file_folder = box_folder + "detection/";
    std::string objects_folder_det_learning = config_detection_file_folder + "learning/20/";
    std::string opt_model = box_folder + "model/" + opt_box_name;
    std::string learning_detection_file = "learning_data.bin";
    std::string learning_data_file_name = objects_folder_det_learning + learning_detection_file;
    std::cout << learning_data_file_name << std::endl;

    vpHomogeneousMatrix cMt_initial;
    
    std::string learned_cMt_filename = "poses.xml";
    std::string learned_cMt_path = box_folder + "poses/"; // This file contains the following two transf. matrix:
    std::string name_initial_cMt = "initial_cMt";
    
    if (!opt_learn_pose) {
      vpXmlParserHomogeneousMatrix pm; // Create a XML parser
      
      if (pm.parse(cMt_initial, learned_cMt_path  + learned_cMt_filename, name_initial_cMt) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
        std::cout << "Cannot found the homogeneous matrix named " << name_initial_cMt<< "." << std::endl;
        return 0;
      }
      else
        std::cout << "Homogeneous matrix " << name_initial_cMt <<": " << std::endl << cMt_initial << std::endl;
    }
    
    
    vpHomogeneousMatrix M_offset;
    M_offset.buildFrom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) ;
    
    double d_t = 0.0;
    double d_r = 0.25;
    
    //Initialize desired poses
    std::vector<vpHomogeneousMatrix> des_poses(5);
    //0 - UP -x
    M_offset.buildFrom(0.0, 0.0, 0.0, -d_r, 0.0, 0.0) ;
    des_poses[0] = cMt_initial * M_offset;
    //1 - RIGHT y
    M_offset.buildFrom(0.0, 0.0, 0.0, 0.0, d_r, 0.0) ;
    des_poses[1] = cMt_initial * M_offset;
    //2 - DOWN x
    M_offset.buildFrom(0.0, 0.0, 0.0, d_r, 0.0, 0.0) ;
    des_poses[2] = cMt_initial * M_offset;
    //3 - LEFT -y
    M_offset.buildFrom(0.0, 0.0, 0.0, 0.0, -d_r, 0.0) ;
    des_poses[3] = cMt_initial * M_offset;
    //4 - INITIAL POSITION
    des_poses[4] = cMt_initial;
    
    M_offset.setIdentity();
    
    // Initialize the template tracker
    bool status_template_tracker;
    vpHomogeneousMatrix cMo_t;
    vpTemplateLocatization t_tracker(opt_model, config_detection_file_folder, cam);
    t_tracker.setTemplateSize(0.13,0.13);
    t_tracker.initDetection(learning_data_file_name);
    t_tracker.setValiditycMoFunction(checkValiditycMo);
    bool onlyDetection = true;
    t_tracker.setOnlyDetection(onlyDetection);
    
    // Homogeneous matrix from template frame visp to maze frame panda.
    vpHomogeneousMatrix tv_M_tp(0.0,0.0,0.0,vpMath::rad(180),0.0,0.0);
    // Homogeneous matrix from panda camera frame visp to visp camera.
    vpHomogeneousMatrix cp_M_cv(0.0,0.0,0.0,vpMath::rad(-90.0),0.0,0.0);
    
    //pthread_mutex_lock(&s_mutex); // Lock the mutex till we have a good pose
    
    
    //******************************************** Arm control *****************************************************
    
    std::vector<std::string> chain_name(2); // LArm or RArm
    chain_name[0] = "LArm";
    chain_name[1] = "RArm";
    
    
    /** Initialization target hands*/
    
    std::string opt_name_file_color_target_path = opt_data_folder + "/" +"target/";
    std::string opt_name_file_color_target_l = opt_name_file_color_target_path + chain_name[0] +"/color.txt";
    std::string opt_name_file_color_target_r = opt_name_file_color_target_path + chain_name[1] +"/color.txt";
    
    std::string opt_name_file_color_target1_l = opt_name_file_color_target_path + chain_name[0] +"/color1.txt";
    std::string opt_name_file_color_target1_r = opt_name_file_color_target_path + chain_name[1] +"/color1.txt";
    
    std::vector<vpBlobsTargetTracker*> hand_tracker;
    std::vector<bool>  status_hand_tracker(2);
    std::vector<vpHomogeneousMatrix>  cMo_hand(2);
    
    const double L1 = 0.025/2;
    std::vector <vpPoint> points1(4);
    points1[2].setWorldCoordinates(-L1,-L1, 0) ;
    points1[1].setWorldCoordinates(-L1,L1, 0) ;
    points1[0].setWorldCoordinates(L1,L1, 0) ;
    points1[3].setWorldCoordinates(L1,-L1,0) ;
    
    
    vpBlobsTargetTracker hand_tracker_l;
    hand_tracker_l.setName(chain_name[0]);
    hand_tracker_l.setCameraParameters(cam);
    hand_tracker_l.setPoints(points1);
    hand_tracker_l.setLeftHandTarget(true);
    
    if(!hand_tracker_l.loadHSV(opt_name_file_color_target_l))
      std::cout << "Error opening the file "<< opt_name_file_color_target_l << std::endl;
    
    
    vpBlobsTargetTracker hand_tracker_r;
    hand_tracker_r.setName(chain_name[1]);
    hand_tracker_r.setCameraParameters(cam);
    hand_tracker_r.setPoints(points1);
    hand_tracker_r.setLeftHandTarget(false);
    
    if(!hand_tracker_r.loadHSV(opt_name_file_color_target1_r))
      std::cout << "Error opening the file "<< opt_name_file_color_target_r << std::endl;
    
    hand_tracker.push_back(&hand_tracker_l);
    hand_tracker.push_back(&hand_tracker_r);
    

    // Constant transformation Target Frame to Arm end-effector (WristPitch)
    std::vector<vpHomogeneousMatrix> hand_Me_Arm(2);
    std::string filename_transform = std::string(ROMEOTK_DATA_FOLDER) + "/transformation.xml";
    // Create twist matrix from box to Arm end-effector (WristPitch)
    std::vector <vpVelocityTwistMatrix> box_Ve_Arm(2);
    // Create twist matrix from target Frame to Arm end-effector (WristPitch)
    std::vector <vpVelocityTwistMatrix> hand_Ve_Arm(2);
    
    // Constant transformation Target Frame to box to target Hand
    std::vector<vpHomogeneousMatrix> box_Mhand(2);
    
    
    for (unsigned int i = 0; i < 2; i++)
    {
      
      std::string name_transform = "qrcode_M_e_" + chain_name[i];
      vpXmlParserHomogeneousMatrix pm; // Create a XML parser
      
      if (pm.parse(hand_Me_Arm[i], filename_transform, name_transform) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
        std::cout << "Cannot found the homogeneous matrix named " << name_transform << "." << std::endl;
        return 0;
      }
      else
        std::cout << "Homogeneous matrix " << name_transform <<": " << std::endl << hand_Me_Arm[i] << std::endl;
      
      hand_Ve_Arm[i].buildFrom(hand_Me_Arm[i]);
      
    }
    
    /************************************************************************************************/
    
    /** Create a new istance NaoqiRobot*/
    vpNaoqiRobot robot;
    if (! opt_ip.empty())
      robot.setRobotIp(opt_ip);
    robot.open();
    
    std::vector<std::string> jointNames = robot.getBodyNames("Head");
    //jointNames.pop_back(); // We don't consider  the last joint of the head = HeadRoll
    std::vector<std::string> jointNamesLEye = robot.getBodyNames("LEye");
    std::vector<std::string> jointNamesREye = robot.getBodyNames("REye");
    
    jointNames.insert(jointNames.end(), jointNamesLEye.begin(), jointNamesLEye.end());
    std::vector<std::string> jointHeadNames_tot = jointNames;
    jointHeadNames_tot.push_back(jointNamesREye.at(0));
    jointHeadNames_tot.push_back(jointNamesREye.at(1));
    
    std::vector<bool> grasp_servo_converged(2);
    grasp_servo_converged[0]= false;
    grasp_servo_converged[1]= false;
    
    std::vector<vpMatrix> eJe(2);
    
    // Initialize arms servoing
    vpServoArm servo_larm_l;
    vpServoArm servo_larm_r;
    
    std::vector<vpServoArm*> servo_larm;
    servo_larm.push_back(&servo_larm_l);
    servo_larm.push_back(&servo_larm_r);
    
    std::vector < std::vector<std::string > > jointNames_arm(2);
    
    jointNames_arm[0] = robot.getBodyNames(chain_name[0]);
    jointNames_arm[1] = robot.getBodyNames(chain_name[1]);
    // Delete last joint Hand, that we don't consider in the servo
    jointNames_arm[0].pop_back();
    jointNames_arm[1].pop_back();
    
    std::vector<std::string> jointArmsNames_tot = jointNames_arm[0];
    
    jointArmsNames_tot.insert(jointArmsNames_tot.end(), jointNames_arm[1].begin(), jointNames_arm[1].end());
    
    const unsigned int numArmJoints =  jointNames_arm[0].size();
    std::vector<vpHomogeneousMatrix> box_dMbox(2);
    
    // Vector containing the joint velocity vectors of the arms
    std::vector<vpColVector> q_dot_arm;
    // Vector containing the joint velocity vectors of the arms for the secondary task
    std::vector<vpColVector> q_dot_arm_sec;
    // Vector joint position of the arms
    std::vector<vpColVector> q;
    // Vector joint real velocities of the arms
    std::vector<vpColVector> q_dot_real;
    
    vpColVector   q_temp(numArmJoints);
    q_dot_arm.push_back(q_temp);
    q_dot_arm.push_back(q_temp);
    
    q_dot_arm_sec.push_back(q_temp);
    q_dot_arm_sec.push_back(q_temp);
    
    q.push_back(q_temp);
    q.push_back(q_temp);
    
    q_dot_real.push_back(q_temp);
    q_dot_real.push_back(q_temp);
    
    
    
    // Initialize the joint avoidance scheme from the joint limits
    std::vector<vpColVector> jointMin;
    std::vector<vpColVector> jointMax;
    
    jointMin.push_back(q_temp);
    jointMin.push_back(q_temp);
    
    jointMax.push_back(q_temp);
    jointMax.push_back(q_temp);
    
    for (unsigned int i = 0; i< 2; i++)
    {
      jointMin[i] = robot.getJointMin(chain_name[i]);
      jointMax[i] = robot.getJointMax(chain_name[i]);
      
      jointMin[i].resize(numArmJoints,false);
      jointMax[i].resize(numArmJoints,false);
      
      //        std::cout <<  jointMin[i].size() << std::endl;
      //        std::cout <<  jointMax[i].size() << std::endl;
      //         std::cout << "max " <<  jointMax[i] << std::endl;
    }
    

    std::vector<bool> first_time_arm_servo(2);
    first_time_arm_servo[0] = true;
    first_time_arm_servo[1] = true;
    
    std::vector< double> servo_arm_time_init(2);
    servo_arm_time_init[0] = 0.0;
    servo_arm_time_init[1] = 0.0;
    
    std::vector<int> cpt_iter_servo_grasp(2);
    cpt_iter_servo_grasp[0] = 0;
    cpt_iter_servo_grasp[1] = 0;
    
    unsigned int index_hand = 1;
    
    bool first_time_box_pose = true;
    
    /************************************************************************************************/
    
    vpMouseButton::vpMouseButtonType button;
    
    vpHomogeneousMatrix elMb; // Homogeneous matrix from right wrist roll to box
    vpHomogeneousMatrix cMo_t_d = cMt_initial; // Desired box final pose
    
    State_t state;
    
    if (opt_learn_pose)
      state = LearnPoseTemplate;
    else if (opt_no_controll)
      state = NoControll;
    else
      state = FindTemplate;
    
    
    AL::ALValue angles_head      = AL::ALValue::array(vpMath::rad(-4.6), vpMath::rad(15.0), vpMath::rad(6.3), vpMath::rad(0.0), vpMath::rad(6.0) , vpMath::rad(9.8), 0.0, 0.0  );
    float fractionMaxSpeed  = 0.1f;
    robot.getProxy()->setAngles(jointHeadNames_tot, angles_head, fractionMaxSpeed);
    
    while(1)
    {
      double t = vpTime::measureTimeMs();
      
      g.acquire(cvI);
      vpImageConvert::convert(cvI, I);
      pthread_mutex_lock(&m_mutex_img);
      m_cvI = cvI.clone();
      m_count_img++;
      pthread_mutex_unlock(&m_mutex_img);
      
      vpDisplay::display(I);
      
      bool click_done = vpDisplay::getClick(I, button, false);
      
      char key[10];
      bool ret = vpDisplay::getKeyboardEvent(I, key, false);
      std::string s = key;
      
      if (ret)
      {
        //        if (s == "r")
        //        {
        //          M_offset.buildFrom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) ;
        
        //          d_t = 0.0;
        //          d_r = 0.15;
        //          std::cout << "Rotation mode. " << std::endl;
        //        }
        
        //        if (s == "t")
        //        {
        //          M_offset.buildFrom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) ;
        
        //          d_t = 0.01;
        //          d_r = 0.0;
        //          std::cout << "Translation mode. " << std::endl;
        //        }
        if (s == "r")
        {
          cMo_t_d = cMt_initial;
          pthread_mutex_lock(&m_mutex_rg);
          m_restart_game = true;
          pthread_mutex_unlock(&m_mutex_rg);
          first_time_box_pose = true;
        }
        
        if (s == "h")
        {
          hand_tracker[index_hand]->setManualBlobInit(true);
          hand_tracker[index_hand]->setForceDetection(true);
        }
        else if ( s == "+")
        {
          unsigned int value = hand_tracker[index_hand]->getGrayLevelMaxBlob() +10;
          hand_tracker[index_hand]->setGrayLevelMaxBlob(value);
          std::cout << "Set to "<< value << "the value of  " << std::endl;
          
        }
        else if (s == "-")
        {
          unsigned int value = hand_tracker[1]->getGrayLevelMaxBlob()-10;
          hand_tracker[index_hand]->setGrayLevelMaxBlob(value-10);
          std::cout << "Set to "<< value << " GrayLevelMaxBlob. " << std::endl;
        }
        
        //          |x
        //      z\  |
        //        \ |
        //         \|_____ y
        //
        
        else if (s == "4") //-y
        {
          M_offset.buildFrom(0.0, -d_t, 0.0, 0.0, -d_r, 0.0) ;
        }
        
        else if (s == "6")  //+y
        {
          M_offset.buildFrom(0.0, d_t, 0.0, 0.0, d_r, 0.0) ;
        }
        
        else if (s == "8")  //+x
        {
          M_offset.buildFrom(d_t, 0.0, 0.0, d_r, 0.0, 0.0) ;
        }
        
        else if (s == "2") //-x
        {
          M_offset.buildFrom(-d_t, 0.0, 0.0, -d_r, 0.0, 0.0) ;
        }
        
        else if (s == "7")//-z
        {
          M_offset.buildFrom(0.0, 0.0, -d_t, 0.0, 0.0, -d_r) ;
        }
        else if (s == "9") //+z
        {
          M_offset.buildFrom(0.0, 0.0, d_t, 0.0, 0.0, d_r) ;
        }
        
        cMo_t_d = cMo_t_d * M_offset;
        M_offset.setIdentity();
        
      }
      
      // Detect and track hand targets
      if (state < WaitPreGrasp && state != NoControll)
      {
        status_hand_tracker[index_hand] = hand_tracker[index_hand]->track(cvI,I);
        
        if (status_hand_tracker[index_hand] ) { // display the tracking results
          cMo_hand[index_hand] = hand_tracker[index_hand]->get_cMo();
          //printPose("cMo right arm: ", cMo_hand[index_hand]);
          // The qrcode frame is only displayed when PBVS is active or learning
          
          vpDisplay::displayFrame(I, cMo_hand[index_hand], cam, 0.04, vpColor::none, 3);
        }
        
      }
      
      // track template
      status_template_tracker = t_tracker.track(I);
      
      if (status_template_tracker ) { // display the tracking results
        cMo_t = t_tracker.get_cMo()*tv_M_tp;
        //printPose("cMo qrcode: ", cMo_t);
        
        vpHomogeneousMatrix offset(0.002, 0.025, 0.0, 0.0, 0.0, 0.0);
        
        vpPoseVector _cpMtp(cp_M_cv*cMo_t*offset);
        //vpPoseVector _cpMtp(0.0, 0.0, 0.0,vpMath::rad(90.0),0.0,0.0);
        
        //printPose("Maze: ", _cpMtp);
        
        pthread_mutex_lock(&m_mutex);
        cMt = _cpMtp;
        if (opt_no_controll)
          pthread_cond_signal( &condition_var );
        pthread_mutex_unlock(&m_mutex);
        
        vpDisplay::displayFrame(I, cMo_t, cam, 0.04, vpColor::none, 3);
        vpDisplay::displayPolygon(I, t_tracker.getCorners(), vpColor::green, 2);
        
      }
      
      if (state == LearnPoseTemplate &&  status_template_tracker )
      {
        vpDisplay::displayText(I, vpImagePoint(10,10), "Left click to save the pose", vpColor::red);
        printPose("cMo qrcode: ", cMo_t);
        
        if (click_done && button == vpMouseButton::button1 ) {
          vpXmlParserHomogeneousMatrix p; // Create a XML parser
          
          //vpHomogeneousMatrix cMo_t_des(-0.009528060139, 0.0, 0.3726390759, vpMath::rad(135.7879077), vpMath::rad(0.0), vpMath::rad(-8.483044353));
          vpHomogeneousMatrix cMo_t_des(-0.01515260512, -0.02381981134, 0.3458028533, vpMath::rad(140.9292059), vpMath::rad(0.0), vpMath::rad(-3.302342968));
          
          if (p.save(cMo_t_des, learned_cMt_path + "/" + learned_cMt_filename , name_initial_cMt) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK)
          {
            std::cout << "Cannot save the Homogeneous matrix" << std::endl;
            return 0;
          }
          printPose("The desired template pose: ", cMt);
          std::cout << "is saved in " << learned_cMt_path + learned_cMt_filename << std::endl;
          return 0;
        }
      }
      
      if (state == FindTemplate &&  status_template_tracker )
      {
        if (click_done && button == vpMouseButton::button1 ) {
          
          angles_head      = AL::ALValue::array(vpMath::rad(-15.2), vpMath::rad(17.6), vpMath::rad(10.3), vpMath::rad(0.0), vpMath::rad(6.0) , vpMath::rad(9.8), 0.0, 0.0  );
          fractionMaxSpeed  = 0.01f;
          robot.getProxy()->setAngles(jointHeadNames_tot, angles_head, fractionMaxSpeed);
          
          click_done = false;
          state = CalibrateRigthArm;
        }
      }
      
      if (state == CalibrateRigthArm &&  status_template_tracker && status_hand_tracker[index_hand] )
      {
        vpDisplay::displayText(I, vpImagePoint(I.getHeight() - 10, 10), "Left click to calibrate right hand", vpColor::red);
        
        vpHomogeneousMatrix box_Me_Arm; // Homogeneous matrix from the box to the left wrist
        
        box_Mhand[index_hand] = cMo_t.inverse() *  cMo_hand[index_hand];
        box_Me_Arm = box_Mhand[index_hand] * hand_Me_Arm[index_hand] ; // from box to WristPitch
        
        // vpDisplay::displayFrame(I, cMo_hand[index_hand] * (cMo_t.inverse() *  cMo_hand[index_hand]).inverse() , cam, 0.04, vpColor::green, 1);
        
        if (click_done && button == vpMouseButton::button1 ) {
          
          box_Ve_Arm[index_hand].buildFrom(box_Me_Arm);
          index_hand = 0;
          state = CalibrateLeftArm;
          
          // Small Plate
          AL::ALValue angles_head      = AL::ALValue::array(vpMath::rad(4.8), vpMath::rad(17.6), vpMath::rad(10.3), vpMath::rad(0.0), vpMath::rad(6.0) , vpMath::rad(9.8), 0.0, 0.0  );
          
          float fractionMaxSpeed  = 0.01f;
          robot.getProxy()->setAngles(jointHeadNames_tot, angles_head, fractionMaxSpeed);
          
          click_done = false;
        }
        
      }
      
      if (state == CalibrateLeftArm && status_template_tracker && status_hand_tracker[index_hand] )
      {
        vpDisplay::displayText(I, vpImagePoint(I.getHeight() - 10, 10), "Left click to calibrate left hand", vpColor::red);
        
        vpHomogeneousMatrix box_Me_Arm; // Homogeneous matrix from the box to the left wrist
        box_Mhand[index_hand] = cMo_t.inverse() *  cMo_hand[index_hand];
        box_Me_Arm = box_Mhand[index_hand] * hand_Me_Arm[index_hand] ; // from box to WristPitch
        
        //            if (first_time_box_pose)
        //            {
        //                // Compute desired box position cMo_t_d
        //                cMo_t_d = cMo_t * M_offset;
        //                first_time_box_pose = false;
        
        //            }
        
        // vpDisplay::displayFrame(I, cMo_t_d, cam, 0.04, vpColor::red, 1);
        
        // vpDisplay::displayFrame(I, cMo_hand[index_hand] * (cMo_t.inverse() *  cMo_hand[index_hand]).inverse() , cam, 0.04, vpColor::green, 1);
        
        if (click_done && button == vpMouseButton::button1 ) {
          
          box_Ve_Arm[index_hand].buildFrom(box_Me_Arm);
          state = WaitPreGrasp;
          click_done = false;
          //AL::ALValue names_head     = AL::ALValue::array("NeckYaw","NeckPitch","HeadPitch","HeadRoll","LEyeYaw", "LEyePitch","LEyeYaw", "LEyePitch" );
          AL::ALValue angles_head      = AL::ALValue::array(vpMath::rad(-4.2), vpMath::rad(14.9), vpMath::rad(5.3), vpMath::rad(0.0), vpMath::rad(6.0) , vpMath::rad(9.8), 0.0, 0.0  );
          float fractionMaxSpeed  = 0.01f;
          robot.getProxy()->setAngles(jointHeadNames_tot, angles_head, fractionMaxSpeed);
        }
      }
      
      if (state == WaitPreGrasp  )
      {
        index_hand = 1;
        
        if (click_done && button == vpMouseButton::button1 ) {
          
          state = PreGraps;
          click_done = false;
        }
        
      }
      
      if (state == PreGraps && status_template_tracker )
      {
        
        vpDisplay::displayText(I, vpImagePoint(I.getHeight() - 10, 10), "Left click to start the servo", vpColor::red);
        vpDisplay::displayFrame(I, cMo_t_d , cam, 0.05, vpColor::none, 3);
        vpDisplay::displayFrame(I, cMo_t *box_Mhand[1] , cam, 0.05, vpColor::none, 3);
        vpDisplay::displayFrame(I, cMo_t *box_Mhand[0] , cam, 0.05, vpColor::none, 3);
        
        if (click_done && button == vpMouseButton::button1 ) {

          state = VSBox;
          click_done = false;
          
          //hand_tracker[index_hand] = &box_tracker; // Trick to use keys
        }
      }

      if (state == VSBox)
      {
        //        if (first_time_box_pose)
        //        {
        //          vpTime::sleepMs(500);
        //          // Compute desired box position cMo_t_d
        //         // cMo_t_d = cMo_t * M_offset;
        //          pthread_mutex_lock(&m_mutex);
        //          pthread_cond_signal( &condition_var );
        //          pthread_mutex_unlock(&m_mutex);
        //          first_time_box_pose = false;
        //        }
        
        //Get Actual position of the arm joints
        q[0] = robot.getPosition(jointNames_arm[0]);
        q[1] = robot.getPosition(jointNames_arm[1]);
        
        //  if(status_template_tracker && status_hand_tracker[index_hand] )
        if(status_template_tracker )
        {
          
          if (! grasp_servo_converged[0]) {

            unsigned int i = 0;
            vpAdaptiveGain lambda(0.8, 0.05, 8);
            //vpAdaptiveGain lambda(0.4, 0.02, 4);
            
            servo_larm[i]->setLambda(lambda);
            // servo_larm[i]->setLambda(0.2);

            eJe[i] = robot.get_eJe(chain_name[i]);
            
            servo_larm[i]->set_eJe(eJe[i]);
            servo_larm[i]->m_task.set_cVe(box_Ve_Arm[i]);
            
            box_dMbox[i] = cMo_t_d.inverse() * cMo_t;
            //printPose("box_dMbox: ", box_dMbox[i]);
            servo_larm[i]->setCurrentFeature(box_dMbox[i]) ;
            
            vpDisplay::displayFrame(I, cMo_t_d , cam, 0.05, vpColor::none, 3);
            
            
            //                    vpDisplay::displayFrame(I, cMo_t *box_Mhand[1] , cam, 0.05, vpColor::none, 3);
            //                    vpDisplay::displayFrame(I, cMo_t *box_Mhand[0] , cam, 0.05, vpColor::none, 3);
            
            if (first_time_arm_servo[i]) {
              std::cout << "-- Start visual servoing of the arm" << chain_name[i] << "." << std::endl;
              servo_arm_time_init[i] = vpTime::measureTimeSecond();
              first_time_arm_servo[i] = false;
            }
            
            q_dot_arm[i] =  - servo_larm[i]->computeControlLaw(servo_arm_time_init[i]);
            
            q_dot_real[0] = robot.getJointVelocity(jointNames_arm[0]);
            q_dot_real[1] = robot.getJointVelocity(jointNames_arm[1]);
            
            //                    std::cout << "real_q:  " << std::endl <<  real_q << std::endl;
            
            //                    std::cout << " box_Ve_Arm[i]:  " << std::endl << box_Ve_Arm[i] << std::endl;
            //                    std::cout << "  eJe[i][i]:  " << std::endl <<  eJe[i] << std::endl;
            
            //vpColVector real_v = (box_Ve_Arm[i] * eJe[i]) *  q_dot_real[0];
            vpColVector real_v = (box_Ve_Arm[i] * eJe[i]) *  q_dot_arm[0];
            
            //                    std::cout << "real_v:  " << std::endl <<real_v << std::endl;
            //          vpVelocityTwistMatrix cVo(cMo_hand);
            //          vpMatrix cJe = cVo * oJo;
            // Compute the feed-forward terms
            //          vpColVector sec_ter = 0.5 * ((servo_head.m_task_head.getTaskJacobianPseudoInverse() *  (servo_head.m_task_head.getInteractionMatrix() * cJe)) * q_dot_larm);
            //          std::cout <<"Second Term:" <<sec_ter << std::endl;
            //q_dot_head = q_dot_head + sec_ter;
            
            // Compute joint limit avoidance
            q_dot_arm_sec[0]  = servo_larm[0]->m_task.secondaryTaskJointLimitAvoidance(q[0], q_dot_real[0], jointMin[0], jointMax[0]);
            //q_dot_arm_sec[1]  = servo_larm[1]->m_task.secondaryTaskJointLimitAvoidance(q[1], q_dot_real[1], jointMin[1], jointMax[1]);
            
            // vpColVector q_dot_arm_head = q_dot_larm + q2_dot;
            
            //q_dot_arm_head.stack(q_dot_tot);
            //          robot.setVelocity(joint_names_arm_head,q_dot_arm_head);
            //robot.setVelocity(jointNames_arm[i], q_dot_larm);
            
            //          if (opt_plotter_arm) {
            //            plotter_arm->plot(0, cpt_iter_servo_grasp, servo_larm.m_task.getError());
            //            plotter_arm->plot(1, cpt_iter_servo_grasp, q_dot_larm);
            //          }
            
            //          if (opt_plotter_q_sec_arm)
            //          {
            //            plotter_q_sec_arm->plot(0,loop_iter,q2_dot);
            //            plotter_q_sec_arm->plot(1,loop_iter,q_dot_larm + q2_dot);
            
            //          }
            
            cpt_iter_servo_grasp[i] ++;

            eJe[1] = robot.get_eJe(chain_name[1]);
            //                    q_dot_arm[1] += (box_Ve_Arm[1] * eJe[1]).pseudoInverse() * real_v;
            
            q_dot_arm[1] = (box_Ve_Arm[1] * eJe[1]).pseudoInverse() * real_v;
            
            vpColVector q_dot_tot = q_dot_arm[0] + q_dot_arm_sec[0];
            
            q_dot_tot.stack( q_dot_arm[1] + q_dot_arm_sec[1]);
            
            robot.setVelocity(jointArmsNames_tot, q_dot_tot);
            
            vpTranslationVector t_error_grasp = box_dMbox[0].getTranslationVector();
            vpRotationMatrix R_error_grasp;
            box_dMbox[0].extract(R_error_grasp);
            vpThetaUVector tu_error_grasp;
            tu_error_grasp.buildFrom(R_error_grasp);
            double theta_error_grasp;
            vpColVector u_error_grasp;
            tu_error_grasp.extract(theta_error_grasp, u_error_grasp);

            double error_t_treshold = 0.006;
            
            if ( (sqrt(t_error_grasp.sumSquare()) < error_t_treshold) && (theta_error_grasp < vpMath::rad(2)) && first_time_box_pose)
            {
              std::cout << "Reached initial pose. The game is starting!" << std::endl;
              
              // grasp_servo_converged[0] = true;
              
              std::cout << "Click to start demo." << std::endl;

              if (click_done && button == vpMouseButton::button1)
              {
                
                vpTime::sleepMs(500);
                robot.stop(jointArmsNames_tot);
                // Compute desired box position cMo_t_d
                // cMo_t_d = cMo_t * M_offset;
                pthread_mutex_lock(&m_mutex);
                pthread_cond_signal( &condition_var );
                pthread_mutex_unlock(&m_mutex);
                first_time_box_pose = false;
                click_done = false;
              }
            }

            if (!first_time_box_pose)
            {
              pthread_mutex_lock(&m_mutex_com);
              World::R_command command = m_command;
              pthread_mutex_unlock(&m_mutex_com);
              
              switch (command)
              {
              case(World::Up):
                cMo_t_d = des_poses[0];
                break;
                
              case(World::Right):
                cMo_t_d = des_poses[1];
                break;
                
              case(World::Down):
                cMo_t_d = des_poses[2];
                break;
                
              case(World::Left):
                cMo_t_d = des_poses[3];
                break;
                
              case(World::Zero):
                cMo_t_d = des_poses[4];
                break;
                
              default:
                cMo_t_d = des_poses[4];
                
              }
              
            }
            
          }
          
        }
        else {
          // state grasping but one of the tracker fails
          robot.stop(jointArmsNames_tot);
        }
        
        
      }
      
      
      if (state == End)
      {
        std::cout<< "End" << std::endl;
      }
      
      vpDisplay::flush(I);
      if (click_done && button == vpMouseButton::button3)
        break;
      
      //std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
      
    }
    robot.stop(jointArmsNames_tot);
  }
  catch (const vpException &e)
  {
    std::cerr << "Caught exception: " << e.what() << std::endl;
  }
  catch (const AL::ALError &e)
  {
    std::cerr << "Caught exception: " << e.what() << std::endl;
  }
  
  pthread_exit(NULL);
}

#endif
