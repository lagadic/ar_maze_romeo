#ifndef ROMEOGRABBER_H_
#define ROMEOGRABBER_H_

#include <iostream>
#include <string>

#include "vpTemplateLocatization.h"

#include <visp3/core/vpDisplayX.h>
#include <visp3/core/vpImage.h>





#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp_naoqi/vpNaoqiGrabber.h>
#include <visp_naoqi/vpNaoqiConfig.h>
#include <visp/vpPoseVector.h>
#include <visp/vpTime.h>

#include "vpRomeoTkConfig.h"



extern vpPoseVector cMt;
extern pthread_mutex_t m_mutex;
extern pthread_cond_t  condition_var;





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




void *grab_compute_pose(void *)
{

  try
  {
    std::string opt_ip = "198.18.0.1";
    int opt_cam = 0;
    std::string opt_box_name = "star_wars_pic";
    std::string opt_data_folder = std::string(ROMEOTK_DATA_FOLDER);

    //  for (unsigned int i=0; i<argc; i++) {
    //      if (std::string(argv[i]) == "--box-name")
    //        opt_box_name = std::string(argv[i+1]);
    //      else if (std::string(argv[i]) == "--data-folder")
    //        opt_data_folder = std::string(argv[i+1]);
    //      else if (std::string(argv[i]) == "--help") {
    //        std::cout << "Usage: " << argv[0] << "[--box-name] [--data-folder]" << std::endl;

    //        return 0;
    //      }
    //    }

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
    vpImage<unsigned char> I(g.getHeight(), g.getWidth());
    vpDisplayX d(I);
    vpDisplay::setTitle(I, "ViSP viewer");

    std::cout << "Extrinsic Camera parameters: " << g.get_eMc()<< std::endl;




    std::string objects_folder = "objects/" + opt_box_name  + "/";
    std::string box_folder = opt_data_folder +"/" + objects_folder;
    std::string config_detection_file_folder = box_folder + "detection/";
    std::string objects_folder_det_learning = config_detection_file_folder + "learning/20/";
    std::string opt_model = box_folder + "model/" + opt_box_name;
    std::string learning_detection_file = "learning_data.bin";
    std::string learning_data_file_name = objects_folder_det_learning + learning_detection_file;

    std::cout << learning_data_file_name << std::endl;


    // Initialize the template tracker
    bool status_template_tracker;
    vpHomogeneousMatrix cMo_t;
    vpTemplateLocatization t_tracker(opt_model, config_detection_file_folder, cam);
    t_tracker.setTemplateSize(0.175,0.12);
    t_tracker.initDetection(learning_data_file_name);
    t_tracker.setValiditycMoFunction(checkValiditycMo);
    bool onlyDetection = true;
    t_tracker.setOnlyDetection(onlyDetection);

    // Homogeneous matrix from template frame visp to maze frame panda.
    vpHomogeneousMatrix tv_M_tp(0.0,0.0,0.0,vpMath::rad(180),0.0,0.0);
    // Homogeneous matrix from panda camera frame visp to visp camera.
    vpHomogeneousMatrix cp_M_cv(0.0,0.0,0.0,vpMath::rad(-90.0),0.0,0.0);

    //pthread_mutex_lock(&s_mutex); // Lock the mutex till we have a good pose


    while(1)
    {
      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);

      // track qrcode
      status_template_tracker = t_tracker.track(I);


      if (status_template_tracker ) { // display the tracking results
        cMo_t = t_tracker.get_cMo();
        //printPose("cMo qrcode: ", cMo_t);

        vpPoseVector _cpMtp(cp_M_cv*cMo_t*tv_M_tp);
        //vpPoseVector _cpMtp(0.0, 0.0, 0.0,vpMath::rad(90.0),0.0,0.0);

        pthread_mutex_lock(&m_mutex);


        cMt = _cpMtp;
        pthread_cond_signal( &condition_var );

        pthread_mutex_unlock(&m_mutex);




        vpDisplay::displayFrame(I, cMo_t* tv_M_tp, cam, 0.04, vpColor::none, 3);
        vpDisplay::displayPolygon(I, t_tracker.getCorners(), vpColor::green, 2);

      }
//      else
//      {
//        pthread_mutex_lock(&s_mutex);

//      }
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;

    }
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
