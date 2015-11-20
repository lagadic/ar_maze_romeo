/*
 * tut_ball_in_maze.cpp
 *
 *  Created on: 2012-05-19
 *      Author: dri
 *
 * This is a python to C++ translation of Panda3d 1.7 sample/tutorial:
 * Tut-Ball-in-Maze.py
 *
 * Here's the original python tutorial header:
 *
 * Author: Shao Zhang, Phil Saltzman
 * Last Updated: 5/2/2005
 *
 * This tutorial shows how to detect and respond to collisions. It uses solids
 * create in code and the egg files, how to set up collision masks, a traverser,
 * and a handler, how to detect collisions, and how to dispatch function based
 * on the collisions. All of this is put together to simulate a labyrinth-style
 * game
 *
 */

//#if PANDA_NUMERIC_VERSION >= 1008000
#define Colorf LColorf
#define PNG_SKIP_SETJMP_CHECK
//#endif
#include "world.h"
#include "romeo_grabber.h"



vpPoseVector cMt;
cv::Mat m_cvI =  cv::Mat(cv::Size(320,240), CV_8UC3);
unsigned int m_count_img = 0;
#ifdef VISP_HAVE_PTHREAD
pthread_mutex_t m_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t condition_var = PTHREAD_COND_INITIALIZER;
pthread_mutex_t m_mutex_img = PTHREAD_MUTEX_INITIALIZER;
// Restart game
pthread_mutex_t m_mutex_rg = PTHREAD_MUTEX_INITIALIZER;
bool m_restart_game = false;
pthread_mutex_t m_mutex_com = PTHREAD_MUTEX_INITIALIZER;
World::R_command m_command = World::Zero;

#endif
bool valid_cMt = 0;



//void * grab_compute_pose(void *);
void * ar_panda(void *arg);


//void *grab_compute_pose(void *)
//{
//  float count = 0.0;

//  for ( ; ; ){

//    vpPoseVector _cMt(0.0,0.0,0.0,0.0,0.0, count);
//    #ifdef VISP_HAVE_PTHREAD
//    pthread_mutex_lock(&m_mutex);
//    #endif

//    cMt = _cMt;

//    #ifdef VISP_HAVE_PTHREAD
//    pthread_mutex_unlock(&m_mutex);
//    #endif

//   count += 0.1;



//   _cMt.print();

//    vpTime::sleepMs(100);

//  }

//   pthread_exit(NULL);
//}


void *ar_panda(void * arg)
{

struct arg_holder arg_struct = *(struct arg_holder *)arg;

  // setup Panda3d
  PandaFramework pandaFramework;
  pandaFramework.open_framework(arg_struct.argc, arg_struct.argv);
 // WindowProperties prop;
 // prop.get_default();
 // prop.set_size(320,240);

  PT(WindowFramework) windowFrameworkPtr = pandaFramework.open_window();
  if(windowFrameworkPtr == NULL)
  {
    nout << "ERROR: could not open the WindowFramework." << endl;
    return 0; // error
  }
  
  

  // Finally, create an instance of our class and start 3d rendering
  World world(windowFrameworkPtr);

  PT(FrameRateMeter) meter = new FrameRateMeter("fps");
  meter->setup_window(windowFrameworkPtr->get_graphics_output());

  // Run the simulation
  pandaFramework.main_loop();

  std::cout << "############################END THREAD PANDA" <<std::endl;
  // quit Panda3d
  pandaFramework.close_framework();
  free(arg);



  pthread_exit(NULL);

}


int main(int argc, char *argv[])
{

#ifdef VISP_HAVE_PTHREAD
    pthread_t thread_romeo;
    pthread_t thread_maze;


    struct arg_holder * arg_struct = new arg_holder;
    arg_struct->argc = argc;
    arg_struct->argv = argv;


    pthread_create(&thread_romeo, NULL, &grab_compute_pose, arg_struct);
    pthread_create(&thread_maze, NULL, &ar_panda, arg_struct);

//  // setup Panda3d
//  PandaFramework pandaFramework;
//  pandaFramework.open_framework(argc, argv);
//  PT(WindowFramework) windowFrameworkPtr = pandaFramework.open_window();
//  if(windowFrameworkPtr == NULL)
//  {
//    nout << "ERROR: could not open the WindowFramework." << endl;
//    return 1; // error
//  }

//  // Finally, create an instance of our class and start 3d rendering
//  World world(windowFrameworkPtr);

//  // Run the simulation
//  pandaFramework.main_loop();
  pthread_join(thread_maze, 0);
  pthread_cancel(thread_romeo);
  //pthread_join(thread_romeo, 0);


pthread_mutex_destroy(&m_mutex);
pthread_mutex_destroy(&m_mutex_rg);

#endif
//  // quit Panda3d
//  pandaFramework.close_framework();
  return 0; // success
}
