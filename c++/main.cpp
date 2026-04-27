/*
Based on the Bullet Physics RagdollDemo, modified with help of Josh Bongard's MOOC ludobots

Author: Joshua Hathorne-Madell
*/
#include "NoiseWorld.cpp"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"


/*
g++ -O2 -g -Wall -Wl,-rpath=./more_libs/lib,--enable-new-dtags -std=gnu++11 -I../bullet-2.82-r2704/Demos/OpenGL/ -I./more_libs/include/ -I../bullet-2.82-r2704/src/ ./main.cpp -L../bullet-2.82-r2704/Glut/ -L./more_libs/lib/ -L./more_libs/mesa -L../bullet-build/Demos/OpenGL/ -L../more_libs/lib/x86_64-linux-gnu -L../bullet-build/src/BulletDynamics/ -L../bullet-build/src/BulletCollision/ -L../bullet-build/src/LinearMath/ -lOpenGLSupport -lGL -lGLU -lglut -lBulletDynamics -lBulletCollision -lLinearMath -lXi -lXxf86vm -lX11 -o ./app

g++ -fno-inline -O0 -g -Wall -Wl,-rpath=./more_libs/lib,--enable-new-dtags -std=gnu++11 -I../bullet-2.82-r2704/Demos/OpenGL/ -I./more_libs/include/ -I../bullet-2.82-r2704/src/ ./main.cpp -L../bullet-2.82-r2704/Glut/ -L./more_libs/lib/ -L./more_libs/mesa -L../bullet-build/Demos/OpenGL/ -L../more_libs/lib/x86_64-linux-gnu -L../bullet-build/src/BulletDynamics/ -L../bullet-build/src/BulletCollision/ -L../bullet-build/src/LinearMath/ -lOpenGLSupport -lGL -lGLU -lglut -lBulletDynamics -lBulletCollision -lLinearMath -lXi -lXxf86vm -lX11 -o ./app

g++ -O0 -fno-inline -g -std=gnu++11 -I/home/josh/bullet-2.82-r2704/Demos/OpenGL/ -I/home/josh/bullet-2.82-r2704/src/ ./main.cpp -L/home/josh/bullet-build/Demos/OpenGL/ -L/home/josh/bullet-2.82-r2704/bullet-build/src/BulletDynamics/ -L/home/josh/bullet-2.82-r2704/bullet-build/src/BulletCollision/ -L/home/josh/bullet-2.82-r2704/bullet-build/src/LinearMath/ -L/home/josh/bullet-2.82-r2704/bullet-build/Demos/OpenGL -lOpenGLSupport -lGL -lGLU -lglut -lBulletDynamics -lBulletCollision -lLinearMath -o ./app
*/


//GLDebugDrawer	gDebugDrawer;

int main(int argc, char *argv[])

{
  // Initialize GLUT framework first (required for graphics initialization)
  // Safe even in headless mode as glutSwapBuffers is guarded by drawGraphics flag
  glutInit(&argc, argv);
  
  NoiseWorld simulation;
  /*
  if (argc == 1)
    { 
      simulation.drawGraphics = true;
      return glutmain(argc, argv, 640, 480, "Bullet Physics Demo. http://bulletphysics.com",
		      &simulation);
    }
  */
  if (argc == 5)
    {
      simulation.drawGraphics = true;
      simulation.ParseCmdLine(argv[1]);
      simulation.ParseCmdLine(argv[2]);
      simulation.ParseCmdLine(argv[3]);
      simulation.ParseCmdLine(argv[4]);
      simulation.initPhysics();
      //simulation.getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
      return glutmain(argc, argv, 640, 480, "Bullet Physics Demo. http://bulletphysics.com",
		      &simulation);
    }
  else if (argc == 4)
    {
      simulation.drawGraphics = false;
      simulation.ParseCmdLine(argv[1]);
      simulation.ParseCmdLine(argv[2]);
      simulation.ParseCmdLine(argv[3]);
      simulation.initPhysics();
      //simulation.getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
      //cout << "----------START SIM " << argv[1] << "----------" << endl;
      while(simulation.timeStep <= 500) 
	{
	  simulation.clientMoveAndDisplay();
	}
      //cout << "-----------END SIM " << argv[1] << "-----------" << endl;
      return simulation.Save_Position(true);
    }
  else if (argc == 3)
    {
      simulation.drawGraphics = false;
      simulation.ParseCmdLine(argv[1]);
      simulation.ParseCmdLine(argv[2]);
      simulation.initPhysics();
      //simulation.getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
      //cout << "----------START SIM " << argv[1] << "----------" << endl;
      while(simulation.timeStep <= 500) 
	{
	  simulation.clientMoveAndDisplay();
	}
      //cout << "-----------END SIM " << argv[1] << "-----------" << endl;
      return simulation.Save_Position(true);
    }
  else
    {
      cout << "Didn't get 2, 3, or 4 switches!" << endl;
      cout << "Got: " << argv << endl;
    }
}
