#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>


#define TIME_STEP 32
using namespace webots;
using namespace std;

string colors[4] = {"gray","red","green","blue"};
int color;
Camera *camera;
void get_color_at(int x, int y);



int main(int argc, char **argv) {

  Robot *robot = new Robot();
  camera=robot->getCamera("color_sensor");
  camera->enable(TIME_STEP);
  
  while (robot->step(TIME_STEP) != -1) {
  
  get_color_at(32,20);
  get_color_at(32,50);
  
  
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}

void get_color_at(int x, int y){
  
  const unsigned char* image = camera->getImage();
  int image_width = camera->getWidth();
   
  int r = camera->imageGetRed(image, image_width, x, y);
  int g = camera->imageGetGreen(image, image_width, x, y);
  int b = camera->imageGetBlue(image, image_width, x, y);
  
  if(r > g && r > b) color = 1;
  if(g > r && g > b) color = 2;
  if(b > g && b > r) color = 3;
  
  cout << "Detected color = " << colors[color] << '\n';
  
}