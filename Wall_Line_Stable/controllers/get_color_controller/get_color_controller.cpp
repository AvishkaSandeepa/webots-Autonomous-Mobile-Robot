#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>


#define TIME_STEP 32
using namespace webots;
using namespace std;

string colors[4] = {"gray","red","green","blue"};
int color;


int main(int argc, char **argv) {

  Robot *robot = new Robot();
  
  Camera *camera;
  camera=robot->getCamera("color_sensor");
  camera->enable(TIME_STEP);
  void get_color();
  
  
  
  while (robot->step(TIME_STEP) != -1) {
  
  const unsigned char* image = camera->getImage();
  int image_width = camera->getWidth();
  int x = 32; int y = 32; // point of color extraction
  int r = camera->imageGetRed(image, image_width, x, y);
  int g = camera->imageGetGreen(image, image_width, x, y);
  int b = camera->imageGetBlue(image, image_width, x, y);
  // cout << "red = "<< r << "\tGreen = " << g  << "\tBlue = " << b << '\n';
  
  // color detection
  //  red 1 green 2 blue 3
  
  if(r > g && r > b) color = 1;
  if(g > r && g > b) color = 2;
  if(b > g && b > r) color = 3;
  
  cout << "Detected color = " << colors[color] << '\n';
  
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}

// void get_color(){
  // const unsigned char* image = camera->getImage();
  // int image_width = camera->getWidth();
  // int x = 32; int y = 32; 
  // int r = camera->imageGetRed(image, image_width, x, y);
  // int g = camera->imageGetGreen(image, image_width, x, y);
  // int b = camera->imageGetBlue(image, image_width, x, y);
  // cout << "red = "<< r << "\tGreen = " << g  << "\tBlue = " << b << '\n';
// }