#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModel.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::mat3;

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false


/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                            */
int t;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
void Interpolate(vec3 a, vec3 b, vector<vec3>& result);

int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  t = SDL_GetTicks();	/*Set start value for timer.*/

  // Test interpolate function
/*  vector<vec3> result( 4 );
  vec3 a(1,4,9.2);
  vec3 b(4,1,9.8);
  Interpolate( a, b, result );
  for( int i=0; i<result.size(); ++i )
  {
    cout  << "( "
          << result[i].x << ", "
          << result[i].y << ", "
          << result[i].z << " ) ";
  }
*/

  while( NoQuitMessageSDL() )
    {
      Draw(screen);
      Update();
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  // Define colours
  vec3 topLeft(1,0,0);      // red
  vec3 topRight(0,0,1);     // blue
  vec3 bottomRight(0,1,0);  // green
  vec3 bottomLeft(1,1,0);   // yellow

  // Interpolate left and right edges
  vector<vec3> leftSide(SCREEN_HEIGHT);
  vector<vec3> rightSide(SCREEN_HEIGHT);
  Interpolate(topLeft, bottomLeft, leftSide);
  Interpolate(topRight, bottomRight, rightSide);

  for(int i=0; i<SCREEN_HEIGHT; i++)
    {
      // Interpolate each row with left and right edges
      vector<vec3> row(SCREEN_WIDTH);
      Interpolate(leftSide[i], rightSide[i], row);
      for(int j=0; j<SCREEN_WIDTH; j++)
      {
        PutPixelSDL(screen, j, i, row[j]);
      }
    }
}

/*Place updates of parameters here*/
void Update()
{
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  // std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/
}

void Interpolate(vec3 a, vec3 b, vector<vec3>& result)
{
  if (result.size() > 1) {
    float step_x = (b.x - a.x) / (result.size() - 1);
    float step_y = (b.y - a.y) / (result.size() - 1);
    float step_z = (b.z - a.z) / (result.size() - 1);
    result[0].x = a.x;
    result[0].y = a.y;
    result[0].z = a.z;
    for(int i=1; i<result.size(); ++i) {
      result[i].x = result[i-1].x + step_x;
      result[i].y = result[i-1].y + step_y;
      result[i].z = result[i-1].z + step_z;
    }
  }

}
