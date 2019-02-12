#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <limits>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

SDL_Event event;

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

struct Intersection
{
  vec4 position;
  float distance;
  int triangleIndex;
};

vector<Triangle> triangles;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

bool Update();
void Draw(screen* screen);
bool ClosestIntersection(vec4 start, vec4 dir, const vector<Triangle>& triangles, Intersection& closestIntersection);


int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  LoadTestModel(triangles);

  while ( Update())
    {
      Draw(screen);
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
  memset(screen->buffer, 0, SCREEN_WIDTH*SCREEN_HEIGHT*sizeof(uint32_t));

  for (int y = 0; y < SCREEN_HEIGHT; ++y) {
    for (int x = 0; x < SCREEN_WIDTH; ++x) {
      float focalLength = 1.0;
      vec4 d = vec4(x - SCREEN_WIDTH/2, y - SCREEN_HEIGHT/2, focalLength, 1.0);
      vec4 cameraPos(0.0,0.0,0.0,1.0);
      Intersection closest;
      if (ClosestIntersection(cameraPos, d, triangles, closest) == true) {
        vec3 color = triangles[closest.triangleIndex].color;
        PutPixelSDL(screen, x, y, color);
      }
    }
  }
}

/*Place updates of parameters here*/
bool Update()
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;

  SDL_Event e;
  while(SDL_PollEvent(&e))
    {
      if (e.type == SDL_QUIT)
	{
	  return false;
	}
      else
	if (e.type == SDL_KEYDOWN)
	  {
	    int key_code = e.key.keysym.sym;
	    switch(key_code)
	      {
	      case SDLK_UP:
		/* Move camera forward */
		break;
	      case SDLK_DOWN:
		/* Move camera backwards */
		break;
	      case SDLK_LEFT:
		/* Move camera left */
		break;
	      case SDLK_RIGHT:
		/* Move camera right */
		break;
	      case SDLK_ESCAPE:
		/* Move camera quit */
		return false;
	      }
	  }
    }
  return true;
}

bool ClosestIntersection(vec4 start, vec4 dir, const vector<Triangle>& triangles, Intersection& closestIntersection) {
  float m = std::numeric_limits<float>::max();
  closestIntersection.distance = m;
  for (int i = 0; i < triangles.size(); ++i) {
    // find intersection between ray and triangle
    Triangle triangle = triangles[i];
    vec4 v0 = triangle.v0;
    vec4 v1 = triangle.v1;
    vec4 v2 = triangle.v2;
    vec3 e1 = vec3(v1.x-v0.x,v1.y-v0.y,v1.z-v0.z);
    vec3 e2 = vec3(v2.x-v0.x,v2.y-v0.y,v2.z-v0.z);
    vec3 b = vec3(start.x-v0.x,start.y-v0.y,start.z-v0.z);
    vec3 d = vec3(-1 * dir.x, -1 * dir.y, -1 * dir.z);
    mat3 A( d, e1, e2 );
    vec3 x = glm::inverse( A ) * b;
    // x = (t,u,v)
    if ((x.x >= 0) && (x.y > 0) && (x.z > 0) && (x.y + x.z < 1)) {
      if (closestIntersection.distance > x.x) {
        closestIntersection.distance = x.x;
        // vec4 E1 = vec4(v1.x-v0.x,v1.y-v0.y,v1.z-v0.z,1);
        // vec4 E2 = vec4(v2.x-v0.x,v2.y-v0.y,v2.z-v0.z,1);
        closestIntersection.position = v0 + vec4(x.y * e1, 1.0) + vec4(x.z * e2, 1.0);
        closestIntersection.triangleIndex = i;
      }
    }
  }
  if (closestIntersection.distance < m) return true;
  else return false;
}
