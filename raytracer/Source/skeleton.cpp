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

#define SCREEN_WIDTH 32
#define SCREEN_HEIGHT 32
#define FULLSCREEN_MODE false

struct Intersection
{
  vec4 position;
  float distance;
  int triangleIndex;
};

vector<Triangle> triangles;
vec4 cameraPos;
float yaw = 0;
mat4 R;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

bool Update();
void Draw(screen* screen);
bool ClosestIntersection(vec4 start, vec4 dir, const vector<Triangle>& triangles, Intersection& closestIntersection);
void normalise(vec3& new_vec);



int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  LoadTestModel(triangles);
  cameraPos = vec4(0.0,0.0,-2.0,1.0);

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
      float focalLength = SCREEN_HEIGHT/2;
      vec4 d = vec4(x - SCREEN_WIDTH/2, y - SCREEN_HEIGHT/2, focalLength, 1.0);
      vec4 d_rotated = R * d;
      // normalise(d);
      // vec4 d_normalized = vec4(d, 1.0);
      // std::cout << " (" << d_normalized.x << "," << d_normalized.y << "," << d_normalized.z << "," << d_normalized.w << ") ";
      Intersection closest;
      if (ClosestIntersection(cameraPos, d_rotated, triangles, closest) == true) {
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
        cameraPos = vec4(cameraPos.x, cameraPos.y, cameraPos.z + 0.1, 1.0);
		break;
	      case SDLK_DOWN:
		/* Move camera backwards */
        cameraPos = vec4(cameraPos.x, cameraPos.y, cameraPos.z - 0.1, 1.0);
		break;
	      case SDLK_LEFT:
		/* Move camera left */
      // cameraPos = vec4(cameraPos.x - 0.1, cameraPos.y, cameraPos.z, 1.0);
      yaw += 0.1;
      R = mat4(vec4(cos(yaw),0,sin(yaw),0.0), vec4(0,1,0,0), vec4(-sin(yaw),0,cos(yaw),0), vec4(0,0,0,1));
    break;
	      case SDLK_RIGHT:
		/* Move camera right */
      // cameraPos = vec4(cameraPos.x + 0.1, cameraPos.y, cameraPos.z, 1.0);
      yaw -= 0.1;
      R = mat4(vec4(cos(yaw),0,sin(yaw),0.0), vec4(0,1,0,0), vec4(-sin(yaw),0,cos(yaw),0), vec4(0,0,0,1));
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
  float min = std::numeric_limits<float>::max();
  for (int i = 0; i < triangles.size(); ++i) {
    // find intersection between ray and triangle
    Triangle triangle = triangles[i];
    vec4 v0 = triangle.v0;
    vec4 v1 = triangle.v1;
    vec4 v2 = triangle.v2;
    vec3 e1 = vec3(v1.x-v0.x,v1.y-v0.y,v1.z-v0.z);
    vec3 e2 = vec3(v2.x-v0.x,v2.y-v0.y,v2.z-v0.z);
    vec3 d = vec3(-1 * dir.x, -1 * dir.y, -1 * dir.z);
    vec3 b = vec3(start.x-v0.x,start.y-v0.y,start.z-v0.z);
    mat3 A( d, e1, e2 );
    vec3 x = glm::inverse( A ) * b;
    // vec3 p = glm::cross(d, e2);
    // float det = glm::dot(p, e1);
    // if the determinant is negative then the triangle is facing backwards
    // if (det <= 0.0) continue;

    float u = x.y;
    float v = x.z;
    float t = x.x;

    // float u = (glm::dot(p, b)) / det;
    // if (u >= 0.0) {
    // vec3 q = glm::cross(b, e1);
    // float v = (glm::dot(q, d)) / det;
      // if (v < 0.0 || u + v > 1.0) {
    // float t = (glm::dot(q, e2)) / det;
        // if (t >= 0.0) {
    if ((u >= 0.0) && (v >= 0.0) && (u + v <= 1.0) && (t >= 0.0)) {
        // cout << "(" << t << "," << u << "," << v << "), ";
        if (min > t) {
          min = t;
          closestIntersection.distance = t;
          closestIntersection.position = vec4(vec3(v0) + (u * e1) + (v * e2), 1.0);
          closestIntersection.triangleIndex = i;
        }
      }
        // }
      // }
    // }
  }
  if (closestIntersection.distance <= min) return true;
  else return false;
}

void normalise(vec3& new_vec) {
  new_vec = new_vec / sqrt(new_vec.x * new_vec.x + new_vec.y * new_vec.y + new_vec.z * new_vec.z);
}
