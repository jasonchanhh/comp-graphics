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

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 128
#define FULLSCREEN_MODE false
#define PI 3.14159265358979323846

struct Intersection
{
  vec4 position;
  float distance;
  int triangleIndex;
};

vector<Triangle> allTriangles;
vec4 cameraPos;
float yaw = 0;
mat4 R;
vec4 lightPos(0, -0.5, -0.7, 1.0);


/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

bool Update();
void Draw(screen* screen);
bool ClosestIntersection(vec4 start, vec4 dir, const vector<Triangle>& triangles, Intersection& closestIntersection);
vec3 DirectLight( const Intersection& i);


int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  LoadTestModel(allTriangles);
  cameraPos = vec4(0.0,0.0,-3.0,1.0);

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
      float focalLength = SCREEN_HEIGHT;
      vec4 d = vec4(x - SCREEN_WIDTH/2, y - SCREEN_HEIGHT/2, focalLength, 1.0);
      vec4 d_normalize = glm::normalize(d);
      vec4 d_rotated = R * d_normalize;

      Intersection closest;
      if (ClosestIntersection(cameraPos, d_rotated, allTriangles, closest)) {
        vec3 direct = DirectLight(closest);
        vec3 color = allTriangles[closest.triangleIndex].color;
        vec3 reflected = color * ((direct) + 0.5f * vec3(1, 1, 1));
        PutPixelSDL(screen, x, y, reflected);
      }
      else {
        PutPixelSDL(screen, x, y, vec3(0,0,0));
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
        case SDLK_w:
        lightPos.z += 0.1;
    break;
        case SDLK_s:
        lightPos.z -= 0.1;
    break;
        case SDLK_a:
        lightPos.x -= 0.1;
    break;
        case SDLK_d:
        lightPos.x += 0.1;
    break;
        case SDLK_q:
        lightPos.y -= 0.1;
    break;
        case SDLK_e:
        lightPos.y += 0.1;
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
    vec3 d = vec3(dir.x, dir.y, dir.z);
    vec3 b = vec3(start.x-v0.x,start.y-v0.y,start.z-v0.z);
    // mat3 A( vec3(-dir), e1, e2 );
    // if (glm::determinant(A) != 0) {
      // vec3 x = glm::inverse( A ) * b;
      vec3 p = glm::cross(d, e2);
      float det = glm::dot(p, e1);
      // if the determinant is negative then the triangle is facing backwards
      // if (det <= 0.0) continue;

      // float u = x.y;
      // float v = x.z;
      // float t = x.x;

      float u = (glm::dot(p, b)) / det;
      if (u >= 0.0) {
        vec3 q = glm::cross(b, e1);
        float v = (glm::dot(q, d)) / det;
        if (v >= 0.0 && (u + v <= 1.0)) {
          float t = (glm::dot(q, e2)) / det;
          if (t >= 0.0f) {
      // if ((u >= 0.0) && (v >= 0.0) && (u + v <= 1.0) && (t >= 0.0)) {
          // cout << "(" << t << "," << u << "," << v << "), ";
            if (min > t) {
              min = t;
              closestIntersection.distance = t;
              closestIntersection.position = start + t*dir;
              closestIntersection.triangleIndex = i;
            }
        // }
          }
        }
      }
    }
  // }
  return min != std::numeric_limits<float>::max();
}

vec3 DirectLight( const Intersection& i) {
    vec3 lightColor = vec3(1.0,1.0,1.0) * 14.0f;
    vec3 normal = vec3(allTriangles[i.triangleIndex].normal);
    vec3 r = vec3(lightPos - i.position);

    float lightDistance = glm::length(r);
    float projection = glm::dot(normal, glm::normalize(r));
    if (projection < 0.0) projection = 0.0;

    // vec4 r_normal = vec4(glm::normalize(r), 1.0);

    // check if the pixel is in shadow
    Intersection closestShadow;
    vec4 dir = lightPos - i.position;
    vec4 dir_n = glm::normalize(dir);
    vec4 hitBias = i.position + dir_n * 1e-3f;

    // float biasDistance = glm::length(dir);

    if (ClosestIntersection(hitBias, dir, allTriangles, closestShadow)) {
      if (closestShadow.distance < 1) {
          // cout << "(" << closestShadow.distance << "," << biasDistance << "), ";
          projection = 0.0;
      }
    }

    vec3 D = lightColor * float(projection / (4.0*PI*lightDistance*lightDistance));
    return D;
}
