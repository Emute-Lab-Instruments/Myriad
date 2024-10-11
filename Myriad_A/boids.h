#pragma once
#include "drawing.h"
#include <TFT_eSPI.h>
#include <vector>


struct boid {
  float px;
  float py;
  float x;
  float y;

  float vx;
  float vy;
};

class boidsSim {
public:
  void init() {
    boids = std::vector<boid>(6);
    for(auto &v: boids) {
      v.x = random(120,140);
      v.y = random(120,140);
      v.px=v.x;
      v.py=v.y;
      v.vx = (random(100) / 200.0) - 0.5;
      v.vy = (random(100) / 200.0) - 0.5;
    }
    centerX = 120;
    centerY = 120;
  }

  float distBetween(float x1, float y1, float x2, float y2) {
    float dx = x2-x1;
    float dy = y2-y1;
    return sqrt((dx * dx) + (dy * dy));
  }

  void update() {
    centerX = 0;
    centerY = 0;
    for(auto &v: boids) {
      centerX += v.x;
      centerY += v.y;
    }
    centerX *= (1/6.0);
    centerY *= (1/6.0);

    for(auto &v: boids) {
      v.px = v.x;
      v.py = v.y;

      //rule 1 move towards center of mass
      float dcxr1 = (centerX - v.x) * (1/200.0);
      float dcyr1 = (centerX - v.y) * (1/200.0);
      
      //rule 2 keep away from other boids
      float dcxr2=0;
      float dcyr2=0;
      // for(auto &otherBoid: boids) {
      //   if (&v != &otherBoid) {
      //     float dist = distBetween(v.x, v.y, otherBoid.x, otherBoid.y);
      //     if (dist < 30.0) {
      //       dcxr2 = dcxr2 - (v.x - otherBoid.x);
      //       dcyr2 = dcyr2 - (v.y - otherBoid.y);
      //     }
      //   }
      // }
      // dcxr2 *= 0.01;
      // dcyr2 *= 0.01;

      v.vx += (v.vx + dcxr1 + dcxr2);
      v.vy += (v.vy + dcyr1 + dcyr2);

      v.x += v.vy;
      v.y += v.vy;

      //wrapping
      if (v.x > 240) {
        v.x-=240;
      }else if (v.x < 0) {
        v.x += 240;
      }

      if (v.y > 240) {
        v.y-=240;
      }else if (v.y < 0) {
        v.y += 240;
      }
    }
    
  }

  void draw(TFT_eSPI &tft) {
    for(auto &v: boids) {
      tft.drawCircle(v.px, v.py, 10, ELI_BLUE);
      tft.drawCircle(v.x, v.y, 10, ELI_PINK);
    }
  }

private:
  std::vector<boid> boids;
  float centerX;
  float centerY;

};