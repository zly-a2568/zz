#include "shape.h"
#include "vec.h"
#include "array.h"
#include "node_modules/tigr.c/tigr/tigr.h"
#include "world.h"
#include "body.h"
#include <math.h>
#include <stdbool.h>
#define TIGR_IMPLEMENTATION
#include "node_modules/tigr.c/tigr.h"
#include <unistd.h>
#include "world.h"
#include <linux/input.h>
#include <fcntl.h>
#include <stdio.h>
#include <time.h>

void demo1(int argc,char* argv[]){

  struct input_event evt;

  /* ---- 打开鼠标设备（非阻塞） ---- */
  const char *mouse_path = (argc>1)?argv[1]:"/dev/input/event3"; // 按需修改
  int mouse_fd = open(mouse_path, O_RDONLY | O_NONBLOCK);
  if (mouse_fd == -1) {
    perror("open mouse");
    fprintf(stderr, "提示: 需要 root 权限，或将用户加入 input 组\n");
    // 不退出，程序仍可运行，只是无法缩放
  }

  Tigr *screen = tigrWindow(800, 1200, "physics", 0);

  float m1=20000000.0f;
  float m2=1000000.0f;
  float m3=1000.0f;
  vec2 v1={0,0};
  vec2 v2={30,140};
  vec2 v3={-90,180};
  vec2 p1={0,0};
  vec2 p2={200,0};
  vec2 p3={-200,0};
  float g=6.6743*10e-4;

  float scale=1.0f;
  const float scale_min = 0.05f;
  const float scale_max = 50.0f;
  const float scale_step = 1.1f;  // 每格滚轮的缩放倍率

  array tails;
  array_init(sizeof(vec2), &tails);
  while (!tigrClosed(screen)) {

    /* ---- 处理滚轮事件（非阻塞，读完所有待处理事件） ---- */
    if (mouse_fd != -1) {
      while (read(mouse_fd, &evt, sizeof(evt)) == sizeof(evt)) {
        if (evt.type == EV_REL && evt.code == REL_WHEEL) {
          if (evt.value > 0) {
            scale *= scale_step;   // 向上滚：放大
          } else if (evt.value < 0) {
            scale /= scale_step;   // 向下滚：缩小
          }
          if (scale < scale_min) scale = scale_min;
          if (scale > scale_max) scale = scale_max;
        }
#ifdef REL_WHEEL_HI_RES
        else if (evt.type == EV_REL && evt.code == REL_WHEEL_HI_RES) {
          // 若你的鼠标同时上报高精度事件，可选用它实现平滑缩放。
          // 这里简单按 120 为单位折算，避免与 REL_WHEEL 重复计数时可删掉上面分支。
          // scale *= powf(scale_step, evt.value / 120.0f);
        }
#endif
      }
    }

    tigrClear(screen, tigrRGB(0x00, 0x00,0x00));
    array_push(&p2, &tails);
    if(tails.size>30){
      array_remove(0, &tails);
    }
    tigrCircle(screen, 400+p1.x*scale, 600+p1.y*scale, 25*scale, tigrRGB(0xff, 0xff, 0xff));
    tigrCircle(screen, 400+p2.x*scale, 600+p2.y*scale, 5*scale,  tigrRGB(0xff, 0xff, 0xff));
    tigrCircle(screen, 400+p3.x*scale, 600+p3.y*scale, 5*scale,  tigrRGB(0xff, 0xff, 0xff));
    for(int k=0;k<tails.size;k++){
      vec2 p=*(vec2*)array_get(k, &tails);
      tigrPlot(screen, 400+p.x*scale, 600+p.y*scale, tigrRGB(0xff, 0xff, 0xff));
    }
//    printf("\n");
    float d= VEC2_LENGTH(VEC2_SUB(p1,p2));
    float da= VEC2_LENGTH(VEC2_SUB(p1,p3));
    vec2 delta_v2=VEC2_MUL(VEC2_NORMALIZE(VEC2_SUB(p1,p2)),g*m1*m2/(d*d*m2));
    vec2 delta_v3=VEC2_MUL(VEC2_NORMALIZE(VEC2_SUB(p1,p3)),g*m1*m3/(da*da*m3));
    vec2 fn=VEC2_MUL(VEC2_NORMALIZE(VEC2_SUB(p2,p3)),g*m2*m3/(da*da));
    v2=VEC2_ADD(v2,delta_v2);
    v3=VEC2_ADD(v3,delta_v3);
    v3=VEC2_ADD(v3,VEC2_MUL(fn,1/m3));
    v2=VEC2_ADD(v2,VEC2_MUL(VEC2_SUB(VEC2_ZERO, fn),1/m2));
    p2=VEC2_ADD(p2, VEC2_MUL(v2,0.01));
    p3=VEC2_ADD(p3, VEC2_MUL(v3,0.01));
    tigrUpdate(screen);
    usleep(1000);
  }
  if (mouse_fd != -1) close(mouse_fd);
  array_free(&tails);
  tigrFree(screen);
  return ;
}

void demo2(int argc,char* argv[]){
  Tigr *screen = tigrWindow(800, 1200, "physics", 0);
  world* w=create_world();
  size_t bid1 = world_create_body(w);
  size_t bid2 = world_create_body(w);
  shape shape1=default_shape;
  shape1.c.r=50;
  shape shape2=default_shape;
  shape2.c.r=100;
  world_set_body_shape(w, bid1, shape1);
  world_set_body_shape(w, bid2, shape2);
  world_set_body_position(w, bid1, -200, 40);
  world_set_body_position(w, bid2, 50, 0);
  world_set_body_velocity(w, bid1, 100, 0);
  world_set_body_mass(w, bid2, 50);
  while(!tigrClosed(screen)){
    tigrClear(screen, tigrRGB(0x00, 0x00, 0x00));
    world_update(w, 0.01);
    vec2 p1=world_get_body_position(w, bid1);
    vec2 p2=world_get_body_position(w, bid2);
    float ro1 = world_get_body_rotation(w, bid1);
    float ro2 = world_get_body_rotation(w, bid2);
    vec2 p1a=VEC2_ADD(p1, VEC2_MUL((vec2){cos(ro1),sin(ro1)},shape1.c.r));
    vec2 p2a=VEC2_ADD(p2, VEC2_MUL((vec2){cos(ro2),sin(ro2)},shape2.c.r));
    tigrCircle(screen, 400+p1.x, 600+p1.y, 50, tigrRGB(0xFF, 0xFF,0xFF));
    tigrLine(screen, 400+p1.x, 600+p1.y, 400+p1a.x, 600+p1a.y, tigrRGB(0xFF, 0xFF,0xFF));
    tigrLine(screen, 400+p2.x, 600+p2.y, 400+p2a.x, 600+p2a.y, tigrRGB(0xFF, 0xFF,0xFF));
    tigrCircle(screen, 400+p2.x, 600+p2.y, 100, tigrRGB(0xFF, 0xFF,0xFF));
    tigrUpdate(screen);
    usleep(10000);
  }
  destroy_world(w);
}
void demo3(int argc,char* argv[]){
  Tigr *screen = tigrWindow(800, 1200, "physics", 0);

  world* w=create_world();
  srand(time(NULL));

  shape shape1=default_shape;
  shape1.c.r=10;
  for(int i=0;i<100;i++){
    size_t bid = world_create_body(w);
    world_set_body_position(w, bid, rand()%800, rand()%1200);
    world_set_body_velocity(w, bid, rand()%600-300, rand()%600-300);
    world_set_body_shape(w, bid, shape1);
  }

  while(!tigrClosed(screen)){

    tigrClear(screen, tigrRGB(0x00, 0x00, 0x00));

    world_update(w, 0.01);
    for(int i=0;i<w->body_count;i++){
      body* b=array_get(i,&w->bodies);
      float ro1=b->rotation.z;
      vec2 p1=VEC2_ADD(b->position, VEC2_MUL((vec2){cos(ro1),sin(ro1)},shape1.c.r));
      if(b->position.x<0){
        b->position.x=800.0f;
      }
      if(b->position.x>800.0f){
        b->position.x=0.0f;
      }
      if(b->position.y<0){
        b->position.y=1200.0f;
      }
      if(b->position.y>1200.0f){
        b->position.y=0.0f;
      }
      tigrCircle(screen, b->position.x, b->position.y, b->shape.c.r, tigrRGB(0xFF, 0xFF,0xFF));
      tigrLine(screen,b->position.x,b->position.y,p1.x,p1.y, tigrRGB(0xFF, 0xFF,0xFF));
    }

    tigrUpdate(screen);

    usleep(10000);

 }

  destroy_world(w);

}

void demo4(int argc,char* argv[]){
    world* w=create_world();
  const float dt=0.01f;
  Tigr *screen = tigrWindow(800, 1200, "physics", 0);
  shape cs=default_shape; cs.c.r=20.0f;
  size_t c=world_create_body(w);
  world_set_body_shape(w,c,cs);
  world_set_body_position(w,c,400.0f,610.0f);
  world_set_body_velocity(w,c,0.0f,0.0f);

  size_t c1=world_create_body(w);
  world_set_body_shape(w,c1,cs);
  world_set_body_position(w,c1,400.0f,600.0f);
  world_set_body_velocity(w,c1,0.0f,0.0f);

  while(!tigrClosed(screen)){
    tigrClear(screen,tigrRGB(0x00, 0x00, 0x00));
    body* b=array_get(c,&w->bodies);
    world_update(w,dt);
    for(int i=0;i<w->body_count;i++){
      body* b=array_get(i,&w->bodies);
      float ro1=b->rotation.z;
      vec2 p1=VEC2_ADD(b->position, VEC2_MUL((vec2){cos(ro1),sin(ro1)},b->shape.c.r));
      tigrCircle(screen, b->position.x, b->position.y, b->shape.c.r, tigrRGB(0xFF, 0xFF,0xFF));
      tigrLine(screen,b->position.x,b->position.y,p1.x,p1.y, tigrRGB(0xFF, 0xFF,0xFF));
    }
    tigrUpdate(screen);
    usleep(10000);
  }
  destroy_world(w); 
}

int main(int argc,char* argv[]){
  demo2(argc, argv);
  return 0;
}
