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

  Tigr *screen = tigrWindow(800, 900, "physics", 0);

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
    tigrCircle(screen, 400+p1.x*scale, 450+p1.y*scale, 25*scale, tigrRGB(0xff, 0xff, 0xff));
    tigrCircle(screen, 400+p2.x*scale, 450+p2.y*scale, 5*scale,  tigrRGB(0xff, 0xff, 0xff));
    tigrCircle(screen, 400+p3.x*scale, 450+p3.y*scale, 5*scale,  tigrRGB(0xff, 0xff, 0xff));
    for(int k=0;k<tails.size;k++){
      vec2 p=*(vec2*)array_get(k, &tails);
      tigrPlot(screen, 400+p.x*scale, 450+p.y*scale, tigrRGB(0xff, 0xff, 0xff));
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
  Tigr *screen = tigrWindow(800, 900, "physics", 0);
  world* w=create_world();
  size_t bid1 = world_create_body(w);
  size_t bid2 = world_create_body(w);
  shape shape1=default_shape;
  shape1.c.r=50;
  shape shape2;
  shape2.id=TYPE_SEGMENT;
  shape2.s.p1=(vec2){0.0,300.0f};
  shape2.s.p2=(vec2){0.0,-300.0f};
  world_set_body_shape(w, bid1, shape1);
  world_set_body_shape(w, bid2, shape2);
  world_set_body_position(w, bid1, 200, 400);
  world_set_body_position(w, bid2, 450, 400);
  world_set_body_velocity(w, bid1, 100, -30.0);
  world_set_body_mass(w, bid2, 5);
  //world_set_body_dynamic(w, bid2, false);
  while(!tigrClosed(screen)){
    tigrClear(screen, tigrRGB(0x00, 0x00, 0x00));
    world_update(w, 0.01);
    vec2 p1=world_get_body_position(w, bid1);
    body* sb=array_get(bid2,&w->bodies);
    float ro1 = world_get_body_rotation(w, bid1);
    vec2 p1a=VEC2_ADD(p1, VEC2_MUL((vec2){cos(ro1),sin(ro1)},shape1.c.r));
    tigrCircle(screen, p1.x, p1.y, 50, tigrRGB(0xFF, 0xFF,0xFF));
    tigrLine(screen, p1.x,p1.y,p1a.x,p1a.y, tigrRGB(0xFF, 0xFF,0xFF));
    vec2 lp1=body_to_world(sb, shape2.s.p1);
    vec2 lp2=body_to_world(sb, shape2.s.p2);
    tigrLine(screen,lp1.x,lp1.y,lp2.x,lp2.y, tigrRGB(0xFF, 0xFF, 0xFF));
    tigrUpdate(screen);
    usleep(10000);
  }
  destroy_world(w);
}
void demo3(int argc,char* argv[]){
  Tigr *screen = tigrWindow(800, 900, "physics", 0);

  world* w=create_world();
  srand(time(NULL));

  shape shape1=default_shape;
  shape1.c.r=10;
  for(int i=0;i<100;i++){
    size_t bid = world_create_body(w);
    world_set_body_position(w, bid, rand()%700+50, rand()%800+50);
    world_set_body_velocity(w, bid, rand()%600-300, rand()%600-300);
    world_set_body_shape(w, bid, shape1);
  }
  /* A closed frame just inside the window. tigrPlot skips pixels on/after the
   * last row/column, so the edges are inset by a small margin. */
  const float W=800.0f, H=900.0f, M=2.0f;
  const float x0=M, y0=M, x1=W-M, y1=H-M;
  const float hx=(x1-x0)*0.5f;      /* half width  */
  const float hy=(y1-y0)*0.5f;      /* half height */
  const float cx=(x0+x1)*0.5f, cy=(y0+y1)*0.5f;

  shape h_seg=default_shape;
  h_seg.id=TYPE_SEGMENT;
  h_seg.s.p1=(vec2){-hx,0.0f};
  h_seg.s.p2=(vec2){ hx,0.0f};

  shape v_seg=default_shape;
  v_seg.id=TYPE_SEGMENT;
  v_seg.s.p1=(vec2){0.0f,-hy};
  v_seg.s.p2=(vec2){0.0f, hy};

  size_t l1=world_create_body(w);   /* top    */
  size_t l2=world_create_body(w);   /* left   */
  size_t l3=world_create_body(w);   /* bottom */
  size_t l4=world_create_body(w);   /* right  */
  world_set_body_shape(w, l1, h_seg);
  world_set_body_shape(w, l3, h_seg);
  world_set_body_shape(w, l2, v_seg);
  world_set_body_shape(w, l4, v_seg);
  world_set_body_dynamic(w, l1, false);
  world_set_body_dynamic(w, l2, false);
  world_set_body_dynamic(w, l3, false);
  world_set_body_dynamic(w, l4, false);
  world_set_body_position(w, l1, cx, y0);
  world_set_body_position(w, l3, cx, y1);
  world_set_body_position(w, l2, x0, cy);
  world_set_body_position(w, l4, x1, cy);

  while(!tigrClosed(screen)){

    tigrClear(screen, tigrRGB(0x00, 0x00, 0x00));

    world_update(w, 0.005);
    for(int i=0;i<w->body_count;i++){
      body* b=array_get(i,&w->bodies);
      if(b->shape.id==TYPE_CIRCLE){
        float ro1=b->rotation.z;
        vec2 p1=VEC2_ADD(b->position, VEC2_MUL((vec2){cos(ro1),sin(ro1)},shape1.c.r));
        b->linear_velocity=VEC2_ADD(b->linear_velocity, (vec2){0.0,9.8f});
        if(b->position.x<0){
          b->position.x=800.0f;
        }
        if(b->position.x>800.0f){
          b->position.x=0.0f;
        }
        if(b->position.y<0){
          b->position.y=900.0f;
        }
        if(b->position.y>900.0f){
          b->position.y=0.0f;
        }
        tigrCircle(screen, b->position.x, b->position.y, b->shape.c.r, !b->sleeping?tigrRGB(0xFF, 0xFF,0xFF):tigrRGB(0x00, 0xFF, 0x00));
        tigrLine(screen,b->position.x,b->position.y,p1.x,p1.y, tigrRGB(0xFF, 0xFF,0xFF));
      }
      else if (b->shape.id==TYPE_SEGMENT) {
        vec2 lp1=body_to_world(b, b->shape.s.p1);
        vec2 lp2=body_to_world(b, b->shape.s.p2);
        tigrLine(screen, lp1.x, lp1.y, lp2.x, lp2.y, tigrRGB(0xFF, 0xFF, 0xFF));
      }
    }

    tigrUpdate(screen);

    usleep(5000);

 }

  destroy_world(w);

}

void demo4(int argc,char* argv[]){
    world* w=create_world();
  const float dt=0.01f;
  Tigr *screen = tigrWindow(800, 900, "physics", 0);
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
      if (b->shape.id==TYPE_CIRCLE){
        float ro1=b->rotation.z;
        vec2 p1=VEC2_ADD(b->position, VEC2_MUL((vec2){cos(ro1),sin(ro1)},b->shape.c.r));
        tigrCircle(screen, b->position.x, b->position.y, b->shape.c.r, tigrRGB(0xFF, 0xFF,0xFF));
        tigrLine(screen,b->position.x,b->position.y,p1.x,p1.y, tigrRGB(0xFF, 0xFF,0xFF));
      }
    }
    tigrUpdate(screen);
    usleep(10000);
  }
  destroy_world(w); 
}

/* Sleep-system showcase: three stacks of different heights rest on a static
 * floor. Each stack is its own island, so it falls asleep on its own and is
 * drawn dim. Every few seconds a ball is dropped onto one stack: that stack
 * (and only that stack) wakes up, is drawn bright, then settles and sleeps
 * again. The bar along the top fills as more of the bodies are asleep. */
void demo5(int argc,char* argv[]){
  (void)argc; (void)argv;
  Tigr *screen = tigrWindow(800, 900, "physics - sleep", 0);
  world* w=create_world();
  const float dt=0.01f;
  const float G=9.8f;
  const float floor_y=820.0f;
  const float r=16.0f;

  /* static floor */
  size_t fl=world_create_body(w);
  shape fs=default_shape;
  fs.id=TYPE_SEGMENT;
  fs.s.p1=(vec2){-350.0f,0.0f};
  fs.s.p2=(vec2){ 350.0f,0.0f};
  world_set_body_shape(w, fl, fs);
  world_set_body_dynamic(w, fl, false);
  world_set_body_position(w, fl, 400.0f, floor_y);

  /* three stacks of different heights: separate islands */
  const int   heights[3]={6,10,14};
  const float xs[3]={200.0f,400.0f,600.0f};
  for(int k=0;k<3;k++){
    for(int i=0;i<heights[k];i++){
      shape cs=default_shape;
      cs.c.r=r;
      size_t id=world_create_body(w);
      world_set_body_shape(w,id,cs);
      world_set_body_position(w,id, xs[k], floor_y - r - i*(2.0f*r+0.02f));
    }
  }

  long frame=0;
  while(!tigrClosed(screen)){
    /* gravity acts on the awake dynamic bodies only */
    for(size_t i=0;i<w->body_count;i++){
      body* b=array_get(i,&w->bodies);
      if(b->shape.id==TYPE_CIRCLE && body_is_awake(b))
        b->linear_velocity=VEC2_ADD(b->linear_velocity,(vec2){0.0f,G});
    }
    world_update(w,dt);

    /* every few seconds drop a ball onto one of the stacks to wake it */
    frame++;
    if(frame%450==0 && w->body_count<40){
      shape bs=default_shape;
      bs.c.r=r;
      size_t id=world_create_body(w);
      world_set_body_shape(w,id,bs);
      world_set_body_position(w,id, xs[(frame/450)%3], 120.0f);
    }

    /* draw: sleeping bodies are dim, awake bodies are bright */
    tigrClear(screen, tigrRGB(0x08,0x08,0x10));
    int dynamic=0, asleep=0;
    for(size_t i=0;i<w->body_count;i++){
      body* b=array_get(i,&w->bodies);
      if(b->shape.id==TYPE_CIRCLE){
        bool sleeping=world_is_body_sleeping(w,i);
        if(b->dynamic){ dynamic++; if(sleeping)asleep++; }
        TPixel col = sleeping ? tigrRGB(0x2E,0x4A,0x6E) : tigrRGB(0xF0,0xF0,0xF0);
        tigrFillCircle(screen,(int)b->position.x,(int)b->position.y,(int)b->shape.c.r,col);
        tigrCircle(screen,(int)b->position.x,(int)b->position.y,(int)b->shape.c.r,tigrRGB(0x00,0x00,0x00));
      }else if(b->shape.id==TYPE_SEGMENT){
        vec2 p1=body_to_world(b,b->shape.s.p1);
        vec2 p2=body_to_world(b,b->shape.s.p2);
        tigrLine(screen,(int)p1.x,(int)p1.y,(int)p2.x,(int)p2.y,tigrRGB(0x70,0x70,0x70));
      }
    }
    /* bar at the top: fraction of dynamic bodies that are asleep */
    float frac = dynamic ? (float)asleep/(float)dynamic : 1.0f;
    tigrRect(screen, 40, 30, 720, 18, tigrRGB(0x40,0x40,0x40));
    tigrFillRect(screen, 41, 31, (int)(718.0f*frac), 16, tigrRGB(0x2E,0x4A,0x6E));

    tigrUpdate(screen);
    usleep(10000);
  }
  destroy_world(w);
}

int main(int argc,char* argv[]){
  demo3(argc, argv);
  return 0;
}
