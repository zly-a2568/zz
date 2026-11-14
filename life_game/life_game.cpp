#include <locale.h>
#include <ncurses.h>
#include <set>
#include <ctime>
#include <cstdlib>
#include <unistd.h>

struct Vec2{ 
  int x;
  int y;
};
bool operator<(const Vec2 a,const Vec2 b){
  if (a.x != b.x) return a.x < b.x;
  return a.y < b.y;
}

using AliveSet = std::set<Vec2>;


void update_grid(AliveSet& grid,int row,int col){
  int dx[]={-1,0,1,-1,1,-1,0,1};
  int dy[]={-1,-1,-1,0,0,1,1,1};

  AliveSet filter1;
  for(const auto block:grid){
    for(int i=0;i<8;i++){
      int x=(block.x+dx[i]+row)%row,y=(block.y+dy[i]+col)%col;
      filter1.insert(Vec2{x,y});
    }
    filter1.insert(block);
  }
  
  AliveSet next;
  for(const auto block:filter1){
    int neighbor_count=0;
    for(int i=0;i<8;i++){
      int nx = (block.x + dx[i] + row) % row;   
      int ny = (block.y + dy[i] + col) % col;  
      if(grid.count(Vec2{nx,ny})) neighbor_count++;
    }
    bool alive=grid.count(block);
    if(alive){
      if(neighbor_count==2||neighbor_count==3){
        next.insert(block);
      }
    }else{
      if(neighbor_count==3)next.insert(block);
    }
  }
  grid = next;

}

void draw(AliveSet& grid,int row,int col){
  erase();
 // 边框
    for (int j = 0; j < row + 2; ++j) {
        mvaddch(0, j, ACS_HLINE);
        mvaddch(col + 1, j, ACS_HLINE);
    }
    mvaddch(0, 0, ACS_ULCORNER);
    mvaddch(0, row + 1, ACS_URCORNER);
    mvaddch(col + 1, 0, ACS_LLCORNER);
    mvaddch(col + 1, row + 1, ACS_LRCORNER);
    for (int i = 0; i < col; ++i) {
        mvaddch(i + 1, 0, ACS_VLINE);
        mvaddch(i + 1, row + 1, ACS_VLINE);
    }
  for(auto& p:grid){
    //mvaddstr(p.y, p.x,"■");
    mvaddch(p.y,p.x,'#');
  }
}

int main(){
  setlocale(LC_ALL, ""); 
  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr,TRUE);
  nodelay(stdscr,TRUE);
  int max_y,max_x;
  getmaxyx(stdscr, max_y, max_x);
  int rows=max_x -2;
  int cols=max_y -4;
  // init grid
  AliveSet g;
  int cx=rows/2-1;
  int cy=cols/2-1; 
  /*g.insert(Vec2{cx+1,cy});
  g.insert(Vec2{cx+2,cy+1});
  g.insert(Vec2{cx,cy+2});
  g.insert(Vec2{cx+1,cy+2});
  g.insert(Vec2{cx+2,cy+2});
  */
  srand(time(NULL));
  for(int i=0;i<rows;i++){
    for(int j=0;j<cols;j++){
      if(rand()%100>50){
        g.insert(Vec2{i,j});
      }
    }
  }
  while (true) {
    draw(g,rows,cols);
    if(getch()=='q')break;
    update_grid(g,rows,cols);
    usleep(15000);
  
  }
  endwin();
  return 0;

}
