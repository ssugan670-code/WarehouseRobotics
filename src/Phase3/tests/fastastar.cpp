#include <cstdio>
#include <queue>
#include <vector>
#include <cmath>
#include <cstdlib>
int main(int argc,char**argv){
  FILE*f=fopen(argv[1],"rb"); int h[6]; if(fread(h,4,6,f)!=6) return 2; int ny=h[0],nx=h[1],si=h[2]-1,sj=h[3]-1,gi=h[4]-1,gj=h[5]-1;
  std::vector<unsigned char> B(ny*nx); if(fread(B.data(),1,ny*nx,f)!=(size_t)(ny*nx)) return 2; fclose(f);
  std::vector<double> g(ny*nx,1e18); std::vector<int> par(ny*nx,-1); std::vector<char> closed(ny*nx,0);
  typedef std::pair<double,int> E; std::priority_queue<E,std::vector<E>,std::greater<E> > pq;
  int s=si+sj*ny,t=gi+gj*ny; g[s]=0; pq.push(E(0,s)); bool found=false; int di[4]={1,-1,0,0},dj[4]={0,0,1,-1};
  while(!pq.empty()){E e=pq.top();pq.pop();int c=e.second;if(closed[c])continue;closed[c]=1;if(c==t){found=true;break;}
    int ci=c%ny,cj=c/ny;
    for(int d=0;d<4;d++){int ni=ci+di[d],nj=cj+dj[d];if(ni<0||ni>=ny||nj<0||nj>=nx)continue;int n=ni+nj*ny;if(B[n]||closed[n])continue;
      double ng=g[c]+1;if(ng<g[n]){g[n]=ng;par[n]=c;pq.push(E(ng+1.0001*(std::abs(ni-gi)+std::abs(nj-gj)),n));}}}
  FILE*o=fopen(argv[2],"w"); if(found){std::vector<int>p;int c=t;while(c!=-1){p.push_back(c);c=par[c];}
    for(int k=(int)p.size()-1;k>=0;k--) fprintf(o,"%d %d\n",p[k]%ny+1,p[k]/ny+1);} fclose(o); return 0;}
