#include<iostream>
#include<vector>
#include<stdlib.h>
#include<stdio.h>
#include<utility>
#include<queue>

using namespace std;

void Johnson(int,vector<pair<int,int>>[]);
vector<int> BellmanFord(int,vector<pair<int,int>>[]);
vector<vector<int>> Dijkstra(int,vector<pair<int,int>>[]);

class Graph{

   public:

    int V;
    vector<pair<int,int>> *adj;  

    Graph(int num)
    {
        V=num;
        adj=new vector<pair<int,int>>[V+1];
    }

    void addEdge(int x,int y,int wt)
    {
        adj[x].push_back({y,wt});
    }

       void printGraph()
   {
    for (int v = 0; v < V; v++)
      {
          cout << "\n Adjacency list of vertex "
               << v << "\n head->";
          for (auto x : adj[v])
             cout <<"("<<x.first<<","<<x.second<<")";
          cout<<"\n";
      }  
   }
};

// Adj has V+1 rows. 

int main()
{
     Graph g(4);
     g.addEdge(0,1,2);
     g.addEdge(1,0,1);
     g.addEdge(0,3,5);
     g.addEdge(3,0,2);
     g.addEdge(1,2,1);
     g.addEdge(2,0,-2);
     g.addEdge(2,3,-2);
      
  Johnson(g.V,g.adj);
}

void Johnson(int V,vector<pair<int,int>> adj[])
{
    
    //---------- 1. Add s with 0 edge weights.-------------//
    for(int i=0;i<V;i++)
    {
        adj[V].push_back({i,0});
    }

  //------ 2. Find h(v) for each V.
  vector<int> h=BellmanFord(V,adj);

  //------ 3. Remove s(along with its edges) from adj list and also delete h(V).
  auto it= h.end()-1;
  h.erase(it); 
  adj[V].clear();

  //-----4. Reweight-----//
   for(int u=0;u<V;u++)
   {
      for(int v=0;v<adj[u].size();v++){
          
          adj[u][v].second=adj[u][v].second+h[u]-h[adj[u][v].first];
      }  
   }
   
   //----- 5.Dijk--------//
   vector<vector<int>> SP=Dijkstra(V,adj);

   //-----6. Final Answer(Original weights back)--------//

   for(int u=0;u<V;u++)
   {
       for(int v=0;v<V;v++)
       {
           SP[u][v]=SP[u][v]+h[v]-h[u];
       }
   }
 //-----7. Display------//
for(int u=0;u<V;u++)
   {
       for(int v=0;v<V;v++)
       {
           cout<<SP[u][v]<<" ";
       }
       cout<<endl;
   }

}

vector<int> BellmanFord(int V,vector<pair<int,int>> adj[])
{
    vector<int> dist(V+1,INT32_MAX);
    dist[V]=0;

    for(int i=0;i<V;i++)
    {
        for(int u=V;u>=0;u--)
        {
            for(auto x:adj[u])
            {
                int v=x.first;
                int wt=x.second;

                if(dist[u]!=INT32_MAX && dist[u]+wt<dist[v])
                dist[v]=dist[u]+wt;
            }
        }
    }

   for(int u=V;u>=0;u--)
   {
       for(auto x:adj[u])
            {
                int v=x.first;
                int wt=x.second;

                if(dist[u]!=INT32_MAX && dist[u]+wt<dist[v])
                {
                    cout<<"Negative Cycle";
                    exit(1);
                }
            }
   }
    return dist;
}

vector<vector<int>> Dijkstra(int V, vector<pair<int,int>> adj[])
{
    vector<vector<int>> dist(V,vector<int>(V,INT32_MAX));
    priority_queue<pair<int,int>,vector<pair<int,int>>,greater<pair<int,int>>> q;

    for(int i=0;i<V;i++)
 {
      int src=i;
      dist[i][i]=0;
      q.push({0,src});

    while(!q.empty())
    {
        int node=q.top().second;
        int currDist=q.top().first;
        q.pop();
        if(dist[i][node]<currDist)
        continue;
        for(auto x:adj[node])
        {
            int ver=x.first;
            int wt=x.second;
            if(wt+dist[i][node]<dist[i][ver])
            {
                dist[i][ver]=wt+dist[i][node];
                q.push({dist[i][ver],ver});
                
            }
        } 
    }
  }
  return dist;
}




