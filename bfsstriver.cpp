#include<iostream>
#include<bits/stdc++.h>
using namespace std ;
class node{
    public :
    int u  ;
    int v ;
    int wt ;
    node(int first , int second , int weight)
    {
        u=first ;
        v=second ;
        wt=weight ;
    }
} ;
void dfs(int node , vector<int> &vis, vector<int> adj[] , vector<int> &store)
{
    store.push_back(node) ;
    vis[node]=1 ;
    for(auto it : adj[node])
    {
        if(!vis[it])
        dfs(it , vis , adj , store) ;
    }
}

vector<int>dfstraversal(int n , vector<int> adj[])
{
    vector<int> store ;
    vector<int> vis((n+1) ,0) ;
    for(int i=1 ; i<=n ; i++)
    {
        if(!vis[i])
        dfs(i , vis , adj , store);
    }
    return store ;
}


vector<int>bfsTraversal(int n , vector<int> adg[])
{
    vector<int> bfs ;
    vector<int> v((n+1) , 0) ;

    for(int i=1 ; i<=n ; i++)
    {
        if(!v[i])
        {
            queue<int> q ;
            q.push(i) ;
            v[i]=1 ;
            while(!q.empty())
            {
                int x = q.front() ;
                q.pop() ;
                bfs.push_back(x) ;
                for(auto it : adg[i])
                {
                    if(!v[it])
                    {
                        q.push(it) ;
                        v[it]=1 ;
                    }
                }
            }
        }
    }
    return bfs ;
}

bool checking(int s , int V , vector<int> adj[] , vector<int> &vis)
{
    queue<pair<int , int>> q ;
    vis[s]=1 ;
    q.push({s,-1}) ;
    while(!q.empty())
    {
        int node = q.front().first ;
        int par = q.front().second ;
        q.pop() ;
        for(auto it : adj[node])
        {
            if(!vis[it])
            {
                vis[it]=1 ;
                q.push({it , node}) ;
            }
            else if(par!=it)
            {
                return true ;
            }
        }
    }
    return false ;
}


bool checkcycle(int V , vector<int> adj[])
{
    vector<int> vis((V+1) , 0) ;
    for(int i=1 ; i<=V ; i++)
    {
        if(!vis[i])
        {
            if(checking(i , V , adj, vis))
            return true ;
        }
    }
    return false ;
}

bool checkForCycledfs(int node, int parent, vector<int> &vis, vector<int> adj[]) {
        vis[node] = 1; 
        for(auto it: adj[node]) {
            if(!vis[it]) {
                if(checkForCycledfs(it, node, vis, adj)) 
                    return true; 
            }
            else if(it!=parent) 
                return true; 
        }
        
        return false; 
    }

	bool isCycle(int V, vector<int>adj[]){
	    vector<int> vis(V+1, 0); 
	    for(int i = 1;i<=V;i++) {
	        if(!vis[i]) {
	            if(checkForCycledfs(i, -1, vis, adj)) return true; 
	        }
	    }
	    
	    return false; 
	}

    bool isBipartite(int s , vector<int> adj[] ,  vector<int> & color)
    {
        queue<int> q ;
        q.push(s) ;
        color[s]=1 ;
        while(!q.empty())
        {
            int node = q.front() ;
            q.pop() ;
            for(auto it :adj[node])
            {
                if(color[it]==-1)
                {
                    color[it]=1-color[node] ;
                    q.push(it) ;
                }
                else if(color[it]==color[node])
                {
                    return false ;
                }
            }
        }
        return true ;
    }

    bool checkbipartite(vector<int> adj[] , int V)
    {
        vector<int> color(V+1 , -1) ;
        for(int i=1 ; i<=V ; i++)
        {
            if(color[i]== -1)
            {
                if(!isBipartite(i , adj , color))
                return false ;
            }
        }
        return true ;
    }

    bool isBipartitedfs(int s , vector<int> adj[] , vector<int> &color)
    {
        if(color[s]==-1)
        {
            color[s]=1 ;
        }
        for(auto it : adj[s])
        {
            if(color[it]==-1)
            {
                color[it]=1-color[s];
                if(!isBipartitedfs(it , adj , color))
                return false ;
            }
            else if(color[it]==color[s])
            return false ;
        }
        return true ;
    }

    bool checkbipartitedfs(int V , vector<int> adj[])
    {
        vector<int> color(V+1 , -1) ;
        for(int i=1 ; i<=V ; i++)
        {
            if(color[i]==-1)
            {
                if(!isBipartitedfs(i , adj , color))
                return false ;
            }
        }
        return true ;
    }
    
    bool iscycledirect(int s , vector<int> adj[] , vector<int> &vis , vector<int> &dfsv)
    {
        vis[s]=1 ;
        dfsv[s]=1 ;
        for(auto it : adj[s])
        {
            if(!vis[it])
            {
            if(iscycledirect(it , adj , vis , dfsv))
            return true ;
            }
            else if(dfsv[it])
            return true ;
        }
        dfsv[s]=0 ;
        return false ;
    }

    bool cycleDirected(int V , vector<int> adj[])
    {
        vector<int> vis(V+1 , 0) ;
        vector<int> dfsv(V+1 , 0 );
        for(int i=1 ; i<=V ; i++)
        {
            if(!vis[i])
            {
                if(iscycledirect(i , adj , vis , dfsv))
                return true ;
            }
        }
        return false ;
    }

    void findtoposort(int s , stack<int> &st , vector<int> &vis , vector<int> adj[])
    {
        
        vis[s]=1 ;

        for(auto it : adj[s])
        {
            if(!vis[it])
            {
                findtoposort(it , st , vis , adj) ;
            }
        }
        st.push(s);
    }

    vector<int> topodfs(int V , vector<int> adj[])
    {
        vector<int> vis(V+1 , 0) ;
        stack<int> st ;
        for(int i=1 ; i<=V ; i++)
        {
            if(!vis[i])
            {
                findtoposort(i , st , vis , adj) ;
            }
        }

        vector<int>topo ;
        while(!st.empty())
        {
            topo.push_back(st.top()) ;
             st.pop() ;
        }
        return topo ;
    }

    vector<int> topobfs(int V , vector<int> adj[])
    {
        queue<int> q ;
        vector<int> indegree(V+1 , 0) ;
        for(int i=1 ; i<=V ; i++)
        {
            for(auto it : adj[i])
            {
                indegree[it]++ ;
            }
        }

        for(int i=1 ; i<=V ; i++)
        {
            if(indegree[i]==0)
            q.push(i) ;
        }

        vector<int> result ;
        while(!q.empty())
        {
            int node = q.front() ;
            q.pop() ;
            result.push_back(node) ;
            for(auto it : adj[node])
            {
                indegree[it]-- ;
                if(indegree[it]==0)
                q.push(it) ;
            }
        }
        return result ;
    }

    bool isCyclicDirBFS(int V , vector<int> adj[])
    {
        queue<int> q ;
        vector<int> indegree(V+1 , 0) ;
        for(int i=1 ; i<=V ; i++)
        {
            for(auto it : adj[i])
            {
                indegree[it]++ ;
            }
        }

        for(int i=1 ; i<=V ; i++)
        {
            if(indegree[i]==0)
            q.push(i) ;
        }

        int count = 0 ;
        while(!q.empty())
        {
            int node = q.front() ;
            q.pop() ;
            count++ ;
            for(auto it : adj[node])
            {
                indegree[it]-- ;
                if(indegree[it]==0)
                q.push(it) ;
            }
        }
        if(count==V)
        return false ;

        return true ;
    }

    void shortpathBFSun(int V , vector<int> adj[] , int s)
    {
        queue<int> q;
        vector<int> dist(V+1 , INT_MAX) ;
        dist[s]=0 ;
        q.push(s) ;
        while(!q.empty())
        {
            int node = q.front() ;
            q.pop() ;
            for(auto it : adj[node])
            {
                if(dist[node]+1 < dist[it])
                {
                    dist[it]=dist[node]+1 ;
                    q.push(it) ;
                }
            }
        }
        for(int i=1 ; i<=V ; i++)
        {
            cout<<dist[i]<<" ";
        }
        cout<<endl ;
    }

    void findtoposort2(int node , stack<int> &st , vector<int> &vis , vector<pair<int , int>> adj[])
    {
        vis[node]=1 ;
        for(auto it : adj[node])
        {
            if(!vis[it.first])
            {
                findtoposort2(it.first , st , vis , adj) ;
            }
        }
        st.push(node) ;
    }

    void shortestDAG(int s , int V , vector<pair<int , int>> adj[])
    {
        vector<int> vis(V+1 , 0) ;
        stack<int> st ;
        for(int i=1 ; i<=V ; i++)
        {
            if(!vis[i])
            findtoposort2(i , st , vis , adj) ;
        }

        vector<int>dist(V+1 , INT_MAX) ;
        dist[s]=0 ;
        while(!st.empty())
        {
            int node = st.top() ;
            st.pop() ;

            if(dist[node]!=INT_MAX){
            for(auto it : adj[node])
            {
             if(dist[node]+it.second < dist[it.first])
             {
                 dist[it.first]=dist[node]+it.second ;
             }   
            }
            }
        }
        for(int i=1 ; i<=V ; i++)
        {
            cout<<dist[i]<<" ";
        }
    }

    void djiktra(int s , int V , vector<pair<int , int>> adj[])
    {
        priority_queue<pair<int,int> , vector<pair<int,int>> , greater<pair<int,int>>> pq ;
        vector<int> dist(V+1 , INT_MAX) ;
        dist[s]=0;
        pq.push(make_pair(0 , s)) ;
        while(!pq.empty())
        {
            int dis = pq.top().first ;
            int prev = pq.top().second ;
            pq.pop() ;
            for(auto it : adj[prev])
            {
                int d = it.second ;
                int v = it.first ;
                if(dist[v]>dist[prev]+d)
                {
                    dist[v]=dist[prev]+d ;
                    pq.push({dist[v] , v}) ;
                }
            }
        }
        for(int i=1 ; i<=V ; i++)
        {
            cout<<dist[i]<<" " ;
        }
    }

    void primbrute(int V , vector<pair<int , int>> adj[])
    {
        vector<int> key(V+1 , INT_MAX) ;
        vector<bool> mvis(V+1 , false) ;
        vector<int> parent(V+1 , -1) ;
        key[1]=0 ;

        for(int count=1 ; count<=V-1 ; count++)
        {
            int mini = INT_MAX , u ;
            for(int i=1 ; i<=V ; i++)
            {
                if(mvis[i]==false && key[i]<mini)
                {
                    mini = key[i] ;
                    u=i ;
                }
            }
            mvis[u]=true ;
            for(auto it : adj[u])
            {
                int v = it.first ;
                int weight = it.second ;
                if(mvis[v]==false && weight < key[v])
                {
                    key[v]=weight ;
                    parent[v]=u ;
                }
            }
             }
             for(int i=1 ; i<=V ; i++)
            {
                cout<<parent[i]<<" ";
            }
            cout<<endl ;
            int sum =0 ;
            for(int i=1 ; i<=V ; i++)
            {
                sum=sum+key[i] ;
            }
            cout<<"sum of spanning tree "<<sum ;
    }



bool comp(node a , node b)
{
    return a.wt<b.wt ;
}

int findpar(int u , vector<int> &parent)
{
    if(u==parent[u])return u ;
    return parent[u]=findpar(parent[u] , parent) ;
}

void unionn (int u , int v , vector<int> &parent , vector<int> &rank)
{
    u=findpar(u , parent) ;
    v=findpar(v , parent) ;
    if(rank[u]<rank[v])
    {
        parent[u]=v ;
    }
    else if(rank[v]<rank[u])
    parent[v]=u ;

    else{
        parent[v]=u ;
        rank[u]++ ;
    }
}

void kruskals(int V , vector<node> edges)
{
    vector<int> parent(V+1);
    for(int i=1 ; i<=V ; i++)
    {
        parent[i]=i ;
    }
    vector<int> rank(V+1 , 0) ;

    int cost = 0;
    vector<pair<int , int>> mst ;

    for(auto it : edges)
    {
        if(findpar(it.u , parent)!=findpar(it.v , parent))
        {
            cost = cost + it.wt ;
            mst.push_back({it.u , it.v}) ;
            unionn(it.u , it.v , parent , rank) ;
        }
    }

    cout<<cost<<endl ;
    for(auto it : mst)
    {
        cout<<it.first<<" "<<it.second<<endl ;
    }
}

void dfs2(int node , int parent , vector<int> &vis , vector<int> &tin , vector<int> &low , int &timer , vector<int> adj[])
{
    vis[node]=1 ;
    tin[node]=low[node]=timer++ ;
    for(auto it : adj[node])
    {
        if(it==parent)continue ;

        if(!vis[it])
        {
            dfs2(it , node , vis , tin , low , timer , adj) ;
            low[node]=min(low[node] , low[it]) ;
            if(low[it]>tin[node])
            {
                cout<<node<<" "<<it<<endl ;
            }
        }
        else{
            low[node]=min(tin[it] , low[node]) ;
        }
    }
}

void bridge(int V , vector<int> adj[])
{
    vector<int> tin(V+1 , -1) ;
    vector<int> low(V+1 , -1) ;
    vector<int> vis(V+1 , 0) ;
    int timer = 1 ;
    for(int i=1 ; i<=V ; i++)
    {
        if(!vis[i])
        {
            dfs2(i , -1 , vis , tin , low , timer , adj) ;
        }
    }
}
void dfs3(int node , int parent , vector<int> &vis , vector<int> &tin , vector<int> &low , int &timer , vector<int> &articulate , vector<int> adj[])
{
    vis[node]=1 ;
    tin[node]=low[node]=timer++ ;
    int count = 0 ;
    for(auto it : adj[node])
    {
        if(it==parent)
        continue ;

        if(!vis[it])
        {
            dfs3(it , node , vis , tin , low , timer , articulate , adj) ;
            low[node]=min(low[it] , low[node]) ;
            if(low[it]>=tin[node] && parent!=-1)
            articulate[node]=1 ;

            count++ ;
        }
        else{
            low[node]=min(tin[it] , low[node]) ;
        }
    }
    if(parent== -1 && count>1)
    {
        articulate[node]=1 ;
    }
}

void articulation(int V , vector<int> adj[])
{
    vector<int> tin(V+1, -1) ;
    vector<int> low(V+1, -1) ;
    vector<int> vis(V+1, 0) ;
    vector<int> articulate(V+1, 0) ;
    int timer = 0 ;
    for(int i=1 ; i<=V ; i++)
    {
        if(!vis[i])
        {
            dfs3(i , -1 , vis , tin , low , timer , articulate , adj) ;
        }
    }

    for(int i=1 ; i<=V ; i++)
    {
        if(articulate[i]>0)
        cout<<i<<" " ;
    }
}
void topoforkosar(int node , vector<int> &vis , stack<int> &st , vector<int> adj[])
{
    vis[node]=1 ;
    for(auto it : adj[node])
    {
        if(!vis[it])
        topoforkosar(it , vis , st , adj) ;
    }
    st.push(node) ;
}

void revdfsforkosar(int node , vector<int> &vis , vector<int>transpose[])
{
    cout<<node<<" ";
    vis[node]=1 ;
    for(auto it : transpose[node])
    {
        if(!vis[it])
        {
            revdfsforkosar(it , vis , transpose) ;
        }
    }
}

void kosaraju(int V , vector<int> adj[])
{
    stack<int> st ;
    vector<int> vis(V+1 , 0) ;
    for(int i=1 ; i<=V ; i++)
    {
        if(!vis[i])
        {
            topoforkosar(i , vis , st , adj) ;
        }
    }
    vector<int> transpose[V+1] ;
    for(int i=1 ; i<=V ; i++)
    {
        vis[i]=0 ;
        for(auto it : adj[i])
        {
            transpose[it].push_back(i) ;
        }
    }

    while(!st.empty())
    {
        int node = st.top() ;
        st.pop() ;
        if(!vis[node])
        {
            cout<<"SCC: ";
            revdfsforkosar(node , vis , transpose) ;
            cout<<endl ;
        }
    }
}



void bellmanford(int src , int V , vector<node> edges)
{
    vector<int> dist(V+1 , INT_MAX) ;
    dist[src]=0 ;

    for(int i=1 ; i<=V-1 ; i++)
    {
       
            for(auto it : edges)
            {
                if((dist[it.u]+it.wt) < dist[it.v] )
                {
                    dist[it.v]=dist[it.u]+it.wt ;
                }
            }
        
    }
    for(int i=1 ; i<=V ; i++)
    cout<<dist[i]<<" ";
}

int main()
{
    int n , m ;
    cout<<"enter the number of vertices \n" ;
    cin>>n ;
    cout<<"enter the number of edges \n" ;
    cin>>m ;

    // vector<pair<int , int>> adg[n+1] ;

    vector<node> edges ;
    for(int i=0 ; i<m ; i++)
    {
        int u , v  , wt;
        cin>>u>>v>>wt ;
        edges.push_back(node(u , v , wt)) ;
        // adg[u].push_back({v, wt}) ;
        // adg[v].push_back(u) ;
    }
    // sort(edges.begin() , edges.end() , comp) ;
//   bool bipar ;

//   bipar = isCyclicDirBFS( n , adg ) ;

//     // vector<int> topo = topobfs(n , adg) ;
//     // for(int i=0 ; i<topo.size() ; i++)
//     // cout<<topo[i]<<" ";
//     // cout<<endl ;

//   cout<<bipar<<" ";

// kruskals(n , edges) ;
bellmanford(1 , n , edges) ;
return 0 ;
}

