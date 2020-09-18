import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.*;
import java.util.*;

//Modified implementation of Kosaraju's Algorithm:
//Purpose: Linear time algorithm to find the strongly connected components of a directed graph
public class KosarajuSCC {

  int nodeCount;
  LinkedList<Integer> adjList[];
  static int sccSize; //size of the strongly connected components
  static int nodesReached; //nodesReached are nodes reached from last traversal
  static Stack s1;
  static Stack sFinishingTime;

  //constructor method
  KosarajuSCC(int nodeCount) {
    this.nodeCount = nodeCount;
    this.adjList = new LinkedList[nodeCount];
    this.sccSize = 0; //
    this.nodesReached = 0;
    this.s1 = new Stack<Integer>();
    this.sFinishingTime = new Stack<Integer>();

    for (int i = 0; i < nodeCount; i++) {
      adjList[i] = new LinkedList<>();
    }
  }

  void setNeighbor(int u, int v) {
    adjList[u].add(v); //add a neighbor v at vertex u reference point
  }

  void littleDFS(int vertex, boolean visitCheck[], Stack s) {
    int neighborSelect;
    this.sccSize++;

    if (visitCheck[vertex] == false) {
      visitCheck[vertex] = true;
    }

    Iterator<Integer> i = adjList[vertex].iterator();
    while (i.hasNext()) {
      neighborSelect = i.next();
      if (visitCheck[neighborSelect] == false) {
        s.push(neighborSelect);
      }
    }
  }

  void littleDFS2(int v1, boolean visitCheck[]) {
    nodesReached++;
    if (visitCheck[v1] == false) {
      visitCheck[v1] = true;
      Iterator<Integer> i = adjList[v1].iterator();

      while (i.hasNext()) {
        int neighborSelect = i.next();
        if (!visitCheck[neighborSelect]) {
          s1.push(neighborSelect);
        }
      }
    }
  }


  //iterate through and reverse directed graph
  KosarajuSCC revGraph() {
    KosarajuSCC revG = new KosarajuSCC(nodeCount);

    for (int i = 0; i < nodeCount; i++) {
      Iterator<Integer> nextVertex = adjList[i].listIterator();
      while (nextVertex.hasNext()) {
        revG.adjList[nextVertex.next()].add(i);
      }
    }
    return revG;
  }


  void traverse(int v, boolean visited[]) {
    Stack s = new Stack();
    s.push(v);
    while (!s.empty()) {
      int vt = (int) s.peek();
      if (visited[vt] == false) {
        visited[vt] = true;
        Iterator<Integer> i = adjList[vt].iterator();

        while (i.hasNext()) {
          int neighborSelect = i.next();
          if (!visited[neighborSelect]) {
            s.push(neighborSelect);
          }
        }
      } else {
        sFinishingTime.push(s.pop());
      }
    }
  }


  void bigDFS() {
    KosarajuSCC normalGraph = revGraph();

    boolean visited[] = new boolean[nodeCount];
    for (int i = 0; i < nodeCount; i++) {
      visited[i] = false;
    }

    for (int i = 0; i < nodeCount; i++) {
      if (visited[i] == false) {
        traverse(i, visited);
      }
    }

    int v = (int) sFinishingTime.pop();

    Stack s = new Stack();
    s.push(v);
    this.s1.push(v);

    for (int i = 0; i < nodeCount; i++) {
      visited[i] = false;
    }

    while (!s.empty()) {
      int v2 = (int) s.pop();

      if (visited[v2] == false) {
        normalGraph.littleDFS(v2, visited, s);
      }
    }

    for (int i = 0; i < nodeCount; i++) {
      visited[i] = false;
    }
  }

  //main method
  public static void main(String args[]) throws IOException {

    //read input stream of directed graph
    BufferedReader input = new BufferedReader(new InputStreamReader(System.in));

    StringTokenizer tokenizer = new StringTokenizer(input.readLine());
    int nodeCount = Integer.parseInt(tokenizer.nextToken());
    int edgeCount = Integer.parseInt(tokenizer.nextToken());
    KosarajuSCC normGraph = new KosarajuSCC(nodeCount);

    //form adjacency list from input stream
    for (int i = 0; i < edgeCount; i++) {
      StringTokenizer tkNext = new StringTokenizer(input.readLine());
      int u = Integer.parseInt(tkNext.nextToken()) - 1;
      int v = Integer.parseInt(tkNext.nextToken()) - 1;

      normGraph.setNeighbor(v, u);
    }

    //run first iteration of depth first search
    normGraph.bigDFS();

    //maintain visited nodes list, initialize unvisited
    boolean[] visited = new boolean[nodeCount];
    for (int i = 0; i < nodeCount; i++) {
      visited[i] = false;
    }

    //run mini depth first search based on stack nodes to fully traverse
    while (s1.isEmpty() == false) {
      int v1 = (int) s1.pop();

      if (visited[v1] == false) {
        normGraph.littleDFS2(v1, visited);
      }
    }

    //output size of strongly connected components, else 0
    if (nodesReached == nodeCount) {
      System.out.println(sccSize);
    } else {
      System.out.println(0);
    }
  }
}