// Useful to sort lists by a custom key
import java.util.*;

float distanceBetween(PVector A, PVector B)
{
  return sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));
}

class Node
{
  String id;
  ArrayList<Wall> polygon = new ArrayList<Wall>();
  ArrayList<Integer> indices = new ArrayList<Integer>();
  PVector center;
  ArrayList<Node> neighbors = new ArrayList<Node>();
  
  //possibly unnecessary
  ArrayList<Wall> connections = new ArrayList<Wall>();
  
  Node(String id, ArrayList<Wall> polygon)
  {
    this.id = id;
    this.polygon = polygon;
    center = getCenter();
  }
  
  PVector getCenter()
  {
    int avgX = 0;
    int avgY = 0;
    
    for(Wall w: polygon) {
      avgX += w.start.x;
      avgY += w.start.y;
    }
    
    avgX/=polygon.size();
    avgY/=polygon.size();
    return new PVector(avgX, avgY);
  }
  
  boolean isNeighbor(Node n)
  {
     int prev = indices.get(indices.size()-1);
     for(Integer i: indices)
     {
       if (n.indices.contains(prev) && n.indices.contains(i)) {
         return true;
       }
       prev = i;
     }
     
     return false;
  }
  
  void addAdjacency(Node n)
  {
    neighbors.add(n);
  }
  
  boolean pointInNode(PVector p)
  {
    return isPointInPolygon(p, polygon);
  }
}


class FrontierEntry{
  Node node;
  FrontierEntry previousFrontier;
  float distanceToEnd;
  float distanceToLast = 0;
  
  FrontierEntry(Node n, FrontierEntry from, PVector end)
  {
    this.node = n;
    this.distanceToEnd = distanceBetween(n.center, end);
    if (from != null)
    {
       this.previousFrontier = from;
       this.distanceToLast = distanceBetween(n.center, from.node.center) + from.distanceToLast;
    }
    
    println("SEARCH FRONTIER FOR " + node.id);
    println("  distance to end: " + distanceToEnd);
    println("  distance to last: " + distanceToLast);
  }
  
  float getHeuristicSum()
  {
    return distanceToEnd + distanceToLast;
  }
}

//only works when outside the class for some reason
//getting heuristic value
float getH(PVector current, PVector destination)
{
  return sqrt(pow((destination.x - current.x), 2) + pow((destination.y - current.y), 2));
}

//getting distance from start
float getDist(PVector start, PVector current)
{
  return (sqrt(pow((start.x - current.x), 2) + pow((start.y - current.y), 2)));
}

class NavMesh
{
  ArrayList<Node> nodes = new ArrayList<Node>();
  int recursionDepth = 0;
  int maxDepth = 1000;
  int pointAmount = 0;
  
  HashMap<PVector, Integer> vertexLookUp = new HashMap<PVector, Integer>();
  ArrayList<PVector> mapVectors = new ArrayList<PVector>();
  

  PVector midPointBetweenNeighbors(Node a, Node b)
  {
     int start = 0, end = 0;
    
     int prev = a.indices.get(a.indices.size()-1);
     for(Integer i: a.indices)
     {
       if (b.indices.contains(prev) && b.indices.contains(i)) {
         start = prev;
         end = i;
         break;
       }
       prev = i;
     }
     println(a.id + " and " + b.id + " share indices " + start + " and " + end);
     
     //we now have wallStart and wallEnd
     PVector startP, endP;
     startP = mapVectors.get(start);
     endP = mapVectors.get(end);
     
     //needs the map's coordinates in order to link up
     
     return new PVector(startP.x + (endP.x - startP.x)/2, startP.y + (endP.y - startP.y)/2);
  }

  void setIndicesOf(Node node)
  {
     for(Wall w: node.polygon)
     {
         node.indices.add(vertexLookUp.get(w.start));
     }
  }
  
  void printOutNodeList()
  {
    for (Node n: nodes)
    {
      print(n.id);
      print(" " + n.polygon.size() + " ");
      println(n.indices);
    }
  }
  
  void calculateAdjacencies()
  {
    for (Node n: nodes)
    {
      n.neighbors.clear();
    }
    
    for (Node a: nodes)
    {      
      //this is terrible for efficiency i'm so sorry
      for (Node b: nodes)
      {
         if (b.equals(a)) continue;
         if (a.isNeighbor(b)) a.addAdjacency(b);
      }
    }
  }
  
  void printAdjacencies()
  {
    for (Node n: nodes)
    {
      println("Node " + n.id + " neighbors:");
      for (Node neighbor: n.neighbors)
      {
        println("     " + neighbor.id);
      }
    }
  }

  //assume indexA < indexB
  void splitPolygon(Node node, int indexA, int indexB, int convexPoint)
  {
    //first, create 2 polygons to represent our splitted polygons
    ArrayList<Wall> polyA = new ArrayList<Wall>();
    ArrayList<Wall> polyB = new ArrayList<Wall>();
    
    //get the vertex positions from your original node
    ArrayList<PVector> vertices = new ArrayList<PVector>();
    for(Wall w: node.polygon)
    {
      vertices.add(w.start);
    }
    
    //for polyA, just make a polygon from index A to B
    //when splitting a polygon, one of them will never have a jump in index values between vertices
    for(int i = indexA; i<=indexB; i++)
    {
      //finishes the polygon
      if (i == indexB) {
        polyA.add( new Wall(vertices.get(indexB), vertices.get(indexA)) );
        //println(indexB + " to " + indexA);
        break;
      }
      
      int nextIndex = i+1;
      if (nextIndex > vertices.size()-1) nextIndex = 0;
      polyA.add( new Wall(vertices.get(i), vertices.get(nextIndex)) );
      //println(i + " to " + nextIndex);
    }
    
    //for polyB
    //a little bit tricker, since poly b has a disjunction between vertex indices
    //the loop is thus different for constructing b
    //start from indexB and go further until you hit index A. You are guaranteed to finish the polygon once you connect A and B.
    int i = indexB;
    boolean completedPolyB = false;
    while (!completedPolyB) {
      if (i == indexA) {
        polyB.add( new Wall(vertices.get(indexA), vertices.get(indexB)) );
        completedPolyB = true;
        break;
      }
      
      int nextIndex = i+1;
      if (nextIndex > vertices.size()-1) nextIndex = 0;
      polyB.add( new Wall(vertices.get(i), vertices.get(nextIndex)) );
      
      i = nextIndex;
    }
    
    //we'll create a node to store poly a
    Node nodeA = new Node(recursionDepth+"A", polyA);
    setIndicesOf(nodeA);
    nodes.add(nodeA);
    
    //the same goes for b
    Node nodeB = new Node(recursionDepth+"B", polyB);
    setIndicesOf(nodeB);    
    nodes.add(nodeB);
    
    //just some debugging
    println("Convex Point Number: " + convexPoint);
    println("");
    for(int k = 0; k<nodes.size(); k++)
    {
      println("Polygon " + k + ": Length of " + nodes.get(k).polygon.size());
    }
    printOutNodeList();
    
    //this portion is not necessary for the program to function but it helps when debugging
    recursionDepth++;
    if (recursionDepth == maxDepth) return;
    
    //polygons are added to the node list, in order of A and B
    //0.[NODE 0A] 
    //1.[NODE 0B]
    
    //getIndexOfBadVertex will return -1 if the shape is all good
    //remove the bad nodes from the list and add in two new ones
    //order in the node list has no effect on neighboring
    //the node list functions identically to a bag in that regard
    if (getIndexOfBadVertex(polyA) != -1) {
      //0.[NODE 0A] 
      //1.[NODE 0B]
      //     V
      //0.[NODE 0B]
      //1.[NODE 1A]
      //2.[NODE 1B]
      
      nodes.remove(nodeA);
      correctPolygon(nodeA);
    }
    if (getIndexOfBadVertex(polyB) != -1) {
      //0.[NODE 0A] 
      //1.[NODE 0B]
      //     V
      //0.[NODE 0A]
      //1.[NODE 1A]
      //2.[NODE 1B]
      nodes.remove(nodeB);
      correctPolygon(nodeB);
    }
  }

  //locates reflexive angles within a list of walls and tells you which index of the polygon is at fault
  int getIndexOfBadVertex(ArrayList<Wall> polygon)
  { 
    //find angles in which you have to turn right
    for (int i = 0; i<polygon.size(); i++)
    {
      PVector aNormal = polygon.get(i).normal;
      int nextIndex = i+1;
      if (nextIndex >= polygon.size()) nextIndex = 0;
      PVector bDirection = polygon.get(nextIndex).direction;
      float dotProduct = aNormal.dot(bDirection);

      if (dotProduct >= 0) {
        return nextIndex;
      }
    }
    
    return -1;
  }
  
  //given a reflexive index, find a vertex that you can go to without intersection another wall 
  int getIndexOfConnectingVertex(ArrayList<Wall> polygon, int convexIndex)
  {
    //you need the PVectors for this one
    ArrayList<PVector> vertices = new ArrayList<PVector>();
    for(Wall w: polygon)
    {
      vertices.add(w.start);
    }
    
    //our "bad" point
    PVector pointAtIndex = vertices.get(convexIndex);

    //we don't need to consider the vertex's neighbors since they obviously can't be connected to
    int nextIndex = convexIndex + 1;
    if (nextIndex >= vertices.size()) nextIndex = 0;

    int lastIndex = convexIndex - 1;
    if (lastIndex < 0) lastIndex = vertices.size() - 1;

    for (int potentialConnecting = vertices.size()-1; potentialConnecting>=0; potentialConnecting--)
    {
      //skip neighbors and the bad point
      if (potentialConnecting == nextIndex || potentialConnecting == convexIndex || potentialConnecting == lastIndex) continue;

      PVector potentialConnectingPoint = vertices.get(potentialConnecting);
      
      if (!map.intersectsWall(pointAtIndex, potentialConnectingPoint))
      {
        return potentialConnecting;
      }
    }
    
    return -1;
  }
  
  //given a node, determine its bad point, a connecting point, then split it
  //assume that the node needs correction, but I put a return case in there anyway
  
  //the idea is that correctPolygon will work recursively. With every recursive iteration it will add 2 more polygons to the map.
  void correctPolygon(Node node)
  {
    int convexIndex = getIndexOfBadVertex(node.polygon);
    if (convexIndex == -1) return;
    else println("      convex index: " + convexIndex);
    
    int connectingIndex = getIndexOfConnectingVertex(node.polygon, convexIndex);
    if (connectingIndex == -1) return;
    else println("      connecting index: " + connectingIndex);
    
    //splitPolygon will create 2 polygons by iterating through the vertex list in numerical order
    //if the convexIndex is 5, and the connecting index is 3, it's going to break
    splitPolygon(node, min(convexIndex, connectingIndex), max(convexIndex, connectingIndex), convexIndex);
  }

  //creates a hashmap with key PVector and value Integer
  //creating a hashmap for this removes the risk of directly comparing PVectors since it should look by reference instead of value
  void setVertexMap(Map m)
  {
    mapVectors.clear();
    vertexLookUp.clear();
    for (int i = 0;i<m.walls.size();i++)
    {
      vertexLookUp.put(m.walls.get(i).start, i);
      mapVectors.add(m.walls.get(i).start);
      println("Added " + m.walls.get(i) + " corresponding to index " + i);
    }
  }

  ArrayList<PVector> path = new ArrayList<PVector>();

  void bake(Map map)
  {    
    //reset previous values
    recursionDepth = 0;
    nodes.clear();
    pointAmount = map.walls.size();
    
    vertexLookUp.clear();
    mapVectors.clear();
    
    //create hashmap
    setVertexMap(map);
    
    //turn map into node
    Node m = new Node("Map", map.walls);
    setIndicesOf(m);
    
    //let the recursive chain run its course
    correctPolygon(m);
    
    //who neighbors who now?
    calculateAdjacencies();
    
    //some nice printouts
    printAdjacencies();
    println("Reduced map to " + nodes.size() + " polygons");
    
    print("Reseting map to have max depth of " + maxDepth);
    
    for(Node n: nodes)
    {
      //println(n.id + ": " + distanceBetween(end, n.center));
    }
  }

  //precondition: assume nodes is initialized with members
  Node nodeFromPoint(PVector p)
  {
    for (Node n: nodes)
    {
      if (n.pointInNode(p))
        return n;
    }
    
    return null;
  }
  
  ArrayList<PVector> findPath(PVector start, PVector destination)
  {
    //implement A*Star to find a path
    ArrayList<PVector> result = new ArrayList<PVector>(); //contains the path the boid will take
    ArrayList<FrontierEntry> frontier = new ArrayList<FrontierEntry>(); //contains the frontiers that A*Star uses to find the path
    ArrayList<Node> visited = new ArrayList<Node>(); //contains nodes that we have already visited, so the boid doesn't go backwards
    Node startNode = nodeFromPoint(start);
    Node finalNode = nodeFromPoint(destination); //contains the node the destination is at 
    
    FrontierEntry s = new FrontierEntry(startNode, null, finalNode.getCenter());
    frontier.add(s);
    visited.add(frontier.get(0).node);
    
    while (frontier.get(0).node != finalNode)
    {
      FrontierEntry front = frontier.get(0);
      for (Node neighbor: front.node.neighbors)
      {
        if (!visited.contains(neighbor))
        {
          frontier.add(new FrontierEntry(neighbor, front, finalNode.getCenter())); 
        }
      }
      frontier.remove(0);
      frontier.sort((a,b) -> {
        if (a.getHeuristicSum() > b.getHeuristicSum()) return 1;
        else if (a.getHeuristicSum() < b.getHeuristicSum()) return -1;
        else return 0;
      });
      visited.add(front.node);
      frontierPrintOut(frontier);
      println("-----------------------");
    }
    
    return traceFrontierPath(destination, startNode, frontier);
  }
  
  void frontierPrintOut(ArrayList<FrontierEntry> frontier) {
    for (FrontierEntry f: frontier) {
      println(f.node.id + " (" + f.getHeuristicSum() + ")");
    }
  }
  
  ArrayList<PVector> traceFrontierPath(PVector destination, Node startNode, ArrayList<FrontierEntry> genPath)
  {
    ArrayList<PVector> result = new ArrayList<PVector>();
    result.add(destination);
    FrontierEntry front = genPath.get(0);
    while (front.node != startNode) {
      PVector midPoint = midPointBetweenNeighbors(front.node, front.previousFrontier.node);
      //result.add(front.node.center);
      result.add(midPoint);
      front = front.previousFrontier;
    }
    Collections.reverse(result);
    return result;
  }

  
  void update(float dt)
  {
    draw();
  }

  void draw()
  {
    for(int i = 0; i<map.walls.size();i++)
    {
      text(i,map.walls.get(i).start.x+10, map.walls.get(i).start.y+10);
    }
    
    for (Node n: nodes)
    {
      for (Wall w: n.polygon)
      {
          line(w.start.x, w.start.y, w.end.x, w.end.y);
      }
      PVector center = n.getCenter();
      text(n.id + "", center.x, center.y); 
      for (Node nb: n.neighbors)
      {
        //line(center.x, center.y, nb.getCenter().x, nb.getCenter().y);
      }
    }
    for (int i = 0; i<path.size()-1; i++)
    {
      int j = i+1;
      line(path.get(i).x, path.get(i).y, path.get(j).x, path.get(j).y);
    }
  }
}
