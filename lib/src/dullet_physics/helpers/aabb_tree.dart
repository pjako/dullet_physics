part of dullet_physics;



typedef AABBTreeRayTestResult(AabbTree tree, WebGLPhysicsPrivateBody node, ray, double distance, upperBound);
/*class AABBTreeRayTestResult extends RayHit {
  double factor;
}*/


class PriorityNode {
  AabbTree tree;
  int nodeIndex;
  double distance;
  PriorityNode(this.tree, this.nodeIndex, this.distance);
}


class AABBTreeRay {
  Vector3 origin; // v3
  Vector3 direction; // v3
  double maxFactor;
}

typedef GetKey(AabbTreeNode node);


double getkeyXfn(AabbTreeNode node) {
  var extents = node.extents;
  return (extents.min.storage[0] + extents.max.storage[0]);
}

double getkeyYfn(AabbTreeNode node) {
  var extents = node.extents;
  return (extents.min.storage[1] + extents.max.storage[1]);
}

double getkeyZfn(AabbTreeNode node) {
  var extents = node.extents;
  return (extents.min.storage[2] + extents.max.storage[2]);
}

double getreversekeyXfn(AabbTreeNode node) {
  var extents = node.extents;
  return -(extents.min.storage[0] + extents.max.storage[0]);
}

double getreversekeyYfn(AabbTreeNode node) {
  var extents = node.extents;
  return -(extents.min.storage[1] + extents.max.storage[1]);
}

double getreversekeyZfn(AabbTreeNode node) {
  var extents = node.extents;
  return -(extents.min.storage[2] + extents.max.storage[2]);
}

double getkeyXZfn(AabbTreeNode node) {
  var extents = node.extents;
  return (extents.min.storage[0] + extents.min.storage[2] + extents.max.storage[0] + extents.max.storage[2]);
}

double getkeyZXfn(AabbTreeNode node) {
  var extents = node.extents;
  return (extents.min.storage[0] - extents.min.storage[2] + extents.max.storage[0] - extents.max.storage[2]);
}
double getreversekeyXZfn(AabbTreeNode node) {
  var extents = node.extents;
  return -(extents.min.storage[0] + extents.min.storage[2] + extents.max.storage[0] + extents.max.storage[2]);
}

double getreversekeyZXfn(AabbTreeNode node) {
  var extents = node.extents;
  return -(extents.min.storage[0] - extents.min.storage[2] + extents.max.storage[0] - extents.max.storage[2]);
}


//
// AABBTreeNode
//

class ExternalAabbTreeNode {
  int aabbTreeIndex;
}

class AabbTreeNode {
  static const int version = 1;


  int escapeNodeOffset;
  ExternalAabbTreeNode externalNode;//     : {};
  Aabb3 extents; //          : any;

  AabbTreeNode(Aabb3 extents, var escapeNodeOffset, [ExternalAabbTreeNode externalNode]) {
    this.escapeNodeOffset = escapeNodeOffset;
    this.externalNode = externalNode;
    this.extents = extents;
  }

  bool isLeaf() {
    return this.externalNode != null;
  }

  void reset(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, 
  int escapeNodeOffset, [ExternalAabbTreeNode externalNode]) {
    this.escapeNodeOffset = escapeNodeOffset;
    this.externalNode = externalNode;
    Aabb3 oldExtents = this.extents;
    oldExtents.min.storage[0] = minX;
    oldExtents.min.storage[1] = minY;
    oldExtents.min.storage[2] = minZ;
    oldExtents.max.storage[0] = maxX;
    oldExtents.max.storage[1] = maxY;
    oldExtents.max.storage[2] = maxZ;
  }

  void clear() {
    this.escapeNodeOffset = 1;
    this.externalNode = null;
    Aabb3 oldExtents = this.extents;
    var maxNumber = double.MAX_FINITE;//Number.MAX_VALUE;
    oldExtents.min.storage[0] = maxNumber;
    oldExtents.min.storage[1] = maxNumber;
    oldExtents.min.storage[2] = maxNumber;
    oldExtents.max.storage[0] = -maxNumber;
    oldExtents.max.storage[1] = -maxNumber;
    oldExtents.max.storage[2] = -maxNumber;
  }

// Constructor dynamic
/*static create(extents: any, escapeNodeOffset: number,
                externalNode?: AABBTreeNode): AABBTreeNode
  {
      return new AABBTreeNode(extents, escapeNodeOffset, externalNode);
  }*/
}

//
// AABBTree
//
class AabbTree {
  static const int version = 1;
  int numNodesLeaf = 4;

  List<AabbTreeNode> nodes = [];
  int endNode;
  bool needsRebuild;
  bool needsRebound;
  int numAdds;
  int numUpdates;
  int numExternalNodes;
  int startUpdate;
  int endUpdate;
  bool highQuality;
  bool ignoreY;
  List<int> nodesStack = [];

  //arrayConstructor: any;

  AabbTree([this.highQuality = false]) {
    this.nodes = [];
    this.endNode = 0;
    this.needsRebuild = false;
    this.needsRebound = false;
    this.numAdds = 0;
    this.numUpdates = 0;
    this.numExternalNodes = 0;
    this.startUpdate = 0x7FFFFFFF;
    this.endUpdate = -0x7FFFFFFF;
    //this.highQuality = highQuality;
    this.ignoreY = false;
    this.nodesStack = new List(32);
  }

  void add(ExternalAabbTreeNode externalNode, Aabb3 extents) {
    var endNode = this.endNode;
    externalNode.aabbTreeIndex = endNode;
    Aabb3 copyExtents = new Aabb3.copy(extents);
    //this.nodes[endNode] = new AabbTreeNode(copyExtents, 1, externalNode);
    this.nodes.add(new AabbTreeNode(copyExtents, 1, externalNode));
    this.endNode = (endNode + 1);
    this.needsRebuild = true;
    this.numAdds += 1;
    this.numExternalNodes += 1;
  }

  void remove(ExternalAabbTreeNode externalNode) {
    var index = externalNode.aabbTreeIndex;
    if (index != null) {
      if (this.numExternalNodes > 1) {
        var nodes = this.nodes;
        nodes[index].clear();
        var endNode = this.endNode;
        if ((index + 1) >= endNode) {
          // No leaf
          while (nodes[endNode - 1].externalNode == null) {
            endNode -= 1;
          }
          this.endNode = endNode;
        } else {
          this.needsRebuild = true;
        }
        this.numExternalNodes -= 1;
      } else {
        this.clear();
      }
      externalNode.aabbTreeIndex = null;
    //delete externalNode.aabbTreeIndex;
    }
  }

  AabbTreeNode findParent(int nodeIndex) {
    var nodes = this.nodes;
    var parentIndex = nodeIndex;
    var nodeDist = 0;
    var parent;
    do {
      parentIndex -= 1;
      nodeDist += 1;
      parent = nodes[parentIndex];
    } while (parent.escapeNodeOffset <= nodeDist);
    return parent;
  }

  void update(ExternalAabbTreeNode externalNode, Aabb3 extents) {
    var index = externalNode.aabbTreeIndex;
    if (index != null) {
      var min0 = extents.min.storage[0];
      var min1 = extents.min.storage[1];
      var min2 = extents.min.storage[2];
      var max0 = extents.max.storage[0];
      var max1 = extents.max.storage[1];
      var max2 = extents.max.storage[2];

      bool needsRebuild = this.needsRebuild;
      bool needsRebound = this.needsRebound;
      var nodes = this.nodes;
      var node = nodes[index];
      var nodeExtents = node.extents;

      bool doUpdate = (needsRebuild ||
       needsRebound ||
       nodeExtents.min.storage[0] > min0 ||
       nodeExtents.min.storage[1] > min1 ||
       nodeExtents.min.storage[2] > min2 ||
       nodeExtents.max.storage[0] < max0 ||
       nodeExtents.max.storage[1] < max1 ||
       nodeExtents.max.storage[2] < max2);

      nodeExtents.min.storage[0] = min0;
      nodeExtents.min.storage[1] = min1;
      nodeExtents.min.storage[2] = min2;
      nodeExtents.max.storage[0] = max0;
      nodeExtents.max.storage[1] = max1;
      nodeExtents.max.storage[2] = max2;

      if (doUpdate)
       {
        if (!needsRebuild && 1 < nodes.length)
         {
          this.numUpdates += 1;
          if (this.startUpdate > index)
           {
            this.startUpdate = index;
          }
          if (this.endUpdate < index)
           {
            this.endUpdate = index;
          }
          if (!needsRebound)
           {
            // force a rebound when things change too much
            if ((2 * this.numUpdates) > this.numExternalNodes)
             {
              this.needsRebound = true;
            }
             else
             {
              var parent = this.findParent(index);
              Aabb3 parentExtents = parent.extents;
              if (parentExtents.min.storage[0] > min0 ||
               parentExtents.min.storage[1] > min1 ||
               parentExtents.min.storage[2] > min2 ||
               parentExtents.max.storage[0] < max0 ||
               parentExtents.max.storage[1] < max1 ||
               parentExtents.max.storage[2] < max2)
               {
                this.needsRebound = true;
              }
            }
          } else {
            // force a rebuild when things change too much
            if (this.numUpdates > (3 * this.numExternalNodes))
             {
              this.needsRebuild = true;
              this.numAdds = this.numUpdates;
            }
          }
        }
      }
    } else {
      this.add(externalNode, extents);
    }
  }

  bool needsFinalize() => (this.needsRebuild || this.needsRebound);

  void finalize() {
    if (this.needsRebuild) {
      this.rebuild();
    } else {
      if (this.needsRebound) {
        this.rebound();
      }
    }
  }

  void rebound() {
    List<AabbTreeNode> nodes = this.nodes;
    if (nodes.length > 1) {
      int startUpdateNodeIndex = this.startUpdate;
      int endUpdateNodeIndex = this.endUpdate;

      List<int> nodesStack = this.nodesStack;
      int numNodesStack = 0;
      int topNodeIndex = 0;
      for (; ;) {
        var topNode = nodes[topNodeIndex];
        var currentNodeIndex = topNodeIndex;
        var currentEscapeNodeIndex = (topNodeIndex + topNode.escapeNodeOffset);
        var nodeIndex = (topNodeIndex + 1); // First child
        var node;
        do {
          node = nodes[nodeIndex];
          var escapeNodeIndex = (nodeIndex + node.escapeNodeOffset);
          if (nodeIndex < endUpdateNodeIndex) {
            if (node.externalNode == null) { // No leaf
              if (escapeNodeIndex > startUpdateNodeIndex)
               {
                nodesStack[numNodesStack] = topNodeIndex;
                numNodesStack += 1;
                topNodeIndex = nodeIndex;
              }
            }
          } else {
            break;
          }
          nodeIndex = escapeNodeIndex;
        } while (nodeIndex < currentEscapeNodeIndex);

        if (topNodeIndex == currentNodeIndex) {
          nodeIndex = (topNodeIndex + 1); // First child
          node = nodes[nodeIndex];

          Aabb3 extents = node.extents;
          var minX = extents.min.storage[0];
          var minY = extents.min.storage[1];
          var minZ = extents.min.storage[2];
          var maxX = extents.max.storage[0];
          var maxY = extents.max.storage[1];
          var maxZ = extents.max.storage[2];

          nodeIndex = (nodeIndex + node.escapeNodeOffset);
          while (nodeIndex < currentEscapeNodeIndex) {
            node = nodes[nodeIndex];
            extents = node.extents;
            /*jshint white: false*/
            if (minX > extents.min.storage[0]) {
              minX = extents.min.storage[0];
            }
            if (minY > extents.min.storage[1]) {
              minY = extents.min.storage[1];
            }
            if (minZ > extents.min.storage[2]) {
              minZ = extents.min.storage[2];
            }
            if (maxX < extents.max.storage[0]) {
              maxX = extents.max.storage[0];
            }
            if (maxY < extents.max.storage[1]) {
              maxY = extents.max.storage[1];
            }
            if (maxZ < extents.max.storage[2]) {
              maxZ = extents.max.storage[2];
            }
            /*jshint white: true*/
            nodeIndex = (nodeIndex + node.escapeNodeOffset);
          }

          extents = topNode.extents;
          extents.min.storage[0] = minX;
          extents.min.storage[1] = minY;
          extents.min.storage[2] = minZ;
          extents.max.storage[0] = maxX;
          extents.max.storage[1] = maxY;
          extents.max.storage[2] = maxZ;

          endUpdateNodeIndex = topNodeIndex;

          if (0 < numNodesStack) {
            numNodesStack -= 1;
            topNodeIndex = nodesStack[numNodesStack];
          } else {
            break;
          }
        }
      }
    }

    this.needsRebuild = false;
    this.needsRebound = false;
    this.numAdds = 0;
    //this.numUpdates = 0;
    this.startUpdate = 0x7FFFFFFF;
    this.endUpdate = -0x7FFFFFFF;
  }
  void rebuild() {
    if (this.numExternalNodes > 0) {
      var nodes = this.nodes;

      var buildNodes, numBuildNodes, endNodeIndex;

      if (this.numExternalNodes == nodes.length) {
        buildNodes = nodes;
        numBuildNodes = nodes.length;
        nodes = [];
        this.nodes = nodes;
      } else {
        buildNodes = [];
        buildNodes.length = this.numExternalNodes;
        numBuildNodes = 0;
        endNodeIndex = this.endNode;
        for (var n = 0; n < endNodeIndex; n += 1)
         {
          var currentNode = nodes[n];
          if (currentNode.externalNode != null)  // Is leaf
          {
            nodes[n] = null;
            buildNodes[numBuildNodes] = currentNode;
            numBuildNodes += 1;
          }
        }
        if (buildNodes.length > numBuildNodes)
         {
          buildNodes.length = numBuildNodes;
        }
      }

      var rootNode;
      if (numBuildNodes > 1)
       {
        if (numBuildNodes > this.numNodesLeaf &&
         this.numAdds > 0)
         {
          if (this.highQuality)
           {
            this.sortNodesHighQuality(buildNodes);
          }
           else {
            if (this.ignoreY)
             {
              this.sortNodesNoY(buildNodes);
            }
             else
             {
              this.sortNodes(buildNodes);
            }
          }
        }

        nodes.length = this.predictNumNodes(0, numBuildNodes, 0);

        this.recursiveBuild(buildNodes, 0, numBuildNodes, 0);

        endNodeIndex = nodes[0].escapeNodeOffset;
        if (nodes.length > endNodeIndex)
         {
          nodes.length = endNodeIndex;
        }
        this.endNode = endNodeIndex;

        // Check if we should take into account the Y coordinate
        rootNode = nodes[0];
        var extents = rootNode.extents;
        var deltaX = (extents.max.storage[0] - extents.min.storage[0]);
        var deltaY = (extents.max.storage[1] - extents.min.storage[1]);
        var deltaZ = (extents.max.storage[2] - extents.min.storage[2]);
        this.ignoreY = ((4 * deltaY) < (deltaX <= deltaZ ? deltaX : deltaZ));
      } else {
        rootNode = buildNodes[0];
        rootNode.externalNode.aabbTreeIndex = 0;
        nodes.length = 1;
        nodes[0] = rootNode;
        this.endNode = 1;
      }
      buildNodes = null;
    }

    this.needsRebuild = false;
    this.needsRebound = false;
    this.numAdds = 0;
    this.numUpdates = 0;
    this.startUpdate = 0x7FFFFFFF;
    this.endUpdate = -0x7FFFFFFF;
  }



  void sortNodes(List<AabbTreeNode> nodes) {

    var numNodesLeaf = this.numNodesLeaf;
    var numNodes = nodes.length;



    //var nthElement = this.nthElement;
    bool reverse = false;
    int axis = 0;

    void sortNodesRecursive(nodes, startIndex, endIndex)
     {
      /*jshint bitwise: false*/
      int splitNodeIndex = ((startIndex + endIndex) >> 1);
      /*jshint bitwise: true*/

      if (axis == 0) {
        if (reverse) {
          nthElement(nodes, startIndex, splitNodeIndex, endIndex, getreversekeyXfn);
        } else {
          nthElement(nodes, startIndex, splitNodeIndex, endIndex, getkeyXfn);
        }
      } else {
        if (axis == 2) {
          if (reverse) {
            nthElement(nodes, startIndex, splitNodeIndex, endIndex, getreversekeyZfn);
          } else {
            nthElement(nodes, startIndex, splitNodeIndex, endIndex, getkeyZfn);
          }
        } else { //if (axis == 1)
          if (reverse) {
            nthElement(nodes, startIndex, splitNodeIndex, endIndex, getreversekeyYfn);
          } else {
            nthElement(nodes, startIndex, splitNodeIndex, endIndex, getkeyYfn);
          }
        }
      }
      if (axis == 0) {
        axis = 2;
      } else {
        if (axis == 2) {
          axis = 1;
        } else { //if (axis == 1)
          axis = 0;
        }
      }
      reverse = !reverse;

      if ((startIndex + numNodesLeaf) < splitNodeIndex) {
        sortNodesRecursive(nodes, startIndex, splitNodeIndex);
      }

      if ((splitNodeIndex + numNodesLeaf) < endIndex) {
        sortNodesRecursive(nodes, splitNodeIndex, endIndex);
      }
    }
    sortNodesRecursive(nodes, 0, numNodes);
  }

  void sortNodesNoY(List<AabbTreeNode> nodes) {

    var numNodesLeaf = this.numNodesLeaf;
    var numNodes = nodes.length;
    /*
      dynamic getkeyXfn(node)
      {
          var extents = node.extents;
          return (extents[0] + extents[3]);
      }

      dynamic getkeyZfn(node)
      {
          var extents = node.extents;
          return (extents[2] + extents[5]);
      }

      dynamic getreversekeyXfn(node)
      {
          var extents = node.extents;
          return -(extents[0] + extents[3]);
      }

      dynamic getreversekeyZfn(node)
      {
          var extents = node.extents;
          return -(extents[2] + extents[5]);
      }
*/
    //var nthElement = this.nthElement;
    var reverse = false;
    var axis = 0;

    void sortNodesNoYRecursive(List<AabbTreeNode> nodes, int startIndex, int endIndex) {
      /*jshint bitwise: false*/
      var splitNodeIndex = ((startIndex + endIndex) >> 1);
      /*jshint bitwise: true*/

      if (axis == 0) {
        if (reverse) {
          nthElement(nodes, startIndex, splitNodeIndex, endIndex, getreversekeyXfn);
        } else {
          nthElement(nodes, startIndex, splitNodeIndex, endIndex, getkeyXfn);
        }
      } else { //if (axis == 2)
        if (reverse) {
          nthElement(nodes, startIndex, splitNodeIndex, endIndex, getreversekeyZfn);
        } else {
          nthElement(nodes, startIndex, splitNodeIndex, endIndex, getkeyZfn);
        }
      }

      if (axis == 0) {
        axis = 2;
      } else { //if (axis == 2)
        axis = 0;
      }

      reverse = !reverse;

      if ((startIndex + numNodesLeaf) < splitNodeIndex)
       {
        sortNodesNoYRecursive(nodes, startIndex, splitNodeIndex);
      }

      if ((splitNodeIndex + numNodesLeaf) < endIndex)
       {
        sortNodesNoYRecursive(nodes, splitNodeIndex, endIndex);
      }
    }

    sortNodesNoYRecursive(nodes, 0, numNodes);
  }
  void sortNodesHighQuality(List<AabbTreeNode> nodes) {
    var numNodesLeaf = this.numNodesLeaf;
    var numNodes = nodes.length;
    /*
    dynamic getkeyXfn(node) {
        var extents = node.extents;
        return (extents[0] + extents[3]);
    }

    dynamic getkeyYfn(node)
    {
        var extents = node.extents;
        return (extents[1] + extents[4]);
    }

    dynamic getkeyZfn(node)
    {
        var extents = node.extents;
        return (extents[2] + extents[5]);
    }

    dynamic getkeyXZfn(node)
    {
        var extents = node.extents;
        return (extents[0] + extents[2] + extents[3] + extents[5]);
    }

    dynamic getkeyZXfn(node)
    {
        var extents = node.extents;
        return (extents[0] - extents[2] + extents[3] - extents[5]);
    }

    dynamic getreversekeyXfn(node)
    {
        var extents = node.extents;
        return -(extents[0] + extents[3]);
    }

    dynamic getreversekeyYfn(node)
    {
        var extents = node.extents;
        return -(extents[1] + extents[4]);
    }

    dynamic getreversekeyZfn(node)
    {
        var extents = node.extents;
        return -(extents[2] + extents[5]);
    }

    dynamic getreversekeyXZfn(node)
    {
        var extents = node.extents;
        return -(extents[0] + extents[2] + extents[3] + extents[5]);
    }

    dynamic getreversekeyZXfn(node)
    {
        var extents = node.extents;
        return -(extents[0] - extents[2] + extents[3] - extents[5]);
    }
*/

    //var nthElement = this.nthElement;
    var calculateSAH = this.calculateSAH;
    var reverse = false;

    void sortNodesHighQualityRecursive(List<AabbTreeNode> nodes, int startIndex, int endIndex)
     {
      /*jshint bitwise: false*/
      var splitNodeIndex = ((startIndex + endIndex) >> 1);
      /*jshint bitwise: true*/

      nthElement(nodes, startIndex, splitNodeIndex, endIndex, getkeyXfn);
      var sahX = (calculateSAH(nodes, startIndex, splitNodeIndex) + calculateSAH(nodes, splitNodeIndex, endIndex));

      nthElement(nodes, startIndex, splitNodeIndex, endIndex, getkeyYfn);
      var sahY = (calculateSAH(nodes, startIndex, splitNodeIndex) + calculateSAH(nodes, splitNodeIndex, endIndex));

      nthElement(nodes, startIndex, splitNodeIndex, endIndex, getkeyZfn);
      var sahZ = (calculateSAH(nodes, startIndex, splitNodeIndex) + calculateSAH(nodes, splitNodeIndex, endIndex));

      nthElement(nodes, startIndex, splitNodeIndex, endIndex, getkeyXZfn);
      var sahXZ = (calculateSAH(nodes, startIndex, splitNodeIndex) + calculateSAH(nodes, splitNodeIndex, endIndex));

      nthElement(nodes, startIndex, splitNodeIndex, endIndex, getkeyZXfn);
      var sahZX = (calculateSAH(nodes, startIndex, splitNodeIndex) + calculateSAH(nodes, splitNodeIndex, endIndex));

      if (sahX <= sahY &&
       sahX <= sahZ &&
       sahX <= sahXZ &&
       sahX <= sahZX)
       {
        if (reverse)
         {
          nthElement(nodes, startIndex, splitNodeIndex, endIndex, getreversekeyXfn);
        }
         else
         {
          nthElement(nodes, startIndex, splitNodeIndex, endIndex, getkeyXfn);
        }
      }
       else {
        if (sahZ <= sahY &&
         sahZ <= sahXZ &&
         sahZ <= sahZX)
         {
          if (reverse)
           {
            nthElement(nodes, startIndex, splitNodeIndex, endIndex, getreversekeyZfn);
          }
           else
           {
            nthElement(nodes, startIndex, splitNodeIndex, endIndex, getkeyZfn);
          }
        }
         else {
          if (sahY <= sahXZ &&
           sahY <= sahZX)
           {
            if (reverse)
             {
              nthElement(nodes, startIndex, splitNodeIndex, endIndex, getreversekeyYfn);
            }
             else
             {
              nthElement(nodes, startIndex, splitNodeIndex, endIndex, getkeyYfn);
            }
          }
           else {
            if (sahXZ <= sahZX)
             {
              if (reverse)
               {
                nthElement(nodes, startIndex, splitNodeIndex, endIndex, getreversekeyXZfn);
              }
               else
               {
                nthElement(nodes, startIndex, splitNodeIndex, endIndex, getkeyXZfn);
              }
            }
             else  //if (sahZX <= sahXZ)
            {
              if (reverse)
               {
                nthElement(nodes, startIndex, splitNodeIndex, endIndex, getreversekeyZXfn);
              }
               else
               {
                nthElement(nodes, startIndex, splitNodeIndex, endIndex, getkeyZXfn);
              }
            }
          }
        }
      }
      reverse = !reverse;

      if ((startIndex + numNodesLeaf) < splitNodeIndex)
       {
        sortNodesHighQualityRecursive(nodes, startIndex, splitNodeIndex);
      }

      if ((splitNodeIndex + numNodesLeaf) < endIndex)
       {
        sortNodesHighQualityRecursive(nodes, splitNodeIndex, endIndex);
      }
    }

    sortNodesHighQualityRecursive(nodes, 0, numNodes);
  }
  num calculateSAH(List<AabbTreeNode> buildNodes, startIndex, endIndex) {
    AabbTreeNode buildNode;
    Aabb3 extents; //, minX, minY, minZ, maxX, maxY, maxZ;

    buildNode = buildNodes[startIndex];
    extents = buildNode.extents;
    var minX = extents.min.storage[0];
    var minY = extents.min.storage[1];
    var minZ = extents.min.storage[2];
    var maxX = extents.max.storage[0];
    var maxY = extents.max.storage[1];
    var maxZ = extents.max.storage[2];


    for (var n = (startIndex + 1); n < endIndex; n += 1) {
      buildNode = buildNodes[n];
      extents = buildNode.extents;
      /*jshint white: false*/
      if (minX > extents.min.storage[0]) {
        minX = extents.min.storage[0];
      }
      if (minY > extents.min.storage[1]) {
        minY = extents.min.storage[1];
      }
      if (minZ > extents.min.storage[2]) {
        minZ = extents.min.storage[2];
      }
      if (maxX < extents.max.storage[0]) {
        maxX = extents.max.storage[0];
      }
      if (maxY < extents.max.storage[1]) {
        maxY = extents.max.storage[1];
      }
      if (maxZ < extents.max.storage[2]) {
        maxZ = extents.max.storage[2];
      }
    /*jshint white: true*/
    }
    return ((maxX - minX) + (maxY - minY) + (maxZ - minZ));
  }
  void nthElement(List<AabbTreeNode> nodes, int first, int nth, int last, GetKey getkey) {

    dynamic medianFn(double a, double b, double c)
     {
      if (a < b)
       {
        if (b < c)
         {
          return b;
        }
         else {
          if (a < c)
           {
            return c;
          }
           else
           {
            return a;
          }
        }
      }
       else {
        if (a < c)
         {
          return a;
        }
         else {
          if (b < c)
           {
            return c;
          }
        }
      }
      return b;
    }

    void insertionSortFn(List<AabbTreeNode> nodes, int first, int last, GetKey getkey)
     {
      var sorted = (first + 1);
      while (sorted != last)
       {
        var tempNode = nodes[sorted];
        var tempKey = getkey(tempNode);

        var next = sorted;
        var current = (sorted - 1);

        while (next != first && tempKey < getkey(nodes[current]))
         {
          nodes[next] = nodes[current];
          next -= 1;
          current -= 1;
        }

        if (next != sorted)
         {
          nodes[next] = tempNode;
        }

        sorted += 1;
      }
    }

    while ((last - first) > 8)
     {
      /*jshint bitwise: false*/
      var midValue = medianFn(getkey(nodes[first]),
       getkey(nodes[first + ((last - first) >> 1)]),
       getkey(nodes[last - 1]));
      /*jshint bitwise: true*/

      var firstPos = first;
      var lastPos = last;
      var midPos;
      for (; ; firstPos += 1)
       {
        while (getkey(nodes[firstPos]) < midValue)
         {
          firstPos += 1;
        }

        do
         {
          lastPos -= 1;
        }
         while (midValue < getkey(nodes[lastPos]));

        if (firstPos >= lastPos)
         {
          midPos = firstPos;
          break;
        }
         else
         {
          var temp = nodes[firstPos];
          nodes[firstPos] = nodes[lastPos];
          nodes[lastPos] = temp;
        }
      }

      if (midPos <= nth)
       {
        first = midPos;
      }
       else
       {
        last = midPos;
      }
    }

    insertionSortFn(nodes, first, last, getkey);
  }

  void recursiveBuild(List<AabbTreeNode> buildNodes, int startIndex, int endIndex, int lastNodeIndex) {

    var nodes = this.nodes;
    var nodeIndex = lastNodeIndex;
    lastNodeIndex += 1;

    double minX, minY, minZ, maxX, maxY, maxZ;
    var extents;
    var buildNode, lastNode;

    if ((startIndex + this.numNodesLeaf) >= endIndex)
     {
      buildNode = buildNodes[startIndex];
      extents = buildNode.extents;
      minX = extents.min.storage[0];
      minY = extents.min.storage[1];
      minZ = extents.min.storage[2];
      maxX = extents.max.storage[0];
      maxY = extents.max.storage[1];
      maxZ = extents.max.storage[2];

      buildNode.externalNode.aabbTreeIndex = lastNodeIndex;
      this.replaceNode(nodes, lastNodeIndex, buildNode);

      for (var n = (startIndex + 1); n < endIndex; n += 1)
       {
        buildNode = buildNodes[n];
        extents = buildNode.extents;
        /*jshint white: false*/
        if (minX > extents.min.storage[0]) {
          minX = extents.min.storage[0];
        }
        if (minY > extents.min.storage[1]) {
          minY = extents.min.storage[1];
        }
        if (minZ > extents.min.storage[2]) {
          minZ = extents.min.storage[2];
        }
        if (maxX < extents.max.storage[0]) {
          maxX = extents.max.storage[0];
        }
        if (maxY < extents.max.storage[1]) {
          maxY = extents.max.storage[1];
        }
        if (maxZ < extents.max.storage[2]) {
          maxZ = extents.max.storage[2];
        }
        /*jshint white: true*/
        lastNodeIndex += 1;
        buildNode.externalNode.aabbTreeIndex = lastNodeIndex;
        this.replaceNode(nodes, lastNodeIndex, buildNode);
      }

      lastNode = nodes[lastNodeIndex];
    } else {
      /*jshint bitwise: false*/
      var splitPosIndex = ((startIndex + endIndex) >> 1);
      /*jshint bitwise: true*/

      if ((startIndex + 1) >= splitPosIndex)
       {
        buildNode = buildNodes[startIndex];
        buildNode.externalNode.aabbTreeIndex = lastNodeIndex;
        this.replaceNode(nodes, lastNodeIndex, buildNode);
      }
       else
       {
        this.recursiveBuild(buildNodes, startIndex, splitPosIndex, lastNodeIndex);
      }

      lastNode = nodes[lastNodeIndex];
      extents = lastNode.extents;
      minX = extents.min.storage[0];
      minY = extents.min.storage[1];
      minZ = extents.min.storage[2];
      maxX = extents.max.storage[0];
      maxY = extents.max.storage[1];
      maxZ = extents.max.storage[2];

      lastNodeIndex = (lastNodeIndex + lastNode.escapeNodeOffset);

      if ((splitPosIndex + 1) >= endIndex)
       {
        buildNode = buildNodes[splitPosIndex];
        buildNode.externalNode.aabbTreeIndex = lastNodeIndex;
        this.replaceNode(nodes, lastNodeIndex, buildNode);
      }
       else
       {
        this.recursiveBuild(buildNodes, splitPosIndex, endIndex, lastNodeIndex);
      }

      lastNode = nodes[lastNodeIndex];
      extents = lastNode.extents;
      /*jshint white: false*/
      if (minX > extents.min.storage[0]) {
        minX = extents.min.storage[0];
      }
      if (minY > extents.min.storage[1]) {
        minY = extents.min.storage[1];
      }
      if (minZ > extents.min.storage[2]) {
        minZ = extents.min.storage[2];
      }
      if (maxX < extents.max.storage[0]) {
        maxX = extents.max.storage[0];
      }
      if (maxY < extents.max.storage[1]) {
        maxY = extents.max.storage[1];
      }
      if (maxZ < extents.max.storage[2]) {
        maxZ = extents.max.storage[2];
      }
    /*jshint white: true*/
    }

    var node = nodes[nodeIndex];
    if (node != null) {
      node.reset(minX, minY, minZ, maxX, maxY, maxZ,
       (lastNodeIndex + lastNode.escapeNodeOffset - nodeIndex));
    } else {
      Aabb3 parentExtents = new Aabb3();
      //print(minX);
      parentExtents.min.storage[0] = minX;
      parentExtents.min.storage[1] = minY;
      parentExtents.min.storage[2] = minZ;
      parentExtents.max.storage[0] = maxX;
      parentExtents.max.storage[1] = maxY;
      parentExtents.max.storage[2] = maxZ;

      nodes[nodeIndex] = new AabbTreeNode(parentExtents,
       (lastNodeIndex + lastNode.escapeNodeOffset - nodeIndex));
    }
  }
  void replaceNode(List<AabbTreeNode> nodes, int nodeIndex, AabbTreeNode newNode) {
    var oldNode = nodes[nodeIndex];
    nodes[nodeIndex] = newNode;
    if (oldNode != null) {
      do {
        nodeIndex += 1;
      } while (nodes[nodeIndex] != null);
      nodes[nodeIndex] = oldNode;
    }
  }
  int predictNumNodes(int startIndex, int endIndex, int lastNodeIndex) {
    lastNodeIndex += 1;

    if ((startIndex + this.numNodesLeaf) >= endIndex)
     {
      lastNodeIndex += (endIndex - startIndex);
    }
     else
     {
      var splitPosIndex = ((startIndex + endIndex) >> 1);

      if ((startIndex + 1) >= splitPosIndex)
       {
        lastNodeIndex += 1;
      }
       else
       {
        lastNodeIndex = this.predictNumNodes(startIndex, splitPosIndex, lastNodeIndex);
      }

      if ((splitPosIndex + 1) >= endIndex)
       {
        lastNodeIndex += 1;
      }
       else
       {
        lastNodeIndex = this.predictNumNodes(splitPosIndex, endIndex, lastNodeIndex);
      }
    }
    return lastNodeIndex;
  }
  int getVisibleNodes(List<Plane> planes, List<ExternalAabbTreeNode> visibleNodes, [int startIndex]) {
    var numVisibleNodes = 0;
    if (this.numExternalNodes > 0)
     {
      var nodes = this.nodes;
      var endNodeIndex = this.endNode;
      var numPlanes = planes.length;
      var storageIndex = (startIndex == null) ? visibleNodes.length : startIndex;
      var node, extents, endChildren;
      var n0, n1, n2, p0, p1, p2;
      var isInside, n, plane, d0, d1, d2;
      var nodeIndex = 0;

      for (; ;)
       {
        node = nodes[nodeIndex];
        extents = node.extents;
        n0 = extents.min.storage[0];
        n1 = extents.min.storage[1];
        n2 = extents.min.storage[2];
        p0 = extents.min.storage[0];
        p1 = extents.min.storage[1];
        p2 = extents.min.storage[2];
        //isInsidePlanesAABB
        isInside = true;
        n = 0;
        do {
          plane = planes[n];
          d0 = plane.normal.storage[0];
          d1 = plane.normal.storage[1];
          d2 = plane.normal.storage[2];
          if ((d0 * (d0 < 0 ? n0 : p0) + d1 * (d1 < 0 ? n1 : p1) + d2 * (d2 < 0 ? n2 : p2)) < plane.constant/*[3]*/ )
           {
            isInside = false;
            break;
          }
          n += 1;
        } while (n < numPlanes);
        if (isInside) {
          if (node.externalNode)  // Is leaf
          {
            visibleNodes[storageIndex] = node.externalNode;
            storageIndex += 1;
            numVisibleNodes += 1;
            nodeIndex += 1;
            if (nodeIndex >= endNodeIndex)
             {
              break;
            }
          } else {
            //isFullyInsidePlanesAABB
            isInside = true;
            n = 0;
            do
             {
              plane = planes[n];
              d0 = plane.normal.storage[0];
              d1 = plane.normal.storage[1];
              d2 = plane.normal.storage[2];
              if ((d0 * (d0 > 0 ? n0 : p0) + d1 * (d1 > 0 ? n1 : p1) + d2 * (d2 > 0 ? n2 : p2)) < plane.constant/*[3]*/ )
               {
                isInside = false;
                break;
              }
              n += 1;
            }
             while (n < numPlanes);
            if (isInside)
             {
              endChildren = (nodeIndex + node.escapeNodeOffset);
              nodeIndex += 1;
              do {
                node = nodes[nodeIndex];
                if (node.externalNode != null)  // Is leaf
                {
                  visibleNodes[storageIndex] = node.externalNode;
                  storageIndex += 1;
                  numVisibleNodes += 1;
                }
                nodeIndex += 1;
              } while (nodeIndex < endChildren);
              if (nodeIndex >= endNodeIndex) {
                break;
              }
            } else {
              nodeIndex += 1;
            }
          }
        } else {
          nodeIndex += node.escapeNodeOffset;
          if (nodeIndex >= endNodeIndex) {
            break;
          }
        }
      }
    }
    return numVisibleNodes;
  }
  int getOverlappingNodes(Aabb3 queryExtents, List<AabbTreeNode> overlappingNodes, [int startIndex]) {
    if (this.numExternalNodes > 0)
     {
      var queryMinX = queryExtents.min.storage[0];
      var queryMinY = queryExtents.min.storage[1];
      var queryMinZ = queryExtents.min.storage[2];
      var queryMaxX = queryExtents.max.storage[0];
      var queryMaxY = queryExtents.max.storage[1];
      var queryMaxZ = queryExtents.max.storage[2];
      var nodes = this.nodes;
      var endNodeIndex = this.endNode;
      var node, extents, endChildren;
      var numOverlappingNodes = 0;
      var storageIndex = (startIndex == null) ? overlappingNodes.length : startIndex;
      var nodeIndex = 0;
      for (; ;)
       {
        node = nodes[nodeIndex];
        extents = node.extents;
        var minX = extents.min.storage[0];
        var minY = extents.min.storage[1];
        var minZ = extents.min.storage[2];
        var maxX = extents.max.storage[0];
        var maxY = extents.max.storage[1];
        var maxZ = extents.max.storage[2];
        if (queryMinX <= maxX &&
         queryMinY <= maxY &&
         queryMinZ <= maxZ &&
         queryMaxX >= minX &&
         queryMaxY >= minY &&
         queryMaxZ >= minZ)
         {
          if (node.externalNode != null) { // Is leaf
            // fix Range
            overlappingNodes.length = storageIndex + 1;
            //
            overlappingNodes[storageIndex] = node.externalNode;
            storageIndex += 1;
            numOverlappingNodes += 1;
            nodeIndex += 1;
            if (nodeIndex >= endNodeIndex)
             {
              break;
            }
          }
           else
           {
            if (queryMaxX >= maxX &&
             queryMaxY >= maxY &&
             queryMaxZ >= maxZ &&
             queryMinX <= minX &&
             queryMinY <= minY &&
             queryMinZ <= minZ)
             {
              endChildren = (nodeIndex + node.escapeNodeOffset);
              nodeIndex += 1;
              do
               {
                node = nodes[nodeIndex];
                if (node.externalNode != null)  // Is leaf
                {
                  overlappingNodes[storageIndex] = node.externalNode;
                  storageIndex += 1;
                  numOverlappingNodes += 1;
                }
                nodeIndex += 1;
              }
               while (nodeIndex < endChildren);
              if (nodeIndex >= endNodeIndex)
               {
                break;
              }
            }
             else
             {
              nodeIndex += 1;
            }
          }
        }
         else
         {
          nodeIndex += node.escapeNodeOffset;
          if (nodeIndex >= endNodeIndex)
           {
            break;
          }
        }
      }
      return numOverlappingNodes;
    }
     else
     {
      return 0;
    }
  }
  void getSphereOverlappingNodes(Vector3 center, double radius, List<AabbTreeNode> overlappingNodes) {
    if (this.numExternalNodes > 0) {
      var radiusSquared = (radius * radius);
      var centerX = center.storage[0];
      var centerY = center.storage[1];
      var centerZ = center.storage[2];
      var nodes = this.nodes;
      var endNodeIndex = this.endNode;
      var node, extents;
      var numOverlappingNodes = overlappingNodes.length;
      var nodeIndex = 0;
      for (; ;)
       {
        node = nodes[nodeIndex];
        extents = node.extents;
        var minX = extents.min.storage[0];
        var minY = extents.min.storage[1];
        var minZ = extents.min.storage[2];
        var maxX = extents.max.storage[0];
        var maxY = extents.max.storage[1];
        var maxZ = extents.max.storage[2];
        var totalDistance = 0, sideDistance;
        if (centerX < minX)
         {
          sideDistance = (minX - centerX);
          totalDistance += (sideDistance * sideDistance);
        }
         else {
          if (centerX > maxX)
           {
            sideDistance = (centerX - maxX);
            totalDistance += (sideDistance * sideDistance);
          }
        }
        if (centerY < minY)
         {
          sideDistance = (minY - centerY);
          totalDistance += (sideDistance * sideDistance);
        }
         else {
          if (centerY > maxY)
           {
            sideDistance = (centerY - maxY);
            totalDistance += (sideDistance * sideDistance);
          }
        }
        if (centerZ < minZ)
         {
          sideDistance = (minZ - centerZ);
          totalDistance += (sideDistance * sideDistance);
        }
         else {
          if (centerZ > maxZ)
           {
            sideDistance = (centerZ - maxZ);
            totalDistance += (sideDistance * sideDistance);
          }
        }
        if (totalDistance <= radiusSquared)
         {
          nodeIndex += 1;
          if (node.externalNode != null)  // Is leaf
          {
            overlappingNodes[numOverlappingNodes] = node.externalNode;
            numOverlappingNodes += 1;
            if (nodeIndex >= endNodeIndex)
             {
              break;
            }
          }
        } else {
          nodeIndex += node.escapeNodeOffset;
          if (nodeIndex >= endNodeIndex)
           {
            break;
          }
        }
      }
    }
  }
  int getOverlappingPairs(List<AabbTreeNode> overlappingPairs, int startIndex) {
    if (this.numExternalNodes > 0) {
      var nodes = this.nodes;
      var endNodeIndex = this.endNode;
      var currentNode, currentExternalNode, node, extents;
      var numInsertions = 0;
      var storageIndex = (startIndex == null) ? overlappingPairs.length : startIndex;
      var currentNodeIndex = 0, nodeIndex;
      for (; ;)
       {
        currentNode = nodes[currentNodeIndex];
        while (currentNode.externalNode == null)  // No leaf
        {
          currentNodeIndex += 1;
          currentNode = nodes[currentNodeIndex];
        }

        currentNodeIndex += 1;
        if (currentNodeIndex < endNodeIndex)
         {
          currentExternalNode = currentNode.externalNode;
          extents = currentNode.extents;
          var minX = extents.min.storage[0];
          var minY = extents.min.storage[1];
          var minZ = extents.min.storage[2];
          var maxX = extents.max.storage[0];
          var maxY = extents.max.storage[1];
          var maxZ = extents.max.storage[2];

          nodeIndex = currentNodeIndex;
          for (; ;)
           {
            node = nodes[nodeIndex];
            extents = node.extents;
            if (minX <= extents.max.storage[0] &&
             minY <= extents.max.storage[1] &&
             minZ <= extents.max.storage[2] &&
             maxX >= extents.min.storage[0] &&
             maxY >= extents.min.storage[1] &&
             maxZ >= extents.min.storage[2])
             {
              nodeIndex += 1;
              if (node.externalNode != null)  // Is leaf
              {
                overlappingPairs.add(null);
                overlappingPairs.add(null);
                overlappingPairs[storageIndex] = currentExternalNode;
                overlappingPairs[storageIndex + 1] = node.externalNode;
                //overlappingPairs.add(currentExternalNode);
                //overlappingPairs.add(node.externalNode);
                storageIndex += 2;
                numInsertions += 2;
                if (nodeIndex >= endNodeIndex)
                 {
                  break;
                }
              }
            } else {
              nodeIndex += node.escapeNodeOffset;
              if (nodeIndex >= endNodeIndex) {
                break;
              }
            }
          }
        } else {
          break;
        }
      }
      return numInsertions;
    } else {
      return 0;
    }
  }
  AabbTreeNode getRootNode() {
    return this.nodes[0];
  }
  List<AabbTreeNode> getNodes() {
    return this.nodes;
  }
  int getEndNodeIndex() {
    return this.endNode;
  }
  void clear() {
    this.nodes = [];
    this.endNode = 0;
    this.needsRebuild = false;
    this.needsRebound = false;
    this.numAdds = 0;
    this.numUpdates = 0;
    this.numExternalNodes = 0;
    this.startUpdate = 0x7FFFFFFF;
    this.endUpdate = -0x7FFFFFFF;
  }
  static rayTest(var trees, AABBTreeRay ray, AABBTreeRayTestResult callback) {
    // convert ray to parametric form
    var origin = ray.origin;
    var direction = ray.direction;

    // values used throughout calculations.
    var o0 = origin[0];
    var o1 = origin[1];
    var o2 = origin[2];
    var d0 = direction[0];
    var d1 = direction[1];
    var d2 = direction[2];
    var id0 = 1 / d0;
    var id1 = 1 / d1;
    var id2 = 1 / d2;

    // evaluate distance factor to a node's extents from ray origin, along direction
    // use this to induce an ordering on which nodes to check.
    double distanceExtents(Aabb3 extents, double upperBound) {
      var min0 = extents.min.storage[0];
      var min1 = extents.min.storage[1];
      var min2 = extents.min.storage[2];
      var max0 = extents.max.storage[0];
      var max1 = extents.max.storage[1];
      var max2 = extents.max.storage[2];

      // treat origin internal to extents as 0 distance.
      if (min0 <= o0 && o0 <= max0 &&
       min1 <= o1 && o1 <= max1 &&
       min2 <= o2 && o2 <= max2)
       {
        return 0.0;
      }

      double tmin, tmax;
      double tymin, tymax;
      var del;
      if (d0 >= 0.0)
       {
        // Deal with cases where d0 == 0
        del = (min0 - o0);
        tmin = ((del == 0.0) ? 0.0 : (del * id0));
        del = (max0 - o0);
        tmax = ((del == 0.0) ? 0.0 : (del * id0));
      }
       else
       {
        tmin = ((max0 - o0) * id0);
        tmax = ((min0 - o0) * id0);
      }

      if (d1 >= 0)
       {
        // Deal with cases where d1 == 0
        del = (min1 - o1);
        tymin = ((del == 0.0) ? 0.0 : (del * id1));
        del = (max1 - o1);
        tymax = ((del == 0.0) ? 0.0 : (del * id1));
      }
       else
       {
        tymin = ((max1 - o1) * id1);
        tymax = ((min1 - o1) * id1);
      }

      if ((tmin > tymax) || (tymin > tmax))
       {
        return null;
      }

      if (tymin > tmin)
       {
        tmin = tymin;
      }

      if (tymax < tmax)
       {
        tmax = tymax;
      }

      var tzmin, tzmax;
      if (d2 >= 0.0) {
        // Deal with cases where d2 == 0
        del = (min2 - o2);
        tzmin = ((del == 0.0) ? 0.0 : (del * id2));
        del = (max2 - o2);
        tzmax = ((del == 0.0) ? 0.0 : (del * id2));
      } else {
        tzmin = ((max2 - o2) * id2);
        tzmax = ((min2 - o2) * id2);
      }

      if ((tmin > tzmax) || (tzmin > tmax))
       {
        return null;
      }

      if (tzmin > tmin)
       {
        tmin = tzmin;
      }

      if (tzmax < tmax)
       {
        tmax = tzmax;
      }

      if (tmin < 0.0)
       {
        tmin = tmax;
      }

      return (0.0 <= tmin && tmin < upperBound) ? tmin : null;
    }

    // we traverse both trees at once
    // keeping a priority list of nodes to check next.

    // TODO: possibly implement priority list more effeciently?
    //       binary heap probably too much overhead in typical case.
    List<PriorityNode> priorityList = <PriorityNode>[];
    //current upperBound on distance to first intersection
    //and current closest object properties
    var minimumResult = null;

    //if node is a leaf, intersect ray with shape
    // otherwise insert node into priority list.
    double processNode(AabbTree tree, int nodeIndex, double upperBound) {
      var nodes = tree.getNodes();
      var node = nodes[nodeIndex];
      var distance = distanceExtents(node.extents, upperBound);
      if (distance == null) {
        return upperBound;
      }

      if (node.externalNode != null) {
        var result = callback(tree, node.externalNode, ray, distance, upperBound);
        if (result != null) {
          minimumResult = result;
          upperBound = result.factor;
        }
      } else {
        // TODO: change to binary search?
        var length = priorityList.length;
        var i;
        for (i = 0; i < length; i += 1) {
          var curObj = priorityList[i];
          if (distance > curObj.distance)
           {
            break;
          }
        }

        //insert node at index i
        priorityList.insert(i, new PriorityNode(tree, nodeIndex, distance));
      /*
           priorityList.splice(i - 1, 0, {
             tree: tree,
             nodeIndex: nodeIndex,
             distance: distance
           });*/
      }

      return upperBound;
    }

    var upperBound = ray.maxFactor;

    var tree;
    var i;
    for (i = 0; i < trees.length; i += 1) {
      tree = trees[i];
      if (tree.endNode != 0) {
        upperBound = processNode(tree, 0, upperBound);
      }
    }

    while (priorityList.length != 0) {
      var nodeObj = priorityList.removeLast();
      // A node inserted into prioority list after this one may have
      // moved the upper bound.
      if (nodeObj.distance >= upperBound) {
        continue;
      }

      var nodeIndex = nodeObj.nodeIndex;
      tree = nodeObj.tree;
      var nodes = tree.getNodes();

      var node = nodes[nodeIndex];
      var maxIndex = nodeIndex + node.escapeNodeOffset;

      var childIndex = nodeIndex + 1;
      do {
        upperBound = processNode(tree, childIndex, upperBound);
        childIndex += nodes[childIndex].escapeNodeOffset;
      } while (childIndex < maxIndex);
    }

    return minimumResult;
  }

/*static AABBTree create(bool highQuality) {
      return new AABBTree(highQuality ? true : false);
    }*/
}
/*

// Detect correct typed arrays
(dynamic () {
    AABBTree.prototype.arrayConstructor = Array;
    if (typeof Float32Array != null)
    {
        var testArray = new Float32Array(4);
        var textDescriptor = Object.prototype.toString.call(testArray);
        if (textDescriptor == '[object Float32Array]')
        {
            AABBTree.prototype.arrayConstructor = Float32Array;
        }
    }
}());
*/
