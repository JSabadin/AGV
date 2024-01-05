#!/usr/bin/python3
# -*- coding: utf-8 -*-
try:
  import matplotlib as mpl
  if __name__ == '__main__':
    mpl.use('Agg')
  import matplotlib.pyplot as plt
except:
  class FakePlt(object):
    def __init__(self):
      self.figure = None
  plt = FakePlt()
import numpy as np
from math import *
import world
import rospkg
import os
import time
#import pickle
from ams import wrapToPi



TEXT_OFFSET = 0.015



class Edge(object):
  def __init__(self, data=[], name=''):
    self.p = np.zeros((3, 0), dtype=float)
    self.length = 0.0
    self.name = name
    self.add(data)
    
  def add(self, data):
    for d in data:
      self.addOne(d)
  
  def addOne(self, data):
    special = False
    if data[0] == 'L':
      p = self._makeSegment(data[1:])
    elif data[0] == 'A':
      p = self._makeArc(data[1:])
    elif data[0] == 'B':
      p = self._makeBezier(data[1:])
    elif data[0] == 'X':
      p = self._makeSegment(data[1:])
      special = True
    else:
      p = None
    
    if p is not None:
      if special:
        p = np.vstack((p, np.zeros((1,p.shape[1]))))
      else:
        p = np.vstack((p, np.ones((1,p.shape[1]))))
      if self.p.shape[1] > 0:
        self.p = np.hstack((self.p, p[:,1:]))
      else:
        self.p = p
        
      self.length += Edge.getLength(p)

  def setPoints(self, p, preserve=False):
    self.p = np.copy(p)
    if not preserve:
      self.p[2,:] = np.ones((1, self.p.shape[1]))
    self.length = Edge.getLength(self.p)
      
  @staticmethod
  def getLength(p):
    return np.sum(np.sqrt(np.sum(np.diff(p[0:2,:])**2, 0)))
  
  def draw(self, axes=plt, c='k', off=0.0, lw=3, alpha=1.0, direct=False, dirs=False):
    if np.any(self.p[2,:] < 0.5):
      d = np.abs(np.diff(self.p[2,:])) > 0.5
      x = list(np.where(d)[0])
      x.append(d.size)
      x.insert(0, 0)
      state = int(self.p[2,0])
      for i in range(len(x)-1):
        a = x[i]
        b = x[i+1]+1
        p = self.p[0:2,a:b]
        axes.plot(p[0,:]+off, p[1,:]+off, ls='-' if state == 1 else ':', c=c, lw=lw if state == 1 else 1, alpha=alpha, solid_capstyle='round')
        state = 0 if state else 1  
    else:
      if direct:
        #axes.plot(self.p[0,[0,-1]]+off, self.p[1,[0,-1]]+off, ls='-', c=c, lw=lw, alpha=alpha, solid_capstyle='round')
        
        axes.arrow(self.p[0,0], self.p[1,0], self.p[0,-1]-self.p[0,0], self.p[1,-1]-self.p[1,0], width=0.01, color=c, zorder=2, alpha=alpha, length_includes_head=True, head_width=0.04, head_length=0.06, overhang=0)
      else:
        axes.plot(self.p[0,:]+off, self.p[1,:]+off, ls='-', c=c, lw=lw, alpha=alpha, solid_capstyle='round')
        
        if dirs:
          n = floor(self.p.shape[1]/2)
          if n > 0:
            d = 20*sqrt((self.p[0,n]-self.p[0,n-1])**2 + (self.p[1,n]-self.p[1,n-1])**2)
            axes.arrow(self.p[0,n-1], self.p[1,n-1], (self.p[0,n]-self.p[0,n-1])/d, (self.p[1,n]-self.p[1,n-1])/d, width=0.01, color=(0.3, 0.3, 0.3), zorder=2, alpha=0.5, length_includes_head=True, head_width=0.02, head_length=0.03, overhang=0)
  
  def _makeArc(self, data):
    N = int(max(2, round(24*abs(data[4])/360.0)))
    t = np.linspace(data[3], data[3]+data[4], N)/180*pi
    x = data[0] + data[2]*np.cos(t)
    y = data[1] + data[2]*np.sin(t)
    return np.array([x, y])
    
  def _makeBezier(self, data):
    N = 12
    cp = np.array(data)
    cp = cp.reshape((-1,2)).T
    t = np.linspace(0, 1, N)
    t = np.vstack((t, t))
    p = np.zeros((2, N), dtype=float)
    for i, g in enumerate([1, 3, 3, 1]):
      p += g*(t**i)*((1-t)**(3-i))*cp[:,i:i+1]
    return p
    
  def _makeSegment(self, data):
    p = np.array(data)
    p = p.reshape((-1,2)).T
    return p
    
  @staticmethod
  def segmentDistance(p, q, r):
    '''Given the point $p$ and a segment defined by the end points $q$ and $r$,
       it calculates the distance $d$ to the nearest point $c$ on the segment.'''
    a = r - q
    b = p - q
    aa = a.T.dot(a)
    e = (a.T.dot(b))/aa
    if e < 0:
        c = q
        e = 0.0
    elif e > 1:
        c = r
        e = 1.0
    else:
        c = q + a*e
    
    f = sqrt(aa)
    v = p - c
    d = sqrt(v.T.dot(v))
    return (d, c, e, f)
    
  def pointToDistance(self, point):
    dMin=1.0
    sOpt = None
    s = 0.0
    for i in range(self.p.shape[1]-1):
      d, c, e, f = Edge.segmentDistance(point, self.p[0:2,i], self.p[0:2,i+1])
      if d < dMin:
        dMin = d
        sOpt = s + e*f
      s += f
    return sOpt
  
  def getPartNorm(self, a=0.0, b=1.0):
    return self.getPart(a*self.length, b*self.length)
    
  def getPart(self, a=0.0, b=None):
    if b is None:
      b = self.length
    inside = False
    s = 0.0
    p = np.zeros((3,0), dtype=float)
    if b >= a:
      for i in range(self.p.shape[1]-1):
        e = self.p[0:2,i+1:i+2] - self.p[0:2,i:i+1]
        f = sqrt(sum(e**2))
        s += f
        if not inside and s >= a:
          inside = True
          d = a - s + f
          r = self.p[:,i:i+1]
          r = np.vstack((r[0:2,:] + d/f*e, r[2,:]))
          p = np.hstack((p, r))
        if inside:
          if s >= b:
            # Last point
            d = b - s + f
            r = self.p[:,i:i+1]
            r = np.vstack((r[0:2,:] + d/f*e, r[2,:]))
            p = np.hstack((p, r))
            break
          else:
            # Znotraj segmenta
            p = np.hstack((p, self.p[:,i+1:i+2]))
      return p
    
  def simulate(self):
    D = 0.124
    S = 0.060

    tt = np.linspace(0, pi/2, 6)
    #ref = np.vstack((100 + 75*np.cos(tt), 200 + 75*np.sin(tt)))
    ref = self.p
    
    r = 0
    fi0 = atan2(np.diff(ref[1,0:2])[0], np.diff(ref[0,0:2])[0])
    fi0 = round(fi0/pi*2)*pi/2
    q0 = np.array([ref[0,0] - D*cos(fi0),
                   ref[1,0] - D*sin(fi0),
                   fi0,
                   fi0])
    qF0 = np.array([q0[0] + D*cos(q0[2]), q0[1] + D*sin(q0[2])])
    qS0 = np.array([q0[0] + S*cos(q0[2]), q0[1] + S*sin(q0[2])])
    q = q0
    qF = qF0
    qS = qS0
    Q = np.array([np.hstack((q, qF, qS))])

    while True:
      ex = ref[0,r] - qF[0]
      ey = ref[1,r] - qF[1]
      ed = sqrt(ex**2+ey**2)
      ea = atan2(ey, ex) - q[3]
      ea = wrapToPi(ea)
      
      if ed < 0.002:
        r = r + 1
        if r >= ref.shape[1]:
            break
      
      Kd = 0.5
      Ka = 2
      
      v = Kd*ed
      if v > 0.005:
          v = 0.005
      v = v*cos(ea)
      w = v*sin(q[3]-q[2])/D + Ka*ea
      
      q[0] += v*cos(q[2])*cos(q[3] - q[2])
      q[1] += v*sin(q[2])*cos(q[3] - q[2])
      q[2] += v*sin(q[3] - q[2])/D
      q[3] += w
      q[2] = wrapToPi(q[2])
      q[3] = wrapToPi(q[3])
      
      qF = np.array([q[0] + D*cos(q[2]), q[1] + D*sin(q[2])])
      qS = np.array([q[0] + S*cos(q[2]), q[1] + S*sin(q[2])])
      
      dQ = np.hstack((q, qF, qS))
      Q = np.vstack((Q, dQ))
    return Q

  @staticmethod
  def createEdges(data=None):
    edges = {}
    if data is not None:
      for name, d in data.items():
        edge = Edge(data=d, name=name)
        edges[name] = edge
    return edges

  @staticmethod
  def drawEdges(edges, **kwargs):
    for edge in edges.values():
      edge.draw(**kwargs)

  @staticmethod
  def connectNodes(edges, nodes):
    connections = {}
    for name, edge in edges.items():
      start = None
      end = None
      for n, node in nodes.items():
        if np.sum((edge.p[0:2,0]-node.p)**2) < 0.000004:
          start = n
        if np.sum((edge.p[0:2,-1]-node.p)**2) < 0.000004:
          end = n
        if start is not None and end is not None:
          break
      connections[name] = (start, end)
    return connections

  @staticmethod
  def snapToEdge(edges, point, filter=None):
    if filter is None:
      filter = edges.keys()
    
    dMin=1.0
    opt = None
    for name in filter:
      edge = edges[name]
      s = 0.0
      for i in range(edge.p.shape[1]-1):
        q = edge.p[0:2,i]
        r = edge.p[0:2,i+1]
        d, c, e, f = Edge.segmentDistance(point, q, r)
        if d < dMin:
          dMin = d
          dd = r - q
          phi = atan2(dd[1], dd[0])
          opt = (name, s + e*f, (c[0], c[1], phi), d)
        s += f
    return opt



class Node(object):
  def __init__(self, p=(0.0, 0.0), name=''):
    self.p = p
    self.name = name

  def isVirtual(self):
    return self.name > 100

  def draw(self, axes=plt):
    axes.plot(self.p[0], self.p[1], 'ko', ms=10)
    axes.text(self.p[0]-TEXT_OFFSET, self.p[1]-TEXT_OFFSET, '{}'.format(self.name), ha='right', va='top', fontsize=20)

  @staticmethod
  def createNodes(data=None):
    nodes = {}
    if data is not None:
      for name, p in data.items():
        nodes[name] = Node(p=p, name=name)
    return nodes

  @staticmethod
  def drawNodes(nodes, **kwargs):
    for node in nodes.values():
      node.draw(**kwargs)

  @staticmethod
  def connectEdges(nodes, edges):
    connections = {}
    start = {}
    end = {}
    for n, node in nodes.items():
      start[n] = []
      end[n] = []
      for edge in edges.values():
        if np.sum((edge.p[0:2,0]-node.p)**2) < 0.000004:
          start[n].append(edge.name)
        if np.sum((edge.p[0:2,-1]-node.p)**2) < 0.000004:
          end[n].append(edge.name)
      connections[n] = (start[n], end[n])
    return connections



class Graph(object):
  def __init__(self): #, force=False
    #rp = rospkg.RosPack()
    #path = os.path.join(rp.get_path('amsagv'), 'scripts', 'world.pkl')
    #if not force and os.path.exists(path):
    #  with open(path) as f:
    #    d = pickle.load(f)
    #    
    #    self.edges = d.edges
    #    self.nodes = d.nodes
    #    self.tags = d.tags
    #    self.edgeNodes = d.edgeNodes
    #    self.nodeEdges = d.nodeEdges
    #    
    #    self.detTags = d.detTags
    #    self.detEdges = d.detEdges
    #    self.links = d.links
    #    self.tagPoses = d.tagPoses
    #    self.tagMap = d.tagMap
    #else:
      self.edges = Edge.createEdges(world.EDGES)
      self.nodes = Node.createNodes(world.NODES)
      self.tags = Node.createNodes(world.TAGS)
      self.edgeNodes = Edge.connectNodes(self.edges, self.nodes)
      self.nodeEdges = Node.connectEdges(self.nodes, self.edges)
      
      self.compute()
      
    #  with open(path, 'w') as f:
    #    pickle.dump(self, f)
    
  def createDot1(self, dot, positions=False):
    with open('{}.dot'.format(dot), 'w') as out:
      out.write('digraph space {\n')
      out.write('  graph [resolution=200, start=21, splines=polyline, esep=0.5, sep=0.2, overlap=scale];\n')
      out.write('  node [fontsize=10, penwidth=1, style=filled, fillcolor="#EEEEEE", shape=circle, width=0.35, height=0.35, fixedsize=true];\n')
      out.write('  edge [fontsize=10, penwidth=2, arrowsize=0.5];\n')
      
      out.write('\n  # Nodes\n')
      for n, p in self.nodes.items():
        out.write('  {}'.format(n))
        if positions:
          out.write(' [pos="{},{}!"]'.format(p.p[0], p.p[1]))
        out.write(';\n')
      
      out.write('\n  # Connections\n')
      for n, c in self.edges.items():
        a = n.split('-')
        out.write('  {} -> {}'.format(*a))
        out.write(';\n')
        
      out.write('}')
        
  def createDot2(self, dot, positions=False):
    with open('{}.dot'.format(dot), 'w') as out:
      out.write('digraph routes {\n')
      out.write('  graph [resolution=200, start=11, splines=polyline, esep=0.5, sep=0.2, overlap=scale];\n')
      out.write('  node [fontsize=10, penwidth=1, style=filled, fillcolor="#EEEEEE", shape=circle, width=0.35, height=0.35, fixedsize=true];\n')
      out.write('  edge [fontsize=10, penwidth=2, arrowsize=0.5];\n')
      
      out.write('\n  # Nodes\n')
      for tagName, tag in self.tags.items():
        c = '"#FFFF0080"' if tag.isVirtual() else '"#00FFFF80"'
        out.write('  {}'.format(tagName))
        if positions:
          out.write(' [pos="{},{}!", fillcolor={}]'.format(tag.p[0], tag.p[1], c))
        else:
          out.write(' [fillcolor={}]'.format(c))
        out.write(';\n')
      
      out.write('\n  # Connections\n')
      for k, vv in self.links.items():
        for v in vv:
          a = v[2].split('-')
          out.write('  {} -> {}'.format(*a))
          
          if v[0] == 'X':
            c = '"#00AA0080"'
          elif v[0] == 'L':
            c = '"#0000FF80"'
          elif v[0] == 'R':
            c = '"#FF000080"'
          else:
            c = 'black'
            
          e = self.detEdges[v[2]]
          
          out.write(' [label="{:.3f}", color={}]'.format(e.length, c))
          out.write(';\n')
        
      out.write('}')

  def drawAxes(self):
    self.axes.set_xlabel('$x$ [m]')
    self.axes.set_ylabel('$y$ [m]')
    self.axes.set_aspect('equal')
    self.axes.set_xlim(world.BOX[0:3:2])
    self.axes.set_ylim(world.BOX[1:4:2])
    self.axes.set_facecolor((0.9, 0.1, 0.9))
    self.axes.set_axis_off()
    self.axes.set_position((0, 0, 1, 1))
    
  def compute(self):
    # Simulations (move to static Edge method)
    sim = {}
    for edgeName, edge in self.edges.items():
      sim[edgeName] = edge.simulate()
      
    self.detTags = Node.createNodes(world.TAGS)

    # Find detected tags and edge breaks
    edgeBreaks = {}
    SIGMA_VIRTUAL = 0.005
    SIGMA_REAL = 0.020
    SIGMA_MAX = max(SIGMA_REAL, SIGMA_VIRTUAL)
    for tagName, tag in self.tags.items():
      opt = (None, SIGMA_MAX**2, (0, 0))
      for edgeName, edge in self.edges.items():
        for i in range(sim[edgeName].shape[0]):
          qS = sim[edgeName][i,6:8]
          qF = sim[edgeName][i,4:6]
          if tag.isVirtual():
            d = np.sum((qF - tag.p)**2)
            tol = SIGMA_VIRTUAL**2
          else:
            d = np.sum((qS - tag.p)**2)
            tol = SIGMA_REAL**2
          if d < tol and d < opt[1]:
            opt = (edge.name, d, (qF[0], qF[1]))
      
      if opt[0] is not None:
        self.detTags[tagName].p = opt[2]
        edge = self.edges[opt[0]]
        d = edge.pointToDistance(opt[2])
        a = (tagName, d)
        if opt[0] not in edgeBreaks:
          edgeBreaks[opt[0]] = []
        edgeBreaks[opt[0]].append(a)

    # Sort edge breaks
    for edgeBreak in edgeBreaks.values():
      edgeBreak.sort(key=lambda x: x[1])

    # Find end parts
    partsEnd = {}
    for k, line in edgeBreaks.items():
      line.sort(key=lambda x: x[1])
      partsEnd[k] = (line[-1][0], self.edges[k].length-line[-1][1])

    # Find start parts
    partsStart = {}
    for k, line in edgeBreaks.items():
      line.sort(key=lambda x: x[1])
      partsStart[k] = (line[0][0], line[0][1])

    self.detEdges = Edge.createEdges()
    
    tagEdges = {}
    for k, line in edgeBreaks.items():
      # Find edges between the tags on the same line
      if len(line)>1:
        for l in range(len(line)-1):
          a = line[l][0]
          b = line[l+1][0]
          lab = line[l+1][1] - line[l][1]
          tagEdges[a] = [(b, lab)]
          
          edge = Edge()
          edge.name = '{}-{}'.format(a, b)
          edge.setPoints(self.edges[k].getPart(line[l][1], line[l+1][1]))
          self.detEdges[edge.name] = edge
      end = self.edgeNodes[k][1]
      start = []
      for t, n in self.edgeNodes.items():
        if n[0] == end:
          start.append(t)
      
      a = partsEnd[k]
      tagEdges[a[0]] = []
      for s in start:
        if s in partsStart:
          b = partsStart[s]
          tagEdges[a[0]].append((b[0], a[1] + b[1]))
          
          pa = self.edges[k].getPart(self.edges[k].length-a[1])
          pb = self.edges[s].getPart(0.0, b[1])
          edge = Edge()
          edge.name = '{}-{}'.format(a[0], b[0])
          edge.setPoints(np.hstack((pa, pb[:,1:])))
          self.detEdges[edge.name] = edge
          
    # Clean duplicated points
    for n, e in self.detEdges.items():
      z = abs(np.sum((e.p[0:2,1:] - e.p[0:2,0:-1])**2, axis=0)) > 1e-6
      if not np.all(z):
        z = np.hstack((True, z))
        e.p = e.p[:,z]
          
    detEdgeNodes = Edge.connectNodes(self.detEdges, self.detTags)
    detNodeEdges = Node.connectEdges(self.detTags, self.detEdges)

    self.links = {}
    for tagName in self.detTags:
      self.links[tagName] = []
    visited = []
    for n, c in detNodeEdges.items():
      if len(c[0]) > 1:
        psi = [0,]*len(c[0])
        s = [0,]*len(c[0])
        for i, e in enumerate(c[0]):
          s[i] = self.detEdges[e].p.shape[1]
        z = min(s)
        a = self.detEdges[c[0][0]].p[0:2,0:z]
        m = [0,]*(len(c[0])-1)
        for i in range(1, len(c[0])):
          b = self.detEdges[c[0][i]].p[0:2,0:z]
          g = np.sum((b - a)**2, 0)
          m[i-1] = np.min(np.where(g > 0.000001))
        m = max(m)
        for i, e in enumerate(c[0]):
          d = np.diff(self.detEdges[c[0][i]].p[0:2,[max([0,m-1]),m]])
          psi[i] = atan2(d[1], d[0])
        for i in range(1, len(psi)):
          psi[i] = wrapToPi(psi[i] - psi[0])
        psi[0] = 0.0
        o = sorted(range(len(psi)), key=lambda k: psi[k])
        oo = [None,]*len(o)
        for l, v in enumerate(reversed(o)):
          oo[l] = c[0][v]

        ab = detEdgeNodes[oo[0]]
        self.links[ab[0]].append(('L', ab[1], oo[0]))
        for l in range(1,len(oo)-1):
          ab = detEdgeNodes[oo[l]]
          self.links[ab[0]].append(('X', ab[1], oo[l]))
        ab = detEdgeNodes[oo[-1]]
        self.links[ab[0]].append(('R', ab[1], oo[-1]))
        
        for v in c[0]:
          if v not in visited:
            visited.append(v)
        
      if len(c[1]) > 1:
        psi = [0,]*len(c[1])
        s = [0,]*len(c[1])
        for i, e in enumerate(c[1]):
          s[i] = self.detEdges[e].p.shape[1]
        z = min(s)
        a = np.fliplr(self.detEdges[c[1][0]].p)[0:2,0:z]
        m = [0,]*(len(c[1])-1)
        for i in range(1, len(c[1])):
          b = np.fliplr(self.detEdges[c[1][i]].p)[0:2,0:z]
          g = np.sum((b - a)**2, 0)
          m[i-1] = np.min(np.where(g > 0.000001))
        m = max(m)
        for i, e in enumerate(c[1]):
          d = np.diff(np.fliplr(self.detEdges[c[1][i]].p)[0:2,[max([0,m-1]),m]])
          psi[i] = atan2(d[1], d[0])
        for i in range(1, len(psi)):
          psi[i] = wrapToPi(psi[i] - psi[0])
        psi[0] = 0.0
        o = sorted(range(len(psi)), key=lambda k: psi[k])
        oo = [None,]*len(o)
        for l, v in enumerate(o):
          oo[l] = c[1][v]

        ab = detEdgeNodes[oo[0]]
        self.links[ab[0]].append(('L', ab[1], oo[0]))
        for l in range(1,len(oo)-1):
          ab = detEdgeNodes[oo[l]]
          self.links[ab[0]].append(('X', ab[1], oo[l]))
        ab = detEdgeNodes[oo[-1]]
        self.links[ab[0]].append(('R', ab[1], oo[-1]))
        
        for v in c[1]:
          if v not in visited:
            visited.append(v)
          
    rest = set(self.detEdges.keys()) - set(visited)
    for r in rest:
      ab = detEdgeNodes[r]
      self.links[ab[0]].append(('X', ab[1], r))

    self.computeTagPoses()
    self.computeTagMap()

  def computeTagPoses(self):
    self.tagPoses = {}
    for tagName, tag in self.tags.items():
      detTag = self.detTags[tagName]
      if tag.isVirtual():
        self.tagPoses[tagName] = (tag.p[0], tag.p[1])
      else:
        self.tagPoses[tagName] = (tag.p[0], tag.p[1], atan2(detTag.p[1]-tag.p[1], detTag.p[0]-tag.p[0]))
      
  def computeTagMap(self):
    self.tagMap = {}
    for k, link in self.links.items():
      self.tagMap[k] = [0, 0.0, 0, 0.0]
      for s in link:
        e = self.detEdges[s[2]]
        l = e.length
        if s[0] == 'L':
          self.tagMap[k][0] = s[1]
          self.tagMap[k][1] = l
        elif s[0] == 'R':
          self.tagMap[k][2] = s[1]
          self.tagMap[k][3] = l
        elif s[0] == 'X':
          self.tagMap[k] += [s[1], l]

  def draw(self, include='', start=None, goal=None, path=[], nodes=False, tags=True, labels=True, direct=False):
    self.axes.cla()
    for p in world.SPACES:
      patch = mpl.patches.Polygon(p, color='w', fill=True)
      self.axes.add_patch(patch)
    Edge.drawEdges(self.edges, axes=self.axes, c='w', lw=55)
    
    if not path:
      self.drawLinks(include=include, direct=direct)
    else:
      self.drawPath(path)
    Edge.drawEdges(self.edges, axes=self.axes, dirs=nodes)
    if nodes:
      Node.drawNodes(self.nodes, axes=self.axes)
    if tags:
      self.drawTags(det=True)
      if labels:
        self.drawTagLabels()
    self.drawPoint(goal, 'Goal', 'lawngreen')
    self.drawPoint(start, 'Start', 'salmon')
    self.drawAxes()

  def findClosestNode(self, p, nodes):
    opt = (None, 1.0**2)
    for k, v in nodes.items():
      d = (v.p[0] - p[0])**2 + (v.p[1] - p[1])**2
      if d < opt[1]:
        opt = (k, d)
    if opt[0] is not None:
      return (opt[0], nodes[opt[0]].p)
    else:
      return None

  def createFigure(self, fig=plt.figure):
    self.fig = fig(figsize=[x/25.4/5*1000.0 for x in world.BOX[2:4]], facecolor=(0.9, 0.9, 0.9))
    self.axes = self.fig.add_subplot(111)

  def saveFigure(self, name):
    self.fig.savefig(name, facecolor=self.fig.get_facecolor(), edgecolor='none', dpi=128)

  def drawPoint(self, p, label='Goal', c='green'):
    if p is not None and None not in p:
      self.axes.plot(p[0], p[1], 's', c=c)
      ht = self.axes.text(p[0]+TEXT_OFFSET, p[1]+TEXT_OFFSET, label, ha='left', va='bottom', fontsize=10)
      ht.set_bbox(dict(facecolor=c, alpha=1.0, edgecolor='k'))

  def getPath(self, tags):
    p = np.zeros((2,0), dtype=float)
    for i in range(1, len(tags)):
      a = tags[i-1]
      b = tags[i]
      e = None
      if a in self.links:
        vv = self.links[a]
        for v in vv:
          if v[1] == b:
            e = v
            break
      if e is None:
        print('ERROR! Invalid path.') #TODO
        return None
      else:
        edge = self.detEdges[v[2]]
        p = np.hstack((p, edge.p[0:2,:]))
    return p

  def getPathEdges(self, tags):
    edges = []
    for i in range(1, len(tags)):
      a = tags[i-1]
      b = tags[i]
      e = None
      if a in self.links:
        vv = self.links[a]
        for v in vv:
          if v[1] == b:
            e = v
            break
      if e is None:
        print('ERROR! Invalid path.') #TODO
        return None
      else:
        edges.append(v[2])
    return edges

  def snapToEdge(self, point, filter=None):
    return Edge.snapToEdge(self.detEdges, point, filter=filter)

  def drawPath(self, path):
    for i in range(1, len(path)):
      a = path[i-1]
      b = path[i]
      e = None
      if a in self.links:
        vv = self.links[a]
        for v in vv:
          if v[1] == b:
            e = v
            break
      if e is None:
        print('ERROR! Invalid path.') #TODO
        return
      else:
        self.drawLink(v)

  def drawLink(self, v, direct=False):
    if v[0] == 'X':
      c = 'g'
    elif v[0] == 'L':
      c = 'b'
    elif v[0] == 'R':
      c = 'r'
    else:
      c = 'k'
    edge = self.detEdges[v[2]]
    edge.draw(axes=self.axes, c=c, lw=10, alpha=0.3, direct=direct)

  def drawLinks(self, include='LXR', direct=False):
    for k, vv in self.links.items():
      for v in vv:
        if v[0] in include:
          self.drawLink(v, direct)

  def drawTags(self, det=False):
    for tagName, tag in self.tags.items():
      c = 'yellow' if tag.isVirtual() else 'cyan'
      if det:
        detTag = self.detTags[tagName]
        self.axes.plot([tag.p[0], detTag.p[0]], [tag.p[1], detTag.p[1]], '-', c=c, lw=2, alpha=0.8)
        self.axes.plot(detTag.p[0], detTag.p[1], marker='x', c=c)
      self.axes.plot(tag.p[0], tag.p[1], 'o', c=c)

  def drawTagLabels(self):
    for tagName, tag in self.tags.items():
      c = 'yellow' if tag.isVirtual() else 'cyan'
      ht = self.axes.text(tag.p[0]+TEXT_OFFSET, tag.p[1]+TEXT_OFFSET, '{}'.format(tagName), ha='left', va='bottom', fontsize=10)
      ht.set_bbox(dict(facecolor=c, alpha=0.5, edgecolor='k'))



if __name__ == '__main__':
  graph = Graph()

  point = (0.500, 0.075) # Point (x, y)
  opt = graph.snapToEdge(point) # For a given point, find the closest point on the edges.
  # The format of the result is ('edge_name', distance_of_the_closest_point_on_the_edge_from_the_edge_start_point, (x, y, phi), distance_of_the_point_to_the_closest_point_on_the_edge)
  print('The point {} on the edge \'{}\' is the closest to the point {}; distance between the points is {} m. The point on the edge is {} m away from the edge start point.'.format(opt[2], opt[0], point, opt[3], opt[1]))

  path = [2, 11, 126, 131] # Path
  pathEdges = graph.getPathEdges(path) # Get path edges
  pathPoints = graph.getPath(path) # Get path points
  print('The edges of the path that goes through the tags {} are {}.'.format(path, pathEdges))
  opt = graph.snapToEdge(point, filter=pathEdges) # For a given point, find the closest point on the edges that comprise the path
  print('The point {} on the edge \'{}\' is the closest to the point {}; distance between the points is {} m. The point on the edge is {} m away from the edge start point.'.format(opt[2], opt[0], point, opt[3], opt[1]))
  for name in pathEdges: # Print the length of each path edge
    print('Length of the edge \'{}\' is {} m.'.format(name, graph.detEdges[name].length))

  #plt.close('all')

  rp = rospkg.RosPack()
  path = os.path.join(rp.get_path('amsagv'), 'doc', 'img')
  if not os.path.exists(path):
    os.makedirs(path)

  graph.createFigure()
  graph.draw(nodes=True, tags=False)
  graph.saveFigure(os.path.join(path, 'graph.png'))

  graph.createFigure()
  graph.draw()
  graph.saveFigure(os.path.join(path, 'graph-tags.png'))

  graph.createFigure()
  graph.draw(include='LXR')
  graph.saveFigure(os.path.join(path, 'graph-links.png'))

  graph.createFigure()
  graph.draw(path=[2, 11, 126, 131])
  graph.saveFigure(os.path.join(path, 'graph-path.png'))

  with open(os.path.join(rp.get_path('amsagv'), 'scripts', 'graph_gen.py'), 'w') as f:
    f.write('#!/usr/bin/python3\n# -*- coding: utf-8 -*-\n\n')
    f.write('tagMap = {\n')
    for i in graph.tagMap:
      one = graph.tagMap[i]
      f.write('  {:3d}: ({}),\n'.format(i, ', '.join('{:3d}, {:5.3f}'.format(one[j], one[j+1]) for j in range(0, len(one), 2))))
    f.write('}\n')
    f.write('\n')
    f.write('tagPoses = {\n')
    for i, v in graph.tagPoses.items():
      f.write('  {:3d}: ({}),\n'.format(i, ','.join('{:6.3f}'.format(x) for x in v)))
    f.write('}\n')
    f.write('\n')
    f.write('tagDets = {\n')
    for i, v in graph.detTags.items():
      f.write('  {:3d}: ({}),\n'.format(i, ','.join('{:6.3f}'.format(x) for x in v.p)))
    f.write('}\n')

    f.write('''
def findClosestNode(p, nodes=tagDets):
  opt = (None, 1.0**2)
  for k, v in nodes.items():
    d = (v[0] - p[0])**2 + (v[1] - p[1])**2
    if d < opt[1]:
      opt = (k, d)
  if opt[0] is not None:
    return (opt[0], nodes[opt[0]])
  else:
    return None
''')
