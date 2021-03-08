"""Simple class for parsing and plotting OpenStreetMap XML files for
hand-in exercises in course TSFS12 Autonomous Vehicles -
control, planning, and learning

Erik Frisk <erik.frisk@liu.se>
Dept. of Electrical Engineering
Linköping  University
Sweden
"""

from xml.etree import ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.image import imread
import warnings
from misc import LatLongDistance, Timer
import os
import scipy.sparse as sp
import pickle


class OpenStreetMap:
    """Class for representing OpenStreetMap XML files."""

    bounds = {}
    nodes = {}
    ways = {}
    wayXMLindex = {}
    nodeindex = {}
    distancematrix = {}
    figdata = {}
    filename = ''
    nodeposition = {}
        
    def __init__(self, osmFileName=None, imgFileName=None):
        """Initialize OSMObject.

        Reads OpenStreetMap XML file and a corresponding image of the map.
        """
        if osmFileName is not None:
            self.filename = osmFileName
            root = ET.parse(osmFileName).getroot()
            self.parse_osm_tree(root)
            self.purge_nodes_not_on_ways()

            self.wayXMLindex = np.array(list(self.ways.keys()))
            self.nodeindex = np.array(list(self.nodes.keys()))
            self.compute_way_lengths()
            self.compute_distance_matrix()
            self.compute_node_position()

        if imgFileName is not None:
            if len(imgFileName) > 0:
                self.figdata['filename'] = imgFileName
                self.figdata['img'] = imread(imgFileName)

    def __str__(self):
        """Display map info."""
        dx = LatLongDistance((self.bounds['minlon'], self.bounds['minlat']),
                             (self.bounds['maxlon'], self.bounds['minlat']))
        dy = LatLongDistance((self.bounds['minlon'], self.bounds['minlat']),
                             (self.bounds['minlon'], self.bounds['maxlat']))
        s = 'Map (%s): %d nodes, %d ways, %.1fx%.1f m' % (
            os.path.basename(self.filename), len(self.nodes),
            len(self.ways), dx, dy)
        return s

    def pickle(self, fileName):
        """Store map object as a pickle.

        map.pickle(filename)
        """
        d = (self.bounds, self.nodes, self.ways, self.wayXMLindex,
             self.nodeindex, self.distancematrix,
             self.figdata['filename'], self.filename)
        with open(fileName, 'wb') as f:
            pickle.dump(d, f, pickle.HIGHEST_PROTOCOL)

    def compute_node_position(self):
        for k, nodeidx in enumerate(self.nodeindex):
             p = self.nodes[nodeidx]
             self.nodeposition[k] = (p['lon'], p['lat'])    
            
    def loadpickle(self, fileName):
        """Load map object from pickle.

        map.loadpickle(filename)
        """

        with open(fileName, 'rb') as f:
            d = pickle.load(f)
        self.bounds = d[0]
        self.nodes = d[1]
        self.ways = d[2]
        self.wayXMLindex = d[3]
        self.nodeindex = d[4]
        self.distancematrix = d[5]
        figFileName = d[6]
        self.figdata['filename'] = figFileName
        self.figdata['img'] = imread(figFileName)
        self.filename = d[7]
        self.compute_node_position()


    def info(self):
        """Print basic information about map."""
        print('OSMFile: ' + self.filename)
        print('Number of nodes: %d' % len(self.nodes))
        print('Number of roads: %d' % len(self.ways))
        print('Map bounds')
        print('  Lon: %f - %f' % (self.bounds['minlon'],
                                  self.bounds['maxlon']))
        print('  Lat: %f - %f' % (self.bounds['minlat'],
                                  self.bounds['maxlat']))

        dx = LatLongDistance((self.bounds['minlon'], self.bounds['minlat']),
                             (self.bounds['maxlon'], self.bounds['minlat']))
        dy = LatLongDistance((self.bounds['minlon'], self.bounds['minlat']),
                             (self.bounds['minlon'], self.bounds['maxlat']))
        print('  Size: %.1f x %.1f (m)' % (dx, dy))
        if type(self.figdata) is dict:
            print('Figure file: ' + self.figdata['filename'])
            imgSize = self.figdata['img'].shape[0:2]
            print('  Size: %d x %d (px)' % (imgSize[1], imgSize[0]))
        else:
            print('No map figure')

    def purge_nodes_not_on_ways(self):
        """Remove nodes not on any way."""
        nodeIds = list(self.nodes.keys())
        for wId in self.ways:
            nodeIds = [
                ni for ni in nodeIds if ni not in self.ways[wId]['nodes']]
        for n in nodeIds:
            self.nodes.pop(n)

    def parse_osm_tree(self, root):
        """Parse OSM XML file."""
        children = root.getchildren()

        for child in children:
            if child.tag == 'bounds':
                self.bounds = {}
                for bb in child.attrib:
                    self.bounds[bb] = float(child.attrib[bb])

            elif child.tag == 'node':
                self.parse_osm_node(child)
            elif child.tag == 'way':
                self.parse_osm_way(child)

    def parse_osm_node(self, node):
        """Parse node."""
        nodeInfo = {}
        nodeInfo['id'] = int(node.attrib['id'])
        nodeInfo['lat'] = float(node.attrib['lat'])
        nodeInfo['lon'] = float(node.attrib['lon'])
        self.nodes[nodeInfo['id']] = nodeInfo

    def parse_osm_way(self, node):
        """Parse osm_way tag."""
        road_vals = ['motorway', 'motorway_link', 'trunk', 'trunk_link',
                     'primary', 'primary_link', 'secondary', 'secondary_link',
                     'tertiary', 'road', 'residential', 'living_street',
                     'service', 'services', 'motorway_junction',
                     'unclassified']
        w = {'id': 0, 'maxspeed': -1, 'lanes': 0, 'name': '', 'carRoad': False}
        w['id'] = int(node.attrib['id'])
        w['nodes'] = []
        way_children = node.getchildren()
        for c in way_children:
            if c.tag == 'nd':
                w['nodes'].append(int(c.attrib['ref']))
            elif c.tag == 'tag':
                if c.attrib['k'] in ['lanes', 'maxspeed']:
                    w[c.attrib['k']] = int(c.attrib['v'])
                elif c.attrib['k'] == 'name':
                    w['name'] = c.attrib['v']
                elif c.attrib['k'] == 'highway':
                    w['carRoad'] = (c.attrib['v'] in road_vals)
        if w['carRoad']:
            self.ways[w['id']] = w

    def compute_way_lengths(self):
        """Compute individual way lengths."""
        wIdx = self.ways.keys()
        for wId in wIdx:
            wNodes = self.ways[wId]['nodes']
            n = np.zeros(len(wNodes))
            for k in range(1, len(wNodes)):
                p1 = self.nodes[wNodes[k-1]]
                p2 = self.nodes[wNodes[k]]
                p1xy = (p1['lon'], p1['lat'])
                p2xy = (p2['lon'], p2['lat'])

                n[k] = n[k-1] + LatLongDistance(p1xy, p2xy)
            self.ways[wId]['length'] = n

    def neighbours(self, idx):
        """Get index to neighbour nodes."""
        return np.argwhere(self.distancematrix[idx, :])[:, 1]

    def compute_distance_matrix(self):
        """Compute distance matrix."""
        n = len(self.nodeindex)
        self.distancematrix = np.zeros((n, n))
        for wId in self.ways:
            w = self.ways[wId]
            for k in range(0, len(w['nodes'])-1):
                # length from node[k]->node[k+1]
                dl = w['length'][k+1] - w['length'][k]
                nId1 = w['nodes'][k]
                nId2 = w['nodes'][k+1]
                idx1 = np.argwhere(self.nodeindex == nId1)[0, 0]
                idx2 = np.argwhere(self.nodeindex == nId2)[0, 0]
                self.distancematrix[idx1, idx2] = dl
        self.distancematrix = sp.csr_matrix(
            self.distancematrix + self.distancematrix.transpose())

    def plotmap(self):
        """Plot map."""
        c_adj = np.cos(np.pi/180*(self.bounds['maxlat'] +
                                  self.bounds['minlat'])/2.0)
        plt.imshow(self.figdata['img'],
                   extent=[self.bounds['minlon'], self.bounds['maxlon'],
                           self.bounds['minlat'], self.bounds['maxlat']],
                   aspect=1/c_adj)
        ax = plt.gca()
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_visible(False)
        ax.spines['bottom'].set_visible(False)

        ax.yaxis.set_ticks_position('left')
        ax.xaxis.set_ticks_position('bottom')

    def getnodecoordinates(self, nodeId):
        """Get longitude and latitude for a node."""
        return self.XMLNodeXY(self.nodeindex[nodeId])

    def getclosestnodeXMLindex(self, p):
        """Get XML index closest to a point."""
        minDist = np.inf
        nodeId = 0
        for k in self.nodes:
            d = LatLongDistance(p, self.XMLNodeXY(k))
            if d < minDist:
                minDist = d
                nodeId = k
        return nodeId

    def getwaysXMLid(self, pos):
        """Get XML id for a waynode (or closest to position)."""
        if len(pos) > 1:
            nodeId = self.getclosestnodeXMLindex(pos)
        else:
            nodeId = pos
        waysId = []
        for wi in self.ways:
            w = self.ways[wi]
            if nodeId in w['nodes']:
                waysId.append(w['id'])
        return waysId[0]

    def getclosestnodeindex(self, p):
        """Get index to node closest to position."""
        nodeId = self.getclosestnodeXMLindex(p)
        return np.argwhere(self.nodeindex == nodeId)[0, 0]

    def XMLNodeXY(self, id):
        """Get XML id for a node."""
        return (self.nodes[id]['lon'], self.nodes[id]['lat'])

    def NodeXY(self, id):
        """Get longitude and latitude for a node."""
        return (self.nodes[self.nodeindex[id]]['lon'],
                self.nodes[self.nodeindex[id]]['lat'])

    def getpositions(self):
        """Get position from a map."""
        warnings.filterwarnings("ignore", ".*GUI is implemented.*")
        return plt.ginput(1)

    def getmapposition(self):
        """Get node id and position of closest node in a map."""
        p = self.getpositions()[0]
        n = self.getclosestnodeindex(p)
        w = self.getnodeway(n)
        wName = None
        if w:
            wName = w['name']
        return {'id': n, 'pos': p, 'name': wName}

    def getnodeway(osm, node):
        """Get way dictionary from node index."""
        xmlNodeIndex = osm.nodeindex[node]
        wIdx = -1
        for wId in osm.ways:
            w = osm.ways[wId]
            if xmlNodeIndex in w['nodes']:
                wIdx = w['id']
                break
        if wIdx != -1:
            return osm.ways[wIdx]
        else:
            return None

    def getnodewayindices(osm, node):
        """Get way dictionaries from node index."""
        xmlNodeIndex = osm.nodeindex[node]
        wIdx = []
        for wId in osm.ways:
            w = osm.ways[wId]
            if xmlNodeIndex in w['nodes']:
                wIdx.append(w['id'])
        return wIdx

    def getplanwaynames(osm, plan):
        """Get a list of road names for a plan."""
        if plan:
            planWayIds = [osm.getnodewayindices(n) for n in plan]
            planWayIds[0] = [planWayIds[0][0]]  # For start, take first node
            # Boolean selection array for single way nodes
            idx = np.array([len(w) == 1 for w in planWayIds])

            pathWay = np.array([w[0] for w in planWayIds])[idx]
            planWayNames = []
            last = -1
            lastName = ''
            for w in pathWay:
                if (w != last and osm.ways[w]['name'] != lastName
                   and osm.ways[w]['name'] != ''):
                    last = w
                    lastName = osm.ways[w]['name']
                    planWayNames.append(osm.ways[w]['name'])
            return planWayNames
        else:
            return None

    def plotway(self, w, *args, **kwargs):
        """Plot a way segment."""
        self.plotpath(w['nodes'], *args, **kwargs)

    def plotallways(self, *args, **kwargs):
        """Plot all way segments in the object."""
        xl = plt.xlim()
        yl = plt.ylim()
        for wi in self.ways:
            self.plotway(self.ways[wi], *args, **kwargs)
        plt.xlim(xl)
        plt.ylim(yl)

    def plotplan(self, plan, *args, **kwargs):
        """Plot a given plan."""
        self.plotpath([self.nodeindex[ni] for ni in plan], *args, **kwargs)

    def plotpath(self, nodePath, *args, **kwargs):
        """Plot a path."""
        nodePos = np.array([self.XMLNodeXY(n) for n in nodePath])
        if len(args) == 0:
            plt.plot(nodePos[:, 0], nodePos[:, 1], 'b', **kwargs)
        else:
            plt.plot(nodePos[:, 0], nodePos[:, 1], *args, **kwargs)


def loadOSMmap(xmlfile, figurefile):
    """Load OpenStreetMap XML file and parse into map object

    First time map object is saved into a file to speed-up next time.
    """
    _, osmFileName = os.path.split(xmlfile)

    if os.access(osmFileName + '.pickle', os.R_OK):
        osmMap = OpenStreetMap()
        osmMap.loadpickle(osmFileName + '.pickle')
    else:
        t = Timer()
        t.tic()
        print('Pre-parsed file does not exist, parsing XML file')
        print('This will take a while ...')
        osmMap = OpenStreetMap(xmlfile, figurefile)
        print('Done parsing the XML file in %.2f seconds\n' % t.toc())
        print('Saving data in file ' + osmFileName + '.pickle')
        osmMap.pickle(osmFileName + '.pickle')
    return osmMap
