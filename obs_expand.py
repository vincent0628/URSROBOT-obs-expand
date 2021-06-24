# -*- coding: utf-8 -*-
"""
Created on Thu Jun 24 15:02:16 2021

@author: vincent

灑水頭負邊界產生
input: 包含「車子經緯度、車子heading、車頭bias(公尺)」的 ros message (自訂)
output:負邊界座標 shape files(.txt,.shx,.shp,.dbf)
paramter: 灑水頭大小(負邊界半徑)

"""
import numpy as np
import utm
from pyproj import Proj
from shapely.geometry import Point
import shapefile

def writeShapeFile(output, folder_name='test'):
    w = shapefile.Writer(folder_name)
    w.multipoint(output)
    w.field('WAYPOINT', 'C', '40')
    w.record('Waypoint')
    w.close()


class obstacle:
    def __init__(self, mower_gps, heading_degree, heading_bias, 
                 obstacle_radius=0.5, output_path=''):
        """" input """
        self.mower_gps = mower_gps
        self.heading_degree = heading_degree
        self.heading_bias = heading_bias
        self.obstacle_radius = obstacle_radius
        self.output_path = output_path
        
        """" output """
        self.mower_utm = []
        self.center_utm = []
        self.center_gps = []
       
        
    def obs_expand(self):
        _, _, zone, _ = utm.from_latlon(self.mower_gps[0],self.mower_gps[1])
        proj = Proj(proj="utm", zone=zone, ellps="WGS84", preserve_units=False)
        ux, uy = proj(self.mower_gps[1], self.mower_gps[0])
        self.mower_utm = (ux,uy)
        
        x_bias = self.heading_bias*np.cos(np.radians(self.heading_degree))
        y_bias = self.heading_bias*np.sin(np.radians(self.heading_degree))
               
        
        self.center_utm = (ux+x_bias, uy+y_bias)
        boundary_poly = Point(self.center_utm).buffer(self.obstacle_radius)
        self.boundary_utm = boundary_poly.exterior.coords[:]
        lng, lat = proj(*zip(*self.boundary_utm), inverse=True)
        self.boundary_gps = list(zip(lat,lng))
    
    def output(self):
        if self.output_path:
            filename = self.output_path
        else:
            from datetime import datetime
            filename = 'output/'+datetime.now().strftime("%Y%m%d_%H_%M_%S")
        
        """" write waypoint shapefile"""
        writeShapeFile(self.boundary_gps, folder_name=filename)
        wx, wy = zip(*self.boundary_gps)
        
        """ write waypoint to txt"""
        with open(filename + ".txt", "w") as f:
            for i in range(len(wx)):
                f.write("%.8f %.8f\n" % (wx[i], wy[i]))
        # print("completed with results: {}".format(filename + ".txt"))


    def plot(self):
        import matplotlib.pyplot as plt        
        plt.figure(figsize=(8, 6))
        plt.plot(*zip(*self.boundary_utm),'-k', label = 'obstacle boundary')
        plt.plot(*self.center_utm,'*k', label = 'obstacle center')
        plt.plot(*self.mower_utm,'o',color='green', label = 'mower')
        plt.text(*self.mower_utm,f'GPS = {self.mower_gps}')
        plt.legend(bbox_to_anchor=(0,1.05,1,0.2), loc="lower left",
                mode="expand", borderaxespad=0, ncol=3, fontsize=15)     
        plt.grid(True)
        plt.axis('equal')
        plt.setp(plt.xticks()[1], ha='right')
        plt.ylabel('UTM Y [m]', fontsize=22)
        plt.xticks(fontsize=18 )
        plt.xlabel('UTM X [m]', fontsize=22)
        plt.yticks(fontsize=18 )
        text = f'heading_degree:{self.heading_degree},\
                heading_bias:{self.heading_bias},\
                obstacle_radius:{self.obstacle_radius}', 
        plt.figtext(0.5, 0.005, text, ha="center", fontsize=12, color='r' )
        plt.tight_layout()
        plt.savefig('output.png')


if __name__ == "__main__":
    
    PLOT = True
    PLOT = False
    """ Required """
    mower_gps = [25.6215,121.5595]
    heading_degree = 90 # degrees
    heading_bias = 1 # meter

    """ Optional """
    obstacle_radius = 0.5 # meter 半徑
    output_path = 'output/a'
    
    sprinkler = obstacle(mower_gps, heading_degree, heading_bias, 
                         obstacle_radius=0.5, output_path=output_path)
    sprinkler.obs_expand()
    sprinkler.output()
    if PLOT: sprinkler.plot()