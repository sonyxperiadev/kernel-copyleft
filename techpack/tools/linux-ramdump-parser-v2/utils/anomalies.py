# ============================================================================
# Copyright (c) 2021, The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# anomalies.py : Provides utility to record/read anomalies in .json format
#
# Output File: anomalies.json


import os
import sys
import json
import os.path
from collections import OrderedDict
from os import path
import atexit

class Anomaly:
    def __init__(self):
        try:
            self.anomalies_list = []
            self.output_dir = ""
            atexit.register( self.writeAnomaly )

        except Exception as err:
            print( str( err ) )

    def setOutputDir(self,output_dir):
        self.output_dir = output_dir


    def addErr(self, groupname, filename, anomaly):
        self.addAnomaly(groupname,filename,anomaly,"Error")

    def addWarning(self, groupname, filename, anomaly):
        self.addAnomaly( groupname, filename, anomaly, "Warning" )

    def addAnomaly(self,groupname,filename,anomaly,anomalytype):
        try:
            groupname = groupname.upper()

            dict2 = OrderedDict(
                [("filename", ''), ("anomaly", ''), ("level", '')] )
            dict2["filename"] = filename
            if len(anomaly) > 200 :
                anomaly = anomaly[:200]
            dict2["anomaly"] = anomaly
            dict2["level"] = anomalytype

            existing_keys = [item["Group"] for item in self.anomalies_list]
            if groupname in existing_keys:
                for index in range( len( self.anomalies_list ) ):
                    if self.anomalies_list[index]["Group"] == groupname:
                        self.anomalies_list[index]["Anomalies"].append( dict2 )
            else:
                self.anomalies_list.append(
                    {"Group": groupname, "Anomalies": [dict2]} )
        except Exception as err:
            print( str( err ) + "\n" )

    def getAll(self, output_dir):
        try:
            ret_data = None
            if os.path.isfile(
                    os.path.join( self.output_dir, 'anomalies.json' ) ):
                if (os.path.getsize( os.path.join( self.output_dir,
                                                   'anomalies.json' ) ) > 0):
                    with open( os.path.join( self.output_dir, 'anomalies.json' ),
                           'r' ) as fd:
                        ret_data = json.loads( fd.read() )
            return ret_data
        except Exception as err:
            print( str( err ) + "\n" )

    def update_file_data(self,file_data):
        try:
            existing_keys = [item["Group"] for item in file_data]
            for item in self.anomalies_list:
                if item["Group"] in existing_keys:
                    for index in range(len(file_data)):
                        if file_data[index]["Group"] == item["Group"]:
                            for item2 in file_data[index]["Anomalies"] :
                                for item3 in item["Anomalies"] :
                                    if item3["anomaly"] == item2["anomaly"] \
                                            and item3["filename"] == item2["filename"]:
                                        item["Anomalies"].remove(item3)
                            if len( item ):
                                file_data[index]["Anomalies"] += item["Anomalies"]
                else:
                    file_data.append(item)
            return file_data
        except Exception as err:
            print( str( err ) + "\n" )

    def writeAnomaly(self):
        try:
            if bool( self.anomalies_list ):
                if os.path.isfile(
                        os.path.join( self.output_dir, 'anomalies.json' ) ):
                    if (os.path.getsize( os.path.join( self.output_dir,
                                                       'anomalies.json' ) ) > 0):
                        with open( os.path.join( self.output_dir, 'anomalies.json' ),
                               'r+' ) as file:
                            file_data = json.load( file )
                            file_data = self.update_file_data(file_data)
                            file.seek( 0 )
                            json.dump( file_data, file, indent=4 )
                    else:
                        with open( os.path.join( self.output_dir,
                                                 'anomalies.json' ),
                                   'w' ) as fd:
                            json.dump( self.anomalies_list, fd, indent=4 )
                else:
                    with open(os.path.join(self.output_dir,
                                           'anomalies.json'),
                              'w') as fd:
                        json.dump(self.anomalies_list, fd, indent=4)
        except Exception as err:
            print( str( err ) + "\n" )
