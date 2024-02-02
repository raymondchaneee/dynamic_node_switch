#!/usr/bin/env python

import rospy 
import rosnode
import rospkg
from dynamic_reconfigure.server import Server
from dynamic_node_switch.cfg import NodeSwitchConfig
from change_enum import change_enum_items
import libtmux
import time 
import glob
from hdl_graph_slam.srv import SaveMap, DumpGraph
import os
import numpy as np
import cv2
import csv

class DynamicNodeSwitch():
    def __init__(self):
        rospy.init_node("dynamic_reconfigure_demo")
        

        self.modules={  
                "navigation_module":{"Status":False,
                                     "PaneNumber": 2 ,
                                   "StartCmd":"roslaunch autonomy navigation_with_map.launch map_name:=$MAPNAME",
                                   "NodeName":["/move_base",
                                               "/map_server",
                                               "/ground_segmentation"]},
                "localization_module":{"Status":False,
                                       "PaneNumber": 1 ,
                                   "StartCmd":"roslaunch autonomy hdl_localization.launch map_name:=$MAPNAME",
                                   "NodeName":["/hdl_global_localization",
                                               "/hdl_localization_nodelet",
                                               "/globalmap_server_nodelet"]},
                "lidar_module":{"Status":False,
                                "PaneNumber": 3 ,
                                   "StartCmd":"roslaunch rslidar_sdk start.launch",
                                   "NodeName":["/rslidar_sdk_node"]},
                "Go1_controller_module":{"Status":False,
                                         "PaneNumber": 4 ,
                                   "StartCmd":" rosrun unitree_legged_real twist_sub",
                                   "NodeName":["/twist_sub"]},                
                "mapping_module":{"Status":False,
                                    "PaneNumber": 1 ,
                                   "StartCmd":"roslaunch hdl_graph_slam  hdl_graph_slam.launch",
                                   "NodeName":["/map2odom_publisher",
                                               "/hdl_graph_slam_nodelet"]},   
                "urdf_module":{"Status":False,
                               "PaneNumber": 5 ,
                                   "StartCmd":"roslaunch autonomy urdf_tf.launch",
                                   "NodeName":["/joint_state_publisher",
                                               "/robot_state_publisher"]}

              }
        self.mode_on_modules={
            "NAVIGATION_MODE":["navigation_module",
                                "localization_module",
                                "lidar_module",
                                "Go1_controller_module",
                                "urdf_module"
                                ],
            "ONLINE_MAPPING_MODE":["mapping_module",
                                "lidar_module",
                                "Go1_controller_module",
                                "urdf_module"],
            "CLOSE":[]
        }

        


        self.tmuxserver = libtmux.Server()
        mappath=rospkg.RosPack().get_path("autonomy")+"/map_data/*/"
        listdd=glob.glob(mappath)
        
        
        if (self.tmuxserver.has_session("go1")):
            print ("tmux window exist ")
            self.tmux_window=self.tmuxserver.find_where({ "session_name": "go1" }).find_where({ "window_name": "go1" })
            
            # self.start_node_cmd(1,"roslaunch autonomy urdf_tf.launch")
            
        else:
            print ("tmux window dose not exist ")
            raise 

        self.count=0
        self.countmap=2
        self.mapping_Name=""
        self.mode="CLOSE" 
        self.maplist=[]
        # time.sleep(10)
        # self.stop_node_cmd(1)
      
        _, updated_list=self.check_for_map_update()
        change_enum_items(NodeSwitchConfig,"navigation_map", updated_list,0)   
        self.selected_map=self.maplist[0]
        self.server = Server(NodeSwitchConfig, self.reconfigure_callback)
        timer = rospy.Timer(rospy.Duration(1), self.check_node_live_callback)
        addedMaptimer = rospy.Timer(rospy.Duration(10), self.update_map_available_list_callback)


        rospy.spin()
    def reconfigure_callback(self, config, level):
        # rospy.loginfo("Reconfiguration request: int_param=%d, double_param=%.2f, str_param=%s" %
        #             (config.int_param, config.double_param, config.str_param))
        # print (config)
        mode_="CLOSE"
        if (config.navigation_mode and not config.online_mapping_mode ):
            mode_="NAVIGATION_MODE" 
        elif (not config.navigation_mode and config.online_mapping_mode ):
            mode_="ONLINE_MAPPING_MODE" 
        
            
        # elif (not config.navigation_mode and not config.online_mapping_mode and config.offline_mapping_mode):
        #     self.mode="OFFLINE MAPPING MODE" 
        self.selected_map=self.maplist[config.navigation_map]
        self.mapping_Name=config.new_map_name
   
        if (self.mode!= mode_) :
            self.mode=mode_
            self.switching_mode(mode_)
            config.output_command="Switching mode to "+mode_

        if (config.generate_map==True):
            config.generate_map=False
            if config.new_map_name=="":
                config.output_command="ERROR: No Map Name, Fail to generate map"
            else:
                generate_map_status=self.generate_hdl_map_from_mapping()
                
                if not generate_map_status:
                    config.output_command="ERROR: hdl Service fail, Fail to generate map"
                else:
                    config.output_command="generate map successfully"
        return config

    def generate_hdl_map_from_mapping(self):
        try:
            rospy.wait_for_service('/hdl_graph_slam/dump',timeout=1.0)
            rospy.wait_for_service('/hdl_graph_slam/save_map',timeout=1.0)
            if self.mapping_Name not in self.maplist:
                path=rospkg.RosPack().get_path("autonomy")+"/map_data/"+self.mapping_Name
                os.mkdir(path)
                os.mkdir(path+"/dump")
                saveMapSrv = rospy.ServiceProxy('/hdl_graph_slam/save_map', SaveMap)
                dumpMapSrv = rospy.ServiceProxy('/hdl_graph_slam/dump', DumpGraph)
                resSaveMap = saveMapSrv(False, 0.05, path+"/map3d.pcd" )
                resDumpMap = dumpMapSrv( path+"/dump" )
                if resSaveMap.success and resDumpMap.success:
                    print("map save to "+self.mapping_Name+" successfully")
                    self.make_costmap_from_g2o_file( path+"/dump/graph.g2o",path)
                    return True
                else:
                    print ("FAil to Save Map? ")
                    return False
        except  Exception as e:
            print (e)
            return False    
    def make_costmap_from_g2o_file(self,g2o_file,save_map_location):
        #g2o file to point array
        np_g2o_pose=[]
        with open(g2o_file, 'r') as f:
            csvreader=csv.reader(f,delimiter=' ')
            for id, row in enumerate(csvreader):
                if "VERTEX_SE3:QUAT" in row[0]:
                    pose=[float(row[2]),float(row[3]),float(row[4])]
                    np_g2o_pose.append(pose)
        np_g2o_pose=np.array(np_g2o_pose)

        robotRadius=0.5
        resolution=0.05
        
        x_max=np.max(np_g2o_pose[:,0])+robotRadius*2
        y_max=np.max(np_g2o_pose[:,1])+robotRadius*2
        x_min=np.min(np_g2o_pose[:,0])-robotRadius*2
        y_min=np.min(np_g2o_pose[:,1])-robotRadius*2
        print ("x_max:",x_max, " x_min:",x_min," y_max:",y_max, " y_min:",y_min )

        costmap_size_x=int((x_max-x_min)/resolution)
        costmap_size_y=int((y_max-y_min)/resolution)
        print("costmap size:", costmap_size_x,costmap_size_y)

        costmap=np.zeros((costmap_size_y,costmap_size_x), dtype=np.uint8)
        origin=[x_min,y_min]
        costmap=self.draw_path_inflation_to_costmap(np_g2o_pose,costmap,origin,
                                            resolution, robotRadius )


        cv2.imwrite(save_map_location+"/map2d.pgm",costmap)

        yamlinput=(
        "image: "+"map2d.pgm"+"\n"
        +"resolution: "+str(resolution)+"\n"
        +"origin: "+"["+str(x_min) +","+str(y_min) +","+ str(0.000000)+"]"+"\n"
        +"negate: "+str(0)+"\n"
        +"occupied_thresh: "+str(0.65)+"\n"
        +"free_thresh: "+str(0.196)
        )
        f = open(save_map_location+"/map2d.yaml", "w")

        f.write(yamlinput)
        f.close()

    def draw_path_inflation_to_costmap(self,plans, costmap, 
                                        origin, resolution ,
                                        drawRobotRadius):

        costmap[:]=0 #fill all with black

        radius= int(drawRobotRadius/resolution)
        
        for id , pose in enumerate(plans):
            if id==0:
                continue
            pre_coordi=(int((plans[id-1][0]-origin[0])/resolution ), int((plans[id-1][1]-origin[1])/resolution))
            coordi=(int((pose[0]-origin[0])/resolution ), int((pose[1]-origin[1])/resolution))
            
            costmap = cv2.line (costmap, pre_coordi, coordi,  (255,255,255), radius*2) 
        
        costmap=np.flipud(costmap)
        costmap = np.ascontiguousarray(costmap, dtype=np.uint8)

        return  costmap
    def check_node_live_callback(self, timer):
        # self.count+=1
        # boolas=(self.count%2)==0
        # self.server.update_configuration({"mapping_mode":boolas, "output_command":"hello world"+str(self.count) })
        modules_status={}
        changes=0
        for key in self.modules.keys():
            (change,node_is_on)=self.check_modules_live(key)
            modules_status[key]=node_is_on
            changes+=int(change)
        if (changes>0):
            self.server.update_configuration(modules_status)
    
    def update_map_available_list_callback(self, timer):
        has_new_map_update, updated_list=self.check_for_map_update()
        if (has_new_map_update):
            change_enum_items(NodeSwitchConfig,"navigation_map", updated_list,1)     
            self.dynamic_Reconfig_server_restart()

    def dynamic_Reconfig_server_restart(self):
        self.server.set_service.shutdown("reseting")
        self.server.descr_topic.unregister()
        self.server.update_topic.unregister()
        self.server = Server(NodeSwitchConfig, self.reconfigure_callback)
    
    def check_for_map_update(self):
        map_path=rospkg.RosPack().get_path("autonomy")+"/map_data/*/"
        listdd=glob.glob(map_path)
        has_new_map_update=False
        for id, path in enumerate(listdd):
            mapname=self.get_folder_name(path)
            if mapname not in self.maplist:
                has_new_map_update=True
                self.maplist.append(mapname)

        newitem=[]
        for id, mapname in enumerate(self.maplist):
            newitem.append({'name':mapname, 'value':id, 'description':' map for'+mapname})
                      
        return has_new_map_update, newitem
            

    def get_folder_name(self,path):
        slashidx=path.rfind("/")
        newpath=path[:slashidx]
        slashidx=newpath.rfind("/")
        foldername=newpath[slashidx+1:]
        return foldername

            

    def check_modules_live(self, modulename):
        node_is_on,partial_node_is_on=self.check_node_availability(modulename)
        node_status_change=False
        if (self.modules[modulename]["Status"]!=node_is_on):
            node_status_change=True
            self.modules[modulename]["Status"]=node_is_on
        return (node_status_change,self.modules[modulename]["Status"])

    def switching_mode(self,mode):
        for key in self.modules.keys():
            #check if is on/off
            node_is_on,partial_node_is_on =self.check_node_availability(key)
            if key in self.mode_on_modules[mode]: # if modules must on   
                if node_is_on and partial_node_is_on:
                    continue
                elif not node_is_on and partial_node_is_on:
                    self.stop_node(key)
                    time.sleep(5)
                    print ("Switching ON "+key)
                    self.start_node(key)
                else:
                    print ("Switching ON "+key)
                    self.start_node(key)

            else: # if modules must off
                if partial_node_is_on:
                    print ("Switching OFF "+key)
                    self.stop_node(key)
                else:
                    continue

    def check_node_availability(self, modulename):
        node_name_list=self.modules[modulename]["NodeName"]
        status=True
        partial_status=0

        for nodename in node_name_list:
            status*=nodename in rosnode.get_node_names()
            partial_status+=int(nodename in rosnode.get_node_names())
        return status, partial_status>0
    
    
        
    def start_node(self, modulename):
        pane_id=self.modules[modulename]["PaneNumber"]
        cmd=self.modules[modulename]["StartCmd"]
        self.start_node_cmd(pane_id,cmd)
    def stop_node(self, modulename):
        pane_id=self.modules[modulename]["PaneNumber"]
        self.stop_node_cmd(pane_id)

    def start_node_cmd(self,pane_number, cmd):
        pane=self.tmux_window.get_by_id('%'+str(pane_number))
        #replace $MAPNAME with selected_map in cmd
        cmd=cmd.replace("$MAPNAME", self.selected_map) 
        pane.send_keys( cmd , enter=True )
    def stop_node_cmd(self,pane_number):
        pane=self.tmux_window.get_by_id('%'+str(pane_number))
        pane.send_keys('C-c', enter=False, suppress_history=False )
        
        
if __name__ == "__main__":
    DynamicNodeSwitch()
